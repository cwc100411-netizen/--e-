"""
MaixCam 一体程序：
1. 检测数字 1~5，并在屏幕上显示锁定结果。
2. 检测红色激光点和黑色矩形框四角，通过串口发送给 STM32。

依赖:
- digit_detector.py
- reference_digit_1.jpg ~ reference_digit_5.jpg

串口协议:
- 只有激光点时：FF + laser_x + laser_y + FE
- 激光点 + 黑框四角时：FF + laser_x + laser_y + x1+y1+...+x4+y4 + FE
- 数字目标点锁定后：FD + digit1_x + digit1_y + ... + digit5_x + digit5_y + FE
"""

from maix import app, camera, display, err, gpio, image, pinmap, sys, time, uart
import cv2
from digit_detector import DigitDetector


# STM32 端 Tracking.c 当前按 240x240 坐标处理，MaixCam 这里保持一致。
FRAME_WIDTH = 240
FRAME_HEIGHT = 240

# 数字检测间隔，适当降低 CPU 负载。
DETECT_INTERVAL_MS = 120

# ============================================================
# 数字检测 ROI：只检测该矩形框内的数字，忽略框外杂物
# ============================================================
DIGIT_ROI_W = 150
DIGIT_ROI_H = 160
DIGIT_ROI_OFFSET_X = 20
DIGIT_ROI_OFFSET_Y = 15
DIGIT_ROI_X = (FRAME_WIDTH - DIGIT_ROI_W) // 2 + DIGIT_ROI_OFFSET_X
DIGIT_ROI_Y = (FRAME_HEIGHT - DIGIT_ROI_H) // 2 + DIGIT_ROI_OFFSET_Y
DIGIT_ROI_COLOR = image.COLOR_WHITE
DIGIT_ROI_THICKNESS = 3

# ============================================================
# 激光点 + 黑色矩形框识别参数，来自 reference_red_green_laser.py
# ============================================================
TRACKING_ROI = [53, 47, 154, 154]
RED_THRESH = [(0, 100, 14, 51, -8, 17)]

RECT_MIN_AREA = 4000
RECT_APPROX_RATE = 0.02
RECT_CANNY_LOW = 70
RECT_CANNY_HIGH = 180
RECT_DETECT_INTERVAL = 3
PIXELS_THRESHOLD = 2
MERGE_MARGIN = 3
RECT_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

SHOW_IMAGE = True
PRINT_PERIOD_MS = 200

BAUDRATE = 115200
FRAME_HEAD = 0xFF
DIGIT_FRAME_HEAD = 0xFD
FRAME_TAIL = 0xFE
DIGIT_TARGET_SEND_PERIOD_MS = 500

# ============================================================
# A29 按键：每按下一次切换一种串口发送模式
# 接法：A29 接按键一端，按键另一端接 GND，按下为低电平。
# ============================================================
BUTTON_PIN = "A29"
BUTTON_GPIO_NAME = "GPIOA29"
BUTTON_DEBOUNCE_MS = 200

SEND_MODE_LASER_ONLY = 0
SEND_MODE_LASER_RECT = 1
SEND_MODE_LASER_DIGIT = 2
SEND_MODE_COUNT = 3

# 上电后的初始发送模式：只发送激光点。
INITIAL_SEND_MODE = SEND_MODE_LASER_ONLY


def rgb_color(r, g, b):
    # MaixCam 绘图推荐使用 image.Color；旧版本不支持时退回 RGB 元组。
    try:
        return image.Color.from_rgb(r, g, b)
    except Exception:
        return (r, g, b)


TRACKING_ROI_COLOR = rgb_color(255, 255, 0)
CORNER_COLOR = rgb_color(0, 255, 0)
LASER_COLOR = rgb_color(255, 0, 0)


def create_camera():
    # 兼容新版 Camera 对象和旧版 camera.capture。
    if hasattr(camera, "Camera"):
        return camera.Camera(FRAME_WIDTH, FRAME_HEIGHT)
    return None


def create_display():
    # 兼容新版 Display 对象和旧版 display.show。
    if hasattr(display, "Display"):
        return display.Display()
    return None


def open_uart():
    # 根据 MaixCam 型号选择串口 TX 引脚，保持参考程序的接线方式。
    if sys.device_id().lower() == "maixcam2":
        err.check_raise(pinmap.set_pin_function("A21", "UART4_TX"))
        return uart.UART("/dev/ttyS4", BAUDRATE)

    err.check_raise(pinmap.set_pin_function("A19", "UART1_TX"))
    return uart.UART("/dev/ttyS1", BAUDRATE)


def create_button():
    # A29 配置为上拉输入，按键按下时读到 0。
    err.check_raise(pinmap.set_pin_function(BUTTON_PIN, BUTTON_GPIO_NAME))
    return gpio.GPIO(BUTTON_GPIO_NAME, gpio.Mode.IN, gpio.Pull.PULL_UP)


def get_send_mode_name(send_mode):
    if send_mode == SEND_MODE_LASER_RECT:
        return "laser+rect"
    if send_mode == SEND_MODE_LASER_DIGIT:
        return "laser+digit"
    return "laser"


def update_send_mode(button, send_mode, last_button_state, last_button_ms, now):
    # 检测按键从松开到按下的边沿，并做简单消抖。
    if button is None:
        return send_mode, last_button_state, last_button_ms, False

    try:
        button_state = button.value()
    except Exception as exc:
        print("[button] 读取失败:", exc)
        return send_mode, last_button_state, last_button_ms, False

    changed = False
    if last_button_state == 1 and button_state == 0:
        if now - last_button_ms >= BUTTON_DEBOUNCE_MS:
            send_mode = (send_mode + 1) % SEND_MODE_COUNT
            last_button_ms = now
            changed = True
            print("[button] 发送模式:", get_send_mode_name(send_mode))

    return send_mode, button_state, last_button_ms, changed


def get_attr(obj, name, default=None):
    # 同时兼容对象方法、对象属性和字典字段。
    if obj is None:
        return default
    if isinstance(obj, dict):
        return obj.get(name, default)

    attr = getattr(obj, name, default)
    if callable(attr):
        return attr()
    return attr


def capture_image(cam):
    # 采集一帧图像。
    if cam is not None:
        return cam.read()
    return camera.capture()


def show_image(disp, img):
    # 显示调试画面，显示失败不影响识别和串口发送。
    if not SHOW_IMAGE:
        return

    try:
        if disp is not None:
            try:
                disp.show(img, fit=image.Fit.FIT_CONTAIN)
            except Exception:
                disp.show(img)
        else:
            display.show(img)
    except Exception as exc:
        print("[display] 显示失败:", exc)


def blob_area(blob):
    # 不同 MaixPy 版本可能叫 pixels 或 area。
    pixels = get_attr(blob, "pixels", None)
    if pixels is not None:
        return pixels
    return get_attr(blob, "area", 0)


def blob_center(blob):
    # 获取激光色块中心。
    cx = get_attr(blob, "cx", None)
    cy = get_attr(blob, "cy", None)
    if cx is not None and cy is not None:
        return int(cx), int(cy)

    x = get_attr(blob, "x", 0)
    y = get_attr(blob, "y", 0)
    w = get_attr(blob, "w", 0)
    h = get_attr(blob, "h", 0)
    return int(x + w / 2), int(y + h / 2)


def find_laser(img):
    # 只在 TRACKING_ROI 内找最大红色色块作为激光点。
    try:
        blobs = img.find_blobs(
            RED_THRESH,
            roi=TRACKING_ROI,
            merge=True,
            pixels_threshold=PIXELS_THRESHOLD,
            margin=MERGE_MARGIN,
        )
    except TypeError:
        try:
            blobs = img.find_blobs(
                RED_THRESH,
                TRACKING_ROI,
                merge=True,
                pixels_threshold=PIXELS_THRESHOLD,
                margin=MERGE_MARGIN,
            )
        except Exception:
            return None
    except Exception:
        return None

    if not blobs:
        return None

    best_blob = blobs[0]
    for blob in blobs:
        if blob_area(blob) > blob_area(best_blob):
            best_blob = blob

    return blob_center(best_blob)


def rect_area(points):
    # 用四边形面积筛掉小噪声矩形。
    area = 0
    for i in range(4):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % 4]
        area += x1 * y2 - x2 * y1

    if area < 0:
        area = -area
    return area


def find_contours(edges):
    # 兼容不同 OpenCV 版本的 findContours 返回值。
    result = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 3:
        return result[1]
    return result[0]


def find_black_rect(img):
    # 使用 OpenCV 边缘轮廓法识别黑色矩形框，返回 4 个顶点坐标。
    x, y, w, h = TRACKING_ROI

    try:
        img_cv = image.image2cv(img, copy=False)
        roi_cv = img_cv[y:y + h, x:x + w]
        gray = cv2.cvtColor(roi_cv, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, RECT_KERNEL)
        edges = cv2.Canny(gray, RECT_CANNY_LOW, RECT_CANNY_HIGH)
        contours = find_contours(edges)
    except Exception as exc:
        print("[tracking] 黑框识别失败:", exc)
        return None

    best_points = None
    best_area = 0
    for contour in contours:
        contour_area = cv2.contourArea(contour)
        if contour_area < RECT_MIN_AREA:
            continue

        epsilon = RECT_APPROX_RATE * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) != 4:
            continue

        points = []
        for point in approx:
            px, py = point.ravel()
            points.append((int(px) + x, int(py) + y))

        area = rect_area(points)
        if area > best_area:
            best_area = area
            best_points = points

    return best_points


def to_byte(value):
    # 串口每个坐标只占 1 字节，范围限制在 0~255。
    return max(0, min(255, int(value)))


def send_tracking_to_uart(serial, laser, rect):
    # STM32 端已支持 2 字节短包和 10 字节完整包。
    if serial is None or laser is None:
        return

    numbers = [laser[0], laser[1]]
    if rect is not None:
        for point in rect:
            numbers.append(point[0])
            numbers.append(point[1])

    frame = bytes([FRAME_HEAD]) + bytes([to_byte(n) for n in numbers]) + bytes([FRAME_TAIL])
    try:
        serial.write(frame)
    except Exception as exc:
        print("[uart] 发送失败:", exc)


def send_digit_targets_to_uart(serial, locked):
    # 新增：数字 1~5 全部锁定后，按 1->5 顺序发送中心点给 STM32 保存。
    if serial is None:
        return
    for value in range(1, 6):
        if value not in locked:
            return

    numbers = []
    for value in range(1, 6):
        digit = locked[value]
        numbers.append(digit["cx"])
        numbers.append(digit["cy"])

    frame = bytes([DIGIT_FRAME_HEAD]) + bytes([to_byte(n) for n in numbers]) + bytes([FRAME_TAIL])
    try:
        serial.write(frame)
    except Exception as exc:
        print("[uart] 数字目标发送失败:", exc)


def draw_line(img, x1, y1, x2, y2, color, thickness=1):
    # 兼容不同 MaixPy 版本的 draw_line 参数写法。
    try:
        img.draw_line(x1, y1, x2, y2, color=color, thickness=thickness)
    except TypeError:
        img.draw_line(x1, y1, x2, y2, color, thickness)


def draw_cross(img, x, y, color, size=6, thickness=1):
    # 兼容不同 MaixPy 版本的 draw_cross 参数写法。
    try:
        img.draw_cross(x, y, color=color, size=size, thickness=thickness)
    except TypeError:
        img.draw_cross(x, y, color, size, thickness)


def draw_line_rect(img, roi, color):
    # 每帧画出追踪 ROI，方便在屏幕上确认识别范围。
    x, y, w, h = roi
    try:
        img.draw_rect(x, y, w, h, color, 2)
        return
    except Exception:
        pass

    draw_line(img, x, y, x + w, y, color, thickness=2)
    draw_line(img, x + w, y, x + w, y + h, color, thickness=2)
    draw_line(img, x + w, y + h, x, y + h, color, thickness=2)
    draw_line(img, x, y + h, x, y, color, thickness=2)


def draw_tracking_debug(img, rect, laser):
    # 标注追踪 ROI、黑框四角和激光点。
    try:
        draw_line_rect(img, TRACKING_ROI, color=TRACKING_ROI_COLOR)

        if rect is not None:
            for i in range(4):
                x1, y1 = rect[i]
                draw_cross(img, x1, y1, color=CORNER_COLOR, size=6, thickness=2)

        if laser is not None:
            x, y = laser
            draw_cross(img, x, y, color=LASER_COLOR, size=6, thickness=2)
    except Exception as exc:
        print("[tracking] 绘图失败:", exc)


def print_status(laser, rect, fps, locked_count):
    # 定时打印关键状态，便于串口终端观察。
    if laser is None:
        laser = (-1, -1)
    rect_state = "rect=Y" if rect is not None else "rect=N"
    print("laser=%d,%d %s digit=%d/5 fps=%.1f" %
          (laser[0], laser[1], rect_state, locked_count, fps))


def draw_digit_roi_border(img, roi_x, roi_y, roi_w, roi_h):
    """在图像上绘制数字检测 ROI 边框与四角标记。"""
    img.draw_rect(roi_x, roi_y, roi_w, roi_h, color=DIGIT_ROI_COLOR, thickness=DIGIT_ROI_THICKNESS)

    corner_len = 12
    img.draw_line(roi_x, roi_y, roi_x + corner_len, roi_y, color=DIGIT_ROI_COLOR, thickness=DIGIT_ROI_THICKNESS)
    img.draw_line(roi_x, roi_y, roi_x, roi_y + corner_len, color=DIGIT_ROI_COLOR, thickness=DIGIT_ROI_THICKNESS)
    img.draw_line(roi_x + roi_w - corner_len, roi_y, roi_x + roi_w, roi_y,
                  color=DIGIT_ROI_COLOR, thickness=DIGIT_ROI_THICKNESS)
    img.draw_line(roi_x + roi_w, roi_y, roi_x + roi_w, roi_y + corner_len,
                  color=DIGIT_ROI_COLOR, thickness=DIGIT_ROI_THICKNESS)
    img.draw_line(roi_x, roi_y + roi_h - corner_len, roi_x, roi_y + roi_h,
                  color=DIGIT_ROI_COLOR, thickness=DIGIT_ROI_THICKNESS)
    img.draw_line(roi_x, roi_y + roi_h, roi_x + corner_len, roi_y + roi_h,
                  color=DIGIT_ROI_COLOR, thickness=DIGIT_ROI_THICKNESS)
    img.draw_line(roi_x + roi_w - corner_len, roi_y + roi_h, roi_x + roi_w, roi_y + roi_h,
                  color=DIGIT_ROI_COLOR, thickness=DIGIT_ROI_THICKNESS)
    img.draw_line(roi_x + roi_w, roi_y + roi_h - corner_len, roi_x + roi_w, roi_y + roi_h,
                  color=DIGIT_ROI_COLOR, thickness=DIGIT_ROI_THICKNESS)


def draw_digit_overlays(img, digits, locked_values=None):
    """在图像上绘制检测到的数字。"""
    if locked_values is None:
        locked_values = set()

    for digit in digits:
        v = digit["value"]
        is_locked = v in locked_values

        outer_x1 = digit.get("outer_x", digit["x"])
        outer_y1 = digit.get("outer_y", digit["y"])
        outer_x2 = outer_x1 + digit.get("outer_w", digit["w"])
        outer_y2 = outer_y1 + digit.get("outer_h", digit["h"])
        x1 = digit["x"]
        y1 = digit["y"]
        x2 = digit["x"] + digit["w"]
        y2 = digit["y"] + digit["h"]
        outer_cx = digit.get("outer_cx", digit["cx"])
        outer_cy = digit.get("outer_cy", digit["cy"])
        cx = digit["cx"]
        cy = digit["cy"]
        label_y = max(0, outer_y1 - 14)

        frame_color = image.COLOR_GREEN if is_locked else image.COLOR_YELLOW
        if is_locked:
            label = "[L] %d  %.2f" % (v, digit["score"])
        else:
            label = " %d  %.2f" % (v, digit["score"])

        img.draw_rect(outer_x1, outer_y1, max(1, outer_x2 - outer_x1), max(1, outer_y2 - outer_y1),
                      color=frame_color, thickness=2)
        img.draw_rect(x1, y1, max(1, x2 - x1), max(1, y2 - y1),
                      color=image.COLOR_BLUE, thickness=1)
        img.draw_cross(outer_cx, outer_cy, color=frame_color, size=6, thickness=2)
        img.draw_cross(cx, cy, color=image.COLOR_ORANGE, size=4, thickness=1)
        img.draw_string(outer_x1, label_y, label, color=frame_color, scale=1)


def _post_process_digits(detector, img, roi_digits, already_locked=None, locked_boxes=None):
    """数字后处理：去重 + 补漏，尽量确保 1~5 各检测到一个。"""
    if already_locked is None:
        already_locked = set()
    if locked_boxes is None:
        locked_boxes = []

    by_value = {}
    for d in roi_digits:
        v = d["value"]
        if v not in by_value or d["score"] > by_value[v]["score"]:
            by_value[v] = d

    result = list(by_value.values())
    found_values = set(by_value.keys())
    missing = sorted(set(range(1, 6)) - found_values - already_locked)
    if not missing:
        return result

    try:
        from maix import image as maix_image
        img_bgr = maix_image.image2cv(img, copy=False)
    except Exception:
        return result

    exclude_boxes = [(d["x"], d["y"], d["w"], d["h"]) for d in result]
    if locked_boxes:
        exclude_boxes.extend(locked_boxes)

    digit_roi = (DIGIT_ROI_X, DIGIT_ROI_Y, DIGIT_ROI_W, DIGIT_ROI_H)
    candidates = detector.find_unassigned_candidates(
        img_bgr, exclude_boxes, roi=digit_roi, min_score=0.28
    )

    assigned_values = set()
    for c in candidates:
        cv = c["value"]
        if cv in missing and cv not in assigned_values:
            result.append(c)
            assigned_values.add(cv)
            if len(assigned_values) == len(missing):
                break

    return result


def main():
    print("[main] MaixCam 数字检测 + 激光追踪启动中...")
    print("[main] 图像尺寸: %dx%d" % (FRAME_WIDTH, FRAME_HEIGHT))
    print("[digit] ROI: x=%d y=%d w=%d h=%d" %
          (DIGIT_ROI_X, DIGIT_ROI_Y, DIGIT_ROI_W, DIGIT_ROI_H))
    print("[tracking] ROI: x=%d y=%d w=%d h=%d" %
          (TRACKING_ROI[0], TRACKING_ROI[1], TRACKING_ROI[2], TRACKING_ROI[3]))

    try:
        disp = create_display()
        print("[main] 显示器就绪")
    except Exception as exc:
        disp = None
        print("[main] 显示器不可用:", exc)

    cam = create_camera()
    print("[main] 摄像头就绪")

    try:
        serial = open_uart()
        print("[main] 串口就绪: %d" % BAUDRATE)
    except Exception as exc:
        serial = None
        print("[main] 串口不可用，只显示不发送:", exc)

    try:
        button = create_button()
        print("[button] A29 按键就绪，当前发送模式:", get_send_mode_name(INITIAL_SEND_MODE))
    except Exception as exc:
        button = None
        print("[button] A29 按键不可用，保持初始发送模式:", exc)

    detector = DigitDetector(min_score=0.45)
    print("[digit] 数字检测器就绪, OpenCV:", "可用" if detector.available() else "不可用")

    last_digits = []
    last_detect_time = 0
    locked = {}
    all_locked = False

    last_print_ms = time.ticks_ms() - PRINT_PERIOD_MS
    last_digit_target_send_ms = 0
    frame_count = 0
    rect_detect_count = 0
    last_rect = None
    send_mode = INITIAL_SEND_MODE
    last_button_state = 1
    last_button_ms = 0

    print("[main] 开始主循环...")

    while not app.need_exit():
        frame_count += 1
        now = time.ticks_ms()

        img = capture_image(cam)

        send_mode, last_button_state, last_button_ms, mode_changed = update_send_mode(
            button, send_mode, last_button_state, last_button_ms, now
        )
        digit_mode = (send_mode == SEND_MODE_LASER_DIGIT)
        if mode_changed:
            last_digits = []
            if digit_mode:
                # 进入数字模式后重新开始锁定，避免使用上一次留下的旧坐标。
                locked = {}
                all_locked = False
                last_detect_time = 0
                last_digit_target_send_ms = 0

        # 只有按键切到 laser+rect 模式时，才运行黑色矩形框识别。
        if send_mode == SEND_MODE_LASER_RECT:
            if rect_detect_count == 0:
                rect = find_black_rect(img)
                if rect is not None:
                    last_rect = rect

            rect_detect_count += 1
            if rect_detect_count >= RECT_DETECT_INTERVAL:
                rect_detect_count = 0
        else:
            last_rect = None
            rect_detect_count = 0

        laser = find_laser(img)
        if send_mode == SEND_MODE_LASER_RECT:
            send_tracking_to_uart(serial, laser, last_rect)
        else:
            send_tracking_to_uart(serial, laser, None)

        if not digit_mode:
            # 不是数字中心点发送模式时，清除数字锁定状态，避免旧坐标残留。
            locked = {}
            all_locked = False
            last_digits = []
        elif all_locked:
            last_digits = [locked[v] for v in sorted(locked)]
        elif now - last_detect_time >= DETECT_INTERVAL_MS:
            last_detect_time = now
            try:
                all_digits = detector.detect_maix(img)
                if detector.last_error:
                    print("[digit] 检测状态:", detector.last_error)

                roi_digits = [
                    d for d in all_digits
                    if DIGIT_ROI_X <= d["cx"] <= DIGIT_ROI_X + DIGIT_ROI_W
                    and DIGIT_ROI_Y <= d["cy"] <= DIGIT_ROI_Y + DIGIT_ROI_H
                ]

                locked_values = set(locked.keys())
                locked_boxes = [(d["x"], d["y"], d["w"], d["h"]) for d in locked.values()]
                new_digits = _post_process_digits(
                    detector, img, roi_digits,
                    already_locked=locked_values,
                    locked_boxes=locked_boxes,
                )

                for d in new_digits:
                    v = int(d["value"])
                    if v not in locked:
                        locked[v] = d
                        print("[digit] 锁定 digit %d  cx=%d cy=%d  [%d/5]" %
                              (v, d["cx"], d["cy"], len(locked)))

                if len(locked) >= 5:
                    all_locked = True
                    last_digit_target_send_ms = now
                    print("[digit] ===== 全部锁定 =====")
                    for v in sorted(locked):
                        d = locked[v]
                        print("  digit %d: cx=%d cy=%d score=%.2f" %
                              (v, d["cx"], d["cy"], d["score"]))
                    if send_mode == SEND_MODE_LASER_DIGIT:
                        send_digit_targets_to_uart(serial, locked)

                if locked:
                    last_digits = [locked[v] for v in sorted(locked)]
                else:
                    last_digits = new_digits

            except Exception as exc:
                last_digits = [] if not locked else [locked[v] for v in sorted(locked)]
                print("[digit] 检测异常:", exc)

        # 数字 1~5 全部锁定后，周期发送 FD 10 字节包给 STM32。
        if (send_mode == SEND_MODE_LASER_DIGIT and all_locked
                and now - last_digit_target_send_ms >= DIGIT_TARGET_SEND_PERIOD_MS):
            last_digit_target_send_ms = now
            send_digit_targets_to_uart(serial, locked)

        draw_tracking_debug(img, last_rect, laser)
        if digit_mode:
            draw_digit_roi_border(img, DIGIT_ROI_X, DIGIT_ROI_Y, DIGIT_ROI_W, DIGIT_ROI_H)
            draw_digit_overlays(img, last_digits, locked_values=set(locked.keys()))

        if not digit_mode:
            status = get_send_mode_name(send_mode)
            status_color = image.COLOR_WHITE
        elif all_locked:
            status = "ALL LOCKED"
            status_color = image.COLOR_GREEN
        elif locked:
            status = "LOCKING %d/5" % len(locked)
            status_color = image.COLOR_YELLOW
        else:
            status = "SCAN..."
            status_color = image.COLOR_WHITE

        img.draw_string(4, 4, status, color=status_color, scale=1)
        if digit_mode and last_digits:
            vals = ",".join(str(int(d["value"])) for d in last_digits)
            img.draw_string(4, 22, "Found: [%s]" % vals,
                            color=image.COLOR_GREEN if all_locked else image.COLOR_YELLOW,
                            scale=1)

        elapsed_ms = now - last_print_ms
        if elapsed_ms >= PRINT_PERIOD_MS:
            fps = frame_count * 1000 / elapsed_ms
            locked_count = len(locked) if digit_mode else 0
            print_status(laser, last_rect, fps, locked_count)
            frame_count = 0
            last_print_ms = now

        show_image(disp, img)
        time.sleep_ms(10)


if __name__ == "__main__":
    main()
