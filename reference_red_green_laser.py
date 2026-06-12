from maix import app, camera, display, err, image, pinmap, sys, time, uart
import cv2


# 图像尺寸，当前摄像头输出 240x240
CAM_W, CAM_H = 240, 240

# ROI 区域：[x, y, w, h]，只在这个区域内找矩形黑框，可按实际画面调整
RECT_ROI = [52, 48, 161, 159]

# 红色激光阈值
RED_THRESH =  [(0, 100, 14, 51, -8, 17)]

# 黑框和激光识别参数
RECT_MIN_AREA = 4000
RECT_APPROX_RATE = 0.02
RECT_CANNY_LOW = 70
RECT_CANNY_HIGH = 180
RECT_DETECT_INTERVAL = 3
PIXELS_THRESHOLD = 2
MERGE_MARGIN = 3

# 调试显示和打印周期
SHOW_IMAGE = True
PRINT_PERIOD_MS = 200

# 串口协议：FF + 激光 x,y + 四个角点 x,y + FE
BAUDRATE = 115200
FRAME_HEAD = 0xFF
FRAME_TAIL = 0xFE


ROI_COLOR = image.COLOR_YELLOW
CORNER_COLOR = image.COLOR_GREEN
LASER_COLOR = image.COLOR_RED

# 参考工程使用 3x3 闭运算连接矩形边缘，减少断线造成的漏检
RECT_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))


def create_camera():
    # 兼容新版 MaixPy 的 Camera 对象和旧版 camera.capture
    if hasattr(camera, "Camera"):
        return camera.Camera(CAM_W, CAM_H)
    return None


def create_display():
    # 兼容新版 Display 对象和旧版 display.show
    if hasattr(display, "Display"):
        return display.Display()
    return None


def open_uart():
    # 根据设备选择串口发送引脚
    if sys.device_id().lower() == "maixcam2":
        err.check_raise(pinmap.set_pin_function("A21", "UART4_TX"))
        return uart.UART("/dev/ttyS4", BAUDRATE)

    err.check_raise(pinmap.set_pin_function("A19", "UART1_TX"))
    return uart.UART("/dev/ttyS1", BAUDRATE)


cam = create_camera()
disp = create_display()
serial = open_uart()


def get_attr(obj, name, default=None):
    # 同时兼容对象方法、对象属性和字典字段
    if obj is None:
        return default
    if isinstance(obj, dict):
        return obj.get(name, default)

    attr = getattr(obj, name, default)
    if callable(attr):
        return attr()
    return attr


def capture_image():
    # 采集一帧图像
    if cam is not None:
        return cam.read()
    return camera.capture()


def show_image(img):
    # 显示调试画面
    if not SHOW_IMAGE:
        return
    if disp is not None:
        disp.show(img)
    else:
        display.show(img)


def blob_area(blob):
    # 不同 MaixPy 版本可能叫 pixels 或 area
    pixels = get_attr(blob, "pixels", None)
    if pixels is not None:
        return pixels
    return get_attr(blob, "area", 0)


def blob_center(blob):
    # 获取激光色块中心
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
    # 只在 ROI 内找最大红色色块作为激光点
    try:
        blobs = img.find_blobs(
            RED_THRESH,
            roi=RECT_ROI,
            merge=True,
            pixels_threshold=PIXELS_THRESHOLD,
            margin=MERGE_MARGIN,
        )
    except TypeError:
        try:
            blobs = img.find_blobs(
                RED_THRESH,
                RECT_ROI,
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
    # 用四边形面积筛掉小噪声矩形
    area = 0
    for i in range(4):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % 4]
        area += x1 * y2 - x2 * y1

    if area < 0:
        area = -area
    return area


def find_contours(edges):
    # 兼容不同 OpenCV 版本的 findContours 返回值
    result = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 3:
        return result[1]
    return result[0]


def find_black_rect(img):
    # 参考工程方法：灰度化、模糊、闭运算、Canny 边缘，再用轮廓拟合找四边形
    x, y, w, h = RECT_ROI

    try:
        img_cv = image.image2cv(img, copy=False)
        roi_cv = img_cv[y:y + h, x:x + w]
        gray = cv2.cvtColor(roi_cv, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, RECT_KERNEL)
        edges = cv2.Canny(gray, RECT_CANNY_LOW, RECT_CANNY_HIGH)
        contours = find_contours(edges)
    except Exception as e:
        print("rect cv error:", e)
        return None

    best_points = None
    best_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < RECT_MIN_AREA:
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
    # 串口每个坐标只占 1 字节
    return max(0, min(255, int(value)))


def send_tracking_to_uart(laser, rect):
    # STM32 仍需要激光点和四个角点，所以只删除打印，不改串口协议
    if laser is None or rect is None:
        return

    numbers = [laser[0], laser[1]]
    for point in rect:
        numbers.append(point[0])
        numbers.append(point[1])

    frame = bytes([FRAME_HEAD]) + bytes([to_byte(n) for n in numbers]) + bytes([FRAME_TAIL])
    serial.write(frame)


def draw_line(img, x1, y1, x2, y2, color, thickness=1):
    # 兼容不同 MaixPy 版本的 draw_line 参数写法
    try:
        img.draw_line(x1, y1, x2, y2, color=color, thickness=thickness)
    except TypeError:
        img.draw_line(x1, y1, x2, y2, color, thickness)


def draw_cross(img, x, y, color, size=6, thickness=1):
    # 兼容不同 MaixPy 版本的 draw_cross 参数写法
    try:
        img.draw_cross(x, y, color=color, size=size, thickness=thickness)
    except TypeError:
        img.draw_cross(x, y, color, size, thickness)


def draw_line_rect(img, roi, color):
    # 每帧画出 ROI 区域，方便在屏幕上确认识别范围
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


def draw_debug(img, rect, laser):
    # 在屏幕中标注 ROI、四个角点和激光点
    try:
        draw_line_rect(img, RECT_ROI, color=ROI_COLOR)

        if rect is not None:
            for i in range(4):
                x1, y1 = rect[i]
                draw_cross(img, x1, y1, color=CORNER_COLOR, size=6, thickness=2)

        if laser is not None:
            x, y = laser
            draw_cross(img, x, y, color=LASER_COLOR, size=6, thickness=2)
    except Exception as e:
        # 绘图失败不影响识别和串口发送，但需要打印原因方便定位
        print("draw error:", e)


def print_status(laser, fps):
    # 只打印激光点和帧率
    if laser is None:
        laser = (-1, -1)
    print("%d %d | %.1f" % (laser[0], laser[1], fps))


print("laser_x laser_y | fps")
last_print_ms = time.ticks_ms() - PRINT_PERIOD_MS
frame_count = 0
rect_detect_count = 0
last_rect = None

while not app.need_exit():
    img = capture_image()
    frame_count += 1

    if rect_detect_count == 0:
        rect = find_black_rect(img)
        if rect is not None:
            last_rect = rect

    rect_detect_count += 1
    if rect_detect_count >= RECT_DETECT_INTERVAL:
        rect_detect_count = 0

    laser = find_laser(img)

    now_ms = time.ticks_ms()
    elapsed_ms = now_ms - last_print_ms
    if elapsed_ms >= PRINT_PERIOD_MS:
        fps = frame_count * 1000 / elapsed_ms
        print_status(laser, fps)
        frame_count = 0
        last_print_ms = now_ms

    send_tracking_to_uart(laser, last_rect)
    draw_debug(img, last_rect, laser)
    show_image(img)
