from maix import app, camera, display, err, pinmap, sys, time, uart


# 摄像头尺寸参考 notebook 输出，画面中心为 120,120
CAM_W, CAM_H = 240, 240

# LAB 阈值，参考 notebook 中的一号机 2.0 红色激光参数
RED_THRESH =  [(0, 100, 14, 51, -8, 17)]

# LAB 阈值，参考 Maix 示例：先二值化黑色区域，再查找矩形
BLACK_RECT_THRESH = [[0, 33, -100, 100, -100, 100]]
RECT_ROI = [20, 20, CAM_W - 40, CAM_H - 40]
RECT_THRESHOLD = 10000

# 红色激光点通常很小，阈值保持 notebook 的设置
PIXELS_THRESHOLD = 2
MERGE_MARGIN = 3

# 是否显示调试画面
SHOW_IMAGE = True

# 坐标打印周期，单位 ms；串口发送不受这个周期影响
PRINT_PERIOD_MS = 200

# 串口配置，参考“发送.py”
BAUDRATE = 115200
FRAME_HEAD = 0xFF
FRAME_TAIL = 0xFE


def create_camera():
    # 兼容新版 MaixPy 的 camera.Camera 和 V831 notebook 中的 camera.capture
    if hasattr(camera, "Camera"):
        return camera.Camera(CAM_W, CAM_H)
    return None


def create_display():
    # 兼容新版 Display 对象和旧版 display.show 模块函数
    if hasattr(display, "Display"):
        return display.Display()
    return None


cam = create_camera()
disp = create_display()


def open_uart():
    # 根据不同设备选择串口发送引脚
    if sys.device_id().lower() == "maixcam2":
        err.check_raise(pinmap.set_pin_function("A21", "UART4_TX"))
        return uart.UART("/dev/ttyS4", BAUDRATE)

    err.check_raise(pinmap.set_pin_function("A19", "UART1_TX"))
    return uart.UART("/dev/ttyS1", BAUDRATE)


serial = open_uart()


def capture_image():
    # 采集一帧图像
    if cam is not None:
        return cam.read()
    return camera.capture()


def show_image(img):
    # 显示调试图像
    if not SHOW_IMAGE:
        return
    if disp is not None:
        disp.show(img)
    else:
        display.show(img)


def read_attr(obj, name, default=None):
    # 同时支持字典式字段和对象方法/属性
    if obj is None:
        return default
    if isinstance(obj, dict):
        return obj.get(name, default)
    attr = getattr(obj, name, default)
    if callable(attr):
        return attr()
    return attr


def blob_pixels(blob):
    # notebook 使用 pixels 字段，部分 MaixPy 版本使用 area 方法
    pixels = read_attr(blob, "pixels", None)
    if pixels is not None:
        return pixels
    return read_attr(blob, "area", 0)


def find_max(blobs):
    # 找到像素面积最大的色块
    max_id = 0
    for i, blob in enumerate(blobs):
        if blob_pixels(blob) > blob_pixels(blobs[max_id]):
            max_id = i
    return max_id


def blob_center(blob):
    # 计算红色激光色块中心
    cx = read_attr(blob, "cx", None)
    cy = read_attr(blob, "cy", None)
    if cx is not None and cy is not None:
        return int(cx), int(cy)

    x = read_attr(blob, "x", 0)
    y = read_attr(blob, "y", 0)
    w = read_attr(blob, "w", 0)
    h = read_attr(blob, "h", 0)
    return int(x + w / 2), int(y + h / 2)


def rect_to_points(rect):
    # find_rects 返回的矩形对象通过 corners() 读取四个角点
    corners = read_attr(rect, "corners", None)
    if corners is None or len(corners) < 4:
        return None

    points = []
    for i in range(4):
        points.append(int(corners[i][0]))
        points.append(int(corners[i][1]))

    return points


def rect_area(points):
    # 用四边形面积筛选最大矩形，避免误检到较小噪声矩形
    area = 0
    for i in range(4):
        j = (i + 1) % 4
        x1 = points[i * 2]
        y1 = points[i * 2 + 1]
        x2 = points[j * 2]
        y2 = points[j * 2 + 1]
        area += x1 * y2 - x2 * y1

    if area < 0:
        area = -area
    return area


def find_max_rect(rects):
    # 从 find_rects 结果中选出面积最大的矩形
    best_points = None
    best_area = 0

    for rect in rects:
        points = rect_to_points(rect)
        if points is None:
            continue

        area = rect_area(points)
        if area > best_area:
            best_area = area
            best_points = points

    return best_points


def find_black_rect(img):
    # 参考 Maix 示例：复制一张二值图用于矩形检测，原图继续用于激光识别和显示
    try:
        binary_img = img.binary(BLACK_RECT_THRESH, copy=True)
        rects = binary_img.find_rects(RECT_ROI, RECT_THRESHOLD)
    except Exception:
        return None

    if not rects:
        return None

    return find_max_rect(rects)


def rect_center(rect):
    # 由四个角点平均得到矩形中心
    return (
        int((rect[0] + rect[2] + rect[4] + rect[6]) / 4),
        int((rect[1] + rect[3] + rect[5] + rect[7]) / 4),
    )


def draw_debug(img, rect, red_center):
    # 调试绘制，不影响坐标打印
    try:
        if rect is not None:
            img.draw_line(rect[0], rect[1], rect[2], rect[3], color=(0, 0, 255), thickness=2)
            img.draw_line(rect[2], rect[3], rect[4], rect[5], color=(0, 0, 255), thickness=2)
            img.draw_line(rect[4], rect[5], rect[6], rect[7], color=(0, 0, 255), thickness=2)
            img.draw_line(rect[6], rect[7], rect[0], rect[1], color=(0, 0, 255), thickness=2)
        if red_center is not None:
            x, y = red_center
            img.draw_cross(x, y, color=(255, 0, 0), size=8, thickness=2)
    except Exception:
        # 不同 MaixPy 版本绘图接口略有差异，绘制失败时只保留打印
        pass


def to_byte(value):
    # 串口示例协议每个数据只能占 1 字节
    return max(0, min(255, int(value)))


def build_frame(numbers):
    # 协议：帧头 + 10 字节坐标 + 帧尾
    if len(numbers) != 10:
        raise ValueError("必须发送 10 个坐标字节")

    return bytes([FRAME_HEAD]) + bytes([to_byte(n) for n in numbers]) + bytes([FRAME_TAIL])


def send_tracking_to_uart(red_center, rect):
    # 数据格式：激光 x,y + 矩形四个顶点 x,y；无额外状态字节
    if red_center is None or rect is None:
        return

    numbers = [
        red_center[0], red_center[1],
        rect[0], rect[1],
        rect[2], rect[3],
        rect[4], rect[5],
        rect[6], rect[7],
    ]

    frame = build_frame(numbers)
    serial.write(frame)


def print_result(rect, rect_center, red_center):
    # 输出格式：四个角点 | 矩形中心 | 红色激光中心，共六组坐标
    if rect is None:
        p1 = p2 = p3 = p4 = (-1, -1)
        rect_center = (-1, -1)
    else:
        p1 = (rect[0], rect[1])
        p2 = (rect[2], rect[3])
        p3 = (rect[4], rect[5])
        p4 = (rect[6], rect[7])

    if red_center is None:
        red_center = (-1, -1)

    print(
        "%d %d | %d %d | %d %d | %d %d | %d %d | %d %d"
        % (
            p1[0], p1[1],
            p2[0], p2[1],
            p3[0], p3[1],
            p4[0], p4[1],
            rect_center[0], rect_center[1],
            red_center[0], red_center[1],
        )
    )


print("p1_x p1_y | p2_x p2_y | p3_x p3_y | p4_x p4_y | rect_cx rect_cy | red_cx red_cy")
last_print_ms = time.ticks_ms() - PRINT_PERIOD_MS
last_rect = None

while not app.need_exit():
    img = capture_image()

    # 黑框识别：参考 Maix 示例，使用 binary + find_rects，不改动原图上的激光识别
    rect = find_black_rect(img)
    if rect is not None:
        # 串口协议保持 10 个坐标字节；矩形偶尔漏检时继续使用最近一次有效矩形
        last_rect = rect[:]

    uart_rect = last_rect
    center = rect_center(uart_rect) if uart_rect is not None else None

    # 红色激光识别，参考 notebook 中的 img.find_blobs(red, ...)
    red_blobs = img.find_blobs(
        RED_THRESH,
        merge=True,
        pixels_threshold=PIXELS_THRESHOLD,
        margin=MERGE_MARGIN,
    )
    red_center = None
    if red_blobs:
        red_blob = red_blobs[find_max(red_blobs)]
        red_center = blob_center(red_blob)

    now_ms = time.ticks_ms()
    if now_ms - last_print_ms >= PRINT_PERIOD_MS:
        print_result(uart_rect, center, red_center)
        last_print_ms = now_ms

    send_tracking_to_uart(red_center, uart_rect)
    draw_debug(img, uart_rect, red_center)
    show_image(img)
