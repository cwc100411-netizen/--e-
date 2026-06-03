from maix import app, camera, display, err, pinmap, sys, uart


# 摄像头尺寸参考 notebook 输出，画面中心为 120,120
CAM_W, CAM_H = 240, 240

# LAB 阈值，参考 notebook 中的一号机 2.0 红色激光参数
RED_THRESH = [(0, 100, 14, 51, -8, 17)]

# 红色激光点通常很小，阈值保持 notebook 的设置
PIXELS_THRESHOLD = 2
MERGE_MARGIN = 3

# 是否显示调试画面
SHOW_IMAGE = True

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


def line_rect(line):
    # find_line 返回的 rect 为四个角点：x1,y1,x2,y2,x3,y3,x4,y4
    rect = read_attr(line, "rect", None)
    if rect is None or len(rect) < 8:
        return None
    return [int(v) for v in rect[:8]]


def line_center(line, rect):
    # 优先使用 find_line 返回的中心点，没有则由四个角点平均得到
    cx = read_attr(line, "cx", None)
    cy = read_attr(line, "cy", None)
    if cx is not None and cy is not None:
        return int(cx), int(cy)

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
    # 发送.py 中的协议：帧头 + 8 字节数据 + 帧尾
    if len(numbers) != 8:
        raise ValueError("必须发送 8 个数字")

    return bytes([FRAME_HEAD]) + bytes([to_byte(n) for n in numbers]) + bytes([FRAME_TAIL])


def send_laser_to_uart(red_center):
    # 数据格式：red_x, red_y, detected, 0, 0, 0, 0, 0
    if red_center is None:
        numbers = [0, 0, 0, 0, 0, 0, 0, 0]
    else:
        numbers = [red_center[0], red_center[1], 1, 0, 0, 0, 0, 0]

    frame = build_frame(numbers)
    serial.write(frame)
    print("uart:", numbers)


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

while not app.need_exit():
    img = capture_image()

    # 黑框识别，参考 notebook 中的 img.find_line()
    line = img.find_lines()
    rect = line_rect(line)
    rect_center = line_center(line, rect) if rect is not None else None

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

    print_result(rect, rect_center, red_center)
    send_laser_to_uart(red_center)
    draw_debug(img, rect, red_center)
    show_image(img)
