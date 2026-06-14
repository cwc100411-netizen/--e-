cv2 = None
np = None
_SCRIPT_DIR = None


def _ensure_cv():
    global cv2, np
    if cv2 is not None and np is not None:
        return True
    try:
        import cv2 as cv2_module
        import numpy as np_module
    except Exception:
        cv2 = None
        np = None
        return False
    cv2 = cv2_module
    np = np_module
    return True


def _module_dir():
    global _SCRIPT_DIR
    if _SCRIPT_DIR is None:
        try:
            import os
            _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
        except Exception:
            _SCRIPT_DIR = "."
    return _SCRIPT_DIR


def _normalize_digit(binary, size=(32, 48)):
    ys, xs = np.where(binary > 0)
    if len(xs) == 0:
        return None
    x0, x1 = int(xs.min()), int(xs.max()) + 1
    y0, y1 = int(ys.min()), int(ys.max()) + 1
    crop = binary[y0:y1, x0:x1]
    target_w, target_h = size
    crop_h, crop_w = crop.shape[:2]
    if crop_w <= 0 or crop_h <= 0:
        return None
    scale = min(float(target_w - 6) / crop_w, float(target_h - 6) / crop_h)
    new_w = max(1, int(round(crop_w * scale)))
    new_h = max(1, int(round(crop_h * scale)))
    resized = cv2.resize(crop, (new_w, new_h), interpolation=cv2.INTER_AREA)
    out = np.zeros((target_h, target_w), dtype=np.uint8)
    ox = (target_w - new_w) // 2
    oy = (target_h - new_h) // 2
    out[oy:oy + new_h, ox:ox + new_w] = resized
    _, out = cv2.threshold(out, 127, 255, cv2.THRESH_BINARY)
    return out


def _template_for_digit(value, font, scale, thickness):
    canvas = np.zeros((80, 60), dtype=np.uint8)
    text = str(value)
    (text_w, text_h), _ = cv2.getTextSize(text, font, scale, thickness)
    x = (canvas.shape[1] - text_w) // 2
    y = (canvas.shape[0] + text_h) // 2
    cv2.putText(canvas, text, (x, y), font, scale, 255, thickness, cv2.LINE_AA)
    _, canvas = cv2.threshold(canvas, 40, 255, cv2.THRESH_BINARY)
    return _normalize_digit(canvas)


def _iou_score(a, b):
    a_fg = a > 0
    b_fg = b > 0
    union = np.logical_or(a_fg, b_fg).sum()
    if union <= 0:
        return 0.0
    return float(np.logical_and(a_fg, b_fg).sum()) / float(union)


def _projection_peaks(binary):
    fg = binary > 0
    row_counts = fg.sum(axis=1)
    if row_counts.size == 0:
        return []
    threshold = max(3, int(row_counts.max() * 0.45))
    peaks = []
    start = None
    for idx, count in enumerate(row_counts):
        if count >= threshold and start is None:
            start = idx
        if start is not None and (count < threshold or idx == len(row_counts) - 1):
            end = idx - 1 if count < threshold else idx
            if end - start >= 1:
                peaks.append((start, end))
            start = None
    return peaks


def _projection_segments(values, threshold):
    segments = []
    start = None
    for idx, value in enumerate(values):
        if value >= threshold and start is None:
            start = idx
        if start is not None and (value < threshold or idx == len(values) - 1):
            end = idx - 1 if value < threshold else idx
            if end >= start:
                segments.append((start, end))
            start = None
    return segments


def _classify_three_four_shape(normalized, score3, score4):
    fg = normalized > 0
    total = float(fg.sum())
    if total <= 0:
        return None, 0.0

    height = normalized.shape[0]
    center = float(fg[height // 3: 2 * height // 3, :].sum())
    center_ratio = center / total
    peaks = _projection_peaks(normalized)
    longest_peak = max((end - start + 1 for start, end in peaks), default=0)

    # A printed 3 is usually split into top/middle/bottom row-projection bands.
    # A printed 4 has a strong middle band connected to the right vertical stroke.
    if len(peaks) >= 3 and longest_peak <= height * 0.34:
        return 3, max(score3, score4 * 0.98)
    if longest_peak >= height * 0.36 and center_ratio >= 0.45:
        return 4, max(score4, score3 * 0.98)
    return None, 0.0


def _classify_three_five_shape(normalized, score3, score5):
    fg = normalized > 0
    total = float(fg.sum())
    if total <= 0:
        return None, 0.0

    height, width = normalized.shape[:2]
    row_counts = fg.sum(axis=1).astype(float)
    col_counts = fg.sum(axis=0).astype(float)

    top_band = float(row_counts[: max(1, height // 3)].sum()) / total
    mid_band = float(row_counts[height // 3: (2 * height) // 3].sum()) / total
    bottom_band = float(row_counts[(2 * height) // 3:].sum()) / total

    left_band = float(col_counts[: max(1, width // 3)].sum()) / total
    right_band = float(col_counts[(2 * width) // 3:].sum()) / total
    score5_lead = float(score5) - float(score3)

    row_threshold = max(3.0, float(row_counts.max()) * 0.42)
    peak_segments = _projection_segments(row_counts, row_threshold)
    longest_peak = max((end - start + 1 for start, end in peak_segments), default=0)

    if score5_lead >= 0.025:
        return 5, max(score5, score3 * 0.98)

    # Printed 5 usually keeps a stronger top band and left-side mass, while 3
    # shifts more mass to the right with two rounded lobes.
    if top_band >= 0.34 and left_band >= 0.24 and right_band <= 0.40:
        boosted = max(score5, score3 * 0.98)
        if longest_peak >= height * 0.18:
            boosted = max(boosted, score5 + 0.02)
        return 5, boosted
    if right_band >= 0.39 and mid_band <= 0.31 and bottom_band >= 0.28 and score3 >= score5 - 0.015:
        return 3, max(score3, score5 * 0.98)
    return None, 0.0


class DigitDetector:
    def __init__(self, min_score=0.45):
        self.min_score = min_score
        self.last_error = ""
        self._templates = None

    def available(self):
        return _ensure_cv()

    def _build_templates(self):
        if self._templates is not None:
            return self._templates
        if not self.available():
            self._templates = []
            return self._templates
        fonts = [
            cv2.FONT_HERSHEY_SIMPLEX,
            cv2.FONT_HERSHEY_DUPLEX,
            cv2.FONT_HERSHEY_COMPLEX,
            cv2.FONT_HERSHEY_TRIPLEX,
        ]
        templates = []
        for value in (1, 2, 3, 4, 5):
            for font in fonts:
                for scale in (1.5, 1.8, 2.0, 2.2):
                    for thickness in (2, 3, 4):
                        templ = _template_for_digit(value, font, scale, thickness)
                        if templ is not None:
                            templates.append((value, templ))
        # Extra thin / small-scale templates for digit 1, which is naturally narrow.
        for font in fonts:
            for scale in (1.0, 1.2, 1.3, 1.5):
                for thickness in (1, 2):
                    templ = _template_for_digit(1, font, scale, thickness)
                    if templ is not None:
                        templates.append((1, templ))
        templates.extend(self._load_reference_templates())
        self._templates = templates
        return templates

    def _load_reference_templates(self):
        templates = []
        if not self.available():
            return templates
        try:
            import os
            for value in (1, 2, 3, 4, 5):
                path = os.path.join(_module_dir(), "reference_digit_%d.jpg" % value)
                if not os.path.exists(path):
                    continue
                img = cv2.imread(path)
                if img is None:
                    continue
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY_INV)
                normalized = _normalize_digit(binary)
                if normalized is not None:
                    templates.append((value, normalized))
        except Exception:
            return templates
        return templates

    def _binary_dark(self, gray):
        # 根据图像尺寸自适应调整块大小，小分辨率用小块，避免淹没远处的数字
        h, w = gray.shape[:2]
        block_size = max(11, min(31, int(min(w, h) * 0.06) | 1))  # 奇数，范围 11~31
        adaptive = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, block_size, 9
        )
        _, fixed = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        return cv2.bitwise_or(adaptive, fixed)

    def _mask_dark(self, img_bgr):
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        mask = self._binary_dark(gray)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        return mask

    def _suppress_laser_spot(self, img_bgr, laser_hint=None):
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        red1 = cv2.inRange(hsv, (0, 80, 170), (18, 255, 255))
        red2 = cv2.inRange(hsv, (145, 60, 170), (179, 255, 255))
        laser_mask = cv2.bitwise_or(red1, red2)
        if laser_hint is not None:
            lx, ly = [int(v) for v in laser_hint]
            height, width = img_bgr.shape[:2]
            if 0 <= lx < width and 0 <= ly < height:
                hint_mask = np.zeros((height, width), dtype=np.uint8)
                cv2.circle(hint_mask, (lx, ly), 7, 255, -1)
                laser_mask = cv2.bitwise_or(laser_mask, hint_mask)
        if cv2.countNonZero(laser_mask) <= 0:
            return img_bgr
        laser_mask = cv2.dilate(laser_mask, np.ones((5, 5), dtype=np.uint8), iterations=1)
        return cv2.inpaint(img_bgr, laser_mask, 5, cv2.INPAINT_TELEA)

    def _classify_with_scores(self, binary):
        normalized = _normalize_digit(binary)
        if normalized is None:
            return None, 0.0, {}
        best_value = None
        best_score = 0.0
        scores = {}
        for value, templ in self._build_templates():
            score = _iou_score(normalized, templ)
            if score > scores.get(value, 0.0):
                scores[value] = score
            if score > best_score:
                best_score = score
                best_value = value
        score3 = scores.get(3, 0.0)
        score4 = scores.get(4, 0.0)
        score5 = scores.get(5, 0.0)
        if max(score3, score5) >= self.min_score and abs(score3 - score5) <= 0.14:
            shape_value, shape_score = _classify_three_five_shape(normalized, score3, score5)
            if shape_value is not None:
                scores[shape_value] = max(float(scores.get(shape_value, 0.0)), float(shape_score))
                return shape_value, shape_score, scores
        if max(score3, score4) >= self.min_score and abs(score3 - score4) <= 0.12:
            shape_value, shape_score = _classify_three_four_shape(normalized, score3, score4)
            if shape_value is not None:
                scores[shape_value] = max(float(scores.get(shape_value, 0.0)), float(shape_score))
                return shape_value, shape_score, scores
        return best_value, best_score, scores

    def _classify(self, binary):
        value, score, _ = self._classify_with_scores(binary)
        return value, score

    def _clip_box(self, x, y, w, h, limit_w, limit_h):
        x = max(0, min(limit_w - 1, int(x)))
        y = max(0, min(limit_h - 1, int(y)))
        x2 = max(x + 1, min(limit_w, int(x + max(1, w))))
        y2 = max(y + 1, min(limit_h, int(y + max(1, h))))
        return x, y, x2 - x, y2 - y

    def _first_peak(self, values, threshold):
        for idx, value in enumerate(values):
            if value >= threshold:
                return idx
        return int(np.argmax(values)) if len(values) else 0

    def _last_peak(self, values, threshold):
        for idx in range(len(values) - 1, -1, -1):
            if values[idx] >= threshold:
                return idx
        return int(np.argmax(values)) if len(values) else 0

    def _extract_digit_component(self, inner_mask):
        if inner_mask is None or inner_mask.size <= 0:
            return None, None
        work = inner_mask.copy()
        work[0, :] = 0
        work[-1, :] = 0
        work[:, 0] = 0
        work[:, -1] = 0
        if cv2.countNonZero(work) <= 0:
            return None, None

        stats_count, labels, stats, _ = cv2.connectedComponentsWithStats(work, connectivity=8)
        best_label = 0
        best_area = 0
        width = work.shape[1]
        height = work.shape[0]
        min_area = max(8, int(width * height * 0.003))
        for label in range(1, stats_count):
            x, y, w, h, area = stats[label]
            if area < min_area or w < 3 or h < 6:
                continue
            if x <= 0 or y <= 0 or x + w >= width or y + h >= height:
                continue
            if area > best_area:
                best_label = label
                best_area = area

        if best_label <= 0:
            ys, xs = np.where(work > 0)
            if len(xs) == 0:
                return None, None
            x0 = int(xs.min())
            y0 = int(ys.min())
            x1 = int(xs.max()) + 1
            y1 = int(ys.max()) + 1
            return work, (x0, y0, x1 - x0, y1 - y0)

        component = np.zeros_like(work)
        component[labels == best_label] = 255
        x, y, w, h, _ = stats[best_label]
        return component, (int(x), int(y), int(w), int(h))

    def _inset_outer_box(self, outer_box, limit_w, limit_h):
        x, y, w, h = outer_box
        inset_x = max(2, int(round(w * 0.12)))
        inset_y = max(2, int(round(h * 0.08)))
        return self._clip_box(x + inset_x, y + inset_y, w - inset_x * 2, h - inset_y * 2, limit_w, limit_h)

    def _rebalance_outer_box(self, outer_box, digit_box, frame_w, frame_h):
        outer_x, outer_y, outer_w, outer_h = outer_box
        digit_x, digit_y, digit_w, digit_h = digit_box

        left_margin = digit_x - outer_x
        right_margin = (outer_x + outer_w) - (digit_x + digit_w)
        top_margin = digit_y - outer_y
        bottom_margin = (outer_y + outer_h) - (digit_y + digit_h)

        # If one side is clearly over-padded, trim only the dominant side.
        horizontal_threshold = max(3, int(round(digit_w * 0.18)))
        vertical_threshold = max(3, int(round(digit_h * 0.12)))

        if left_margin - right_margin > horizontal_threshold:
            trim = max(0, int(round((left_margin - right_margin) * 0.55)))
            outer_x += trim
            outer_w -= trim
        elif right_margin - left_margin > horizontal_threshold:
            trim = max(0, int(round((right_margin - left_margin) * 0.55)))
            outer_w -= trim

        if top_margin - bottom_margin > vertical_threshold:
            trim = max(0, int(round((top_margin - bottom_margin) * 0.40)))
            outer_y += trim
            outer_h -= trim
        elif bottom_margin - top_margin > vertical_threshold:
            trim = max(0, int(round((bottom_margin - top_margin) * 0.40)))
            outer_h -= trim

        min_w = max(digit_w + 6, int(round(digit_w * 1.18)))
        min_h = max(digit_h + 6, int(round(digit_h * 1.16)))
        outer_w = max(min_w, outer_w)
        outer_h = max(min_h, outer_h)
        return self._clip_box(outer_x, outer_y, outer_w, outer_h, frame_w, frame_h)

    def _result_from_outer_box(self, gray, mask, outer_box):
        frame_h, frame_w = mask.shape[:2]
        outer_x, outer_y, outer_w, outer_h = self._clip_box(outer_box[0], outer_box[1], outer_box[2], outer_box[3], frame_w, frame_h)
        inner_x, inner_y, inner_w, inner_h = self._inset_outer_box((outer_x, outer_y, outer_w, outer_h), frame_w, frame_h)
        inner_mask = mask[inner_y:inner_y + inner_h, inner_x:inner_x + inner_w]
        component_mask, component_box = self._extract_digit_component(inner_mask)
        if component_mask is None or component_box is None:
            return None
        value, score, scores = self._classify_with_scores(component_mask)
        if value is None or score < self.min_score:
            return None
        comp_x, comp_y, comp_w, comp_h = component_box
        digit_x = inner_x + comp_x
        digit_y = inner_y + comp_y
        digit_w = comp_w
        digit_h = comp_h
        outer_x, outer_y, outer_w, outer_h = self._rebalance_outer_box(
            (outer_x, outer_y, outer_w, outer_h),
            (digit_x, digit_y, digit_w, digit_h),
            frame_w,
            frame_h,
        )
        return {
            "value": int(value),
            "score": float(score),
            "x": int(digit_x),
            "y": int(digit_y),
            "w": int(digit_w),
            "h": int(digit_h),
            "cx": int(digit_x + digit_w / 2),
            "cy": int(digit_y + digit_h / 2),
            "outer_x": int(outer_x),
            "outer_y": int(outer_y),
            "outer_w": int(outer_w),
            "outer_h": int(outer_h),
            "outer_cx": int(outer_x + outer_w / 2),
            "outer_cy": int(outer_y + outer_h / 2),
            "score_map": {int(key): float(val) for key, val in scores.items()},
        }

    def _detect_boxed_digits(self, gray, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        height, width = mask.shape[:2]
        # 根据分辨率自适应最小宽高，确保远处的小外框也能被检测
        min_w = max(10, int(width * 0.03))
        min_h = max(14, int(height * 0.05))
        max_area = float(width * height) * 0.65
        results = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1:
                continue
            if w < min_w or h < min_h:
                continue
            area = cv2.contourArea(contour)
            box_area = float(w * h)
            if area < 50 or box_area > max_area:
                continue
            aspect = float(h) / max(1.0, float(w))
            if aspect < 0.8 or aspect > 2.8:
                continue
            rectangularity = float(area) / box_area
            if rectangularity < 0.72:
                continue
            ink_fill = float(cv2.countNonZero(mask[y:y + h, x:x + w])) / box_area
            if ink_fill < 0.06 or ink_fill > 0.82:
                continue
            result = self._result_from_outer_box(gray, mask, (x, y, w, h))
            if result is not None:
                results.append(result)
        return results

    def _estimate_outer_box(self, gray, digit_box):
        frame_h, frame_w = gray.shape[:2]
        x, y, w, h = digit_box
        x, y, w, h = self._clip_box(x, y, w, h, frame_w, frame_h)
        dark = self._binary_dark(gray) > 0
        pad_x = max(8, int(round(w * 0.9)))
        pad_y = max(6, int(round(h * 0.35)))
        sx0 = max(0, x - pad_x)
        sy0 = max(0, y - pad_y)
        sx1 = min(frame_w, x + w + pad_x)
        sy1 = min(frame_h, y + h + pad_y)
        window = dark[sy0:sy1, sx0:sx1]
        if window.size <= 0:
            return self._heuristic_outer_box(digit_box, frame_w, frame_h)

        col_counts = window.sum(axis=0)
        row_counts = window.sum(axis=1)
        left_end = max(1, x - sx0)
        right_start = min(window.shape[1] - 1, x + w - sx0)
        top_end = max(1, y - sy0)
        bottom_start = min(window.shape[0] - 1, y + h - sy0)
        vertical_threshold = max(4, int(window.shape[0] * 0.45))
        horizontal_threshold = max(4, int(window.shape[1] * 0.45))

        left = sx0 + self._last_peak(col_counts[:left_end], vertical_threshold)
        right = sx0 + right_start + self._first_peak(col_counts[right_start:], vertical_threshold)
        top = sy0 + self._last_peak(row_counts[:top_end], horizontal_threshold)
        bottom = sy0 + bottom_start + self._first_peak(row_counts[bottom_start:], horizontal_threshold)

        if left >= x or right <= x + w or top >= y or bottom <= y + h:
            return self._heuristic_outer_box(digit_box, frame_w, frame_h)
        if right - left < max(10, w + 4) or bottom - top < max(14, h + 4):
            return self._heuristic_outer_box(digit_box, frame_w, frame_h)
        return self._clip_box(left, top, right - left + 1, bottom - top + 1, frame_w, frame_h)

    def _heuristic_outer_box(self, digit_box, frame_w, frame_h):
        x, y, w, h = digit_box
        pad_x = max(4, int(round(w * 0.35)))
        pad_y = max(4, int(round(h * 0.12)))
        return self._clip_box(x - pad_x, y - pad_y, w + pad_x * 2, h + pad_y * 2, frame_w, frame_h)

    def _detect_digit_contours(self, gray, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        height, width = mask.shape[:2]
        # 根据分辨率自适应最小面积阈值，高分辨率下远处数字面积也相应变大
        scale_factor = (width * height) / (320.0 * 240.0)
        min_area = max(20, int(40 / scale_factor))
        max_area = int(2200 * scale_factor)
        min_w = max(4, int(6 / scale_factor ** 0.5))
        min_h = max(8, int(14 / scale_factor ** 0.5))
        max_w = int(55 * scale_factor ** 0.5)
        max_h = int(85 * scale_factor ** 0.5)
        results = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if area < min_area or area > max_area:
                continue
            if w < min_w or h < min_h or w > max_w or h > max_h:
                continue
            aspect = float(h) / max(1.0, float(w))
            if aspect < 0.8 or aspect > 7.0:
                continue
            if w <= 6 and h <= 16:
                continue
            if x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1:
                continue
            digit_roi = mask[y:y + h, x:x + w]
            fill = float(cv2.countNonZero(digit_roi)) / float(w * h)
            if fill < 0.07 or fill > 0.78:
                continue
            if w <= 4 and fill > 0.78 and aspect > 4.5:
                continue
            value, score, scores = self._classify_with_scores(digit_roi)
            if value is None or score < self.min_score:
                continue
            outer_box = self._estimate_outer_box(gray, (x, y, w, h))
            result = {
                "value": int(value),
                "score": float(score),
                "x": int(x),
                "y": int(y),
                "w": int(w),
                "h": int(h),
                "cx": int(x + w / 2),
                "cy": int(y + h / 2),
                "outer_x": int(outer_box[0]),
                "outer_y": int(outer_box[1]),
                "outer_w": int(outer_box[2]),
                "outer_h": int(outer_box[3]),
                "outer_cx": int(outer_box[0] + outer_box[2] / 2),
                "outer_cy": int(outer_box[1] + outer_box[3] / 2),
                "score_map": {int(key): float(val) for key, val in scores.items()},
            }
            boxed = self._result_from_outer_box(gray, mask, outer_box)
            if boxed is not None and int(boxed["value"]) == int(value):
                boxed["score"] = max(float(boxed["score"]), float(score))
                result = boxed
            results.append(result)
        return results

    def detect_cv(self, img_bgr, roi=None, laser_hint=None):
        if not self.available():
            self.last_error = "opencv unavailable"
            return []

        img_bgr = self._suppress_laser_spot(img_bgr, laser_hint=laser_hint)
        x0 = 0
        y0 = 0
        frame_h, frame_w = img_bgr.shape[:2]
        if roi:
            x, y, width, height = [int(v) for v in roi]
            x0 = max(0, min(frame_w - 1, x))
            y0 = max(0, min(frame_h - 1, y))
            x1 = max(x0 + 1, min(frame_w, x0 + max(1, width)))
            y1 = max(y0 + 1, min(frame_h, y0 + max(1, height)))
            img_bgr = img_bgr[y0:y1, x0:x1]

        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        mask = self._mask_dark(img_bgr)
        results = self._detect_boxed_digits(gray, mask)
        results.extend(self._detect_digit_contours(gray, mask))
        results = self._dedupe_results(results)
        for item in results:
            item["x"] = int(item["x"] + x0)
            item["y"] = int(item["y"] + y0)
            item["cx"] = int(item["cx"] + x0)
            item["cy"] = int(item["cy"] + y0)
            item["outer_x"] = int(item["outer_x"] + x0)
            item["outer_y"] = int(item["outer_y"] + y0)
            item["outer_cx"] = int(item["outer_cx"] + x0)
            item["outer_cy"] = int(item["outer_cy"] + y0)
        results.sort(key=lambda item: (item["value"], item["y"], item["x"]))
        self.last_error = "" if results else "digits not found"
        return results

    def _dedupe_results(self, results):
        # Keep the strongest candidate for overlapping fragments, then prefer one
        # candidate per value for the track-digits workflow.
        kept = []
        for item in sorted(results, key=lambda value: value["score"], reverse=True):
            duplicate = False
            for previous in kept:
                item_cx = int(item.get("outer_cx", item["cx"]))
                item_cy = int(item.get("outer_cy", item["cy"]))
                prev_cx = int(previous.get("outer_cx", previous["cx"]))
                prev_cy = int(previous.get("outer_cy", previous["cy"]))
                dx = item_cx - prev_cx
                dy = item_cy - prev_cy
                if dx * dx + dy * dy < 18 * 18:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(item)

        by_value = {}
        for item in kept:
            value = int(item["value"])
            previous = by_value.get(value)
            if previous is None or float(item["score"]) > float(previous["score"]):
                by_value[value] = item

        if len(by_value) >= 5:
            return [by_value[value] for value in sorted(by_value)]
        return kept

    def detect_maix(self, img, roi=None, laser_hint=None):
        if not self.available():
            self.last_error = "opencv unavailable"
            return []
        from maix import image

        try:
            img_bgr = image.image2cv(img, copy=True)
        except TypeError:
            img_bgr = image.image2cv(img, copy=False)
        return self.detect_cv(img_bgr, roi=roi, laser_hint=laser_hint)

    def find_unassigned_candidates(self, img_bgr, exclude_boxes, roi=None, min_score=0.30):
        """在排除已检测区域后，从剩余黑色连通域中寻找数字候选。

        用于补齐漏检的数字：当 1~5 中某些数字未被检测到时，
        在未分配区域中搜索最大的暗色连通域并尝试分类。

        Args:
            img_bgr: BGR 图像 (numpy array)
            exclude_boxes: 要排除的矩形 [(x, y, w, h), ...]
            roi: 搜索范围 (x, y, w, h)，None = 全图
            min_score: 宽松的分数阈值

        Returns:
            候选列表 [{value, score, x, y, w, h, cx, cy, area}, ...]
            按面积降序排列
        """
        if not self.available():
            return []

        x0, y0 = 0, 0
        frame_h, frame_w = img_bgr.shape[:2]
        if roi:
            rx, ry, rw, rh = [int(v) for v in roi]
            x0 = max(0, min(frame_w - 1, rx))
            y0 = max(0, min(frame_h - 1, ry))
            x1 = max(x0 + 1, min(frame_w, x0 + max(1, rw)))
            y1 = max(y0 + 1, min(frame_h, y0 + max(1, rh)))
            work_img = img_bgr[y0:y1, x0:x1]
        else:
            work_img = img_bgr

        mask = self._mask_dark(work_img)
        work_h, work_w = mask.shape[:2]

        # 构建排除掩膜：将已检测区域涂黑
        for bx, by, bw, bh in exclude_boxes:
            # 转换到工作图坐标
            ex0 = max(0, int(bx - x0))
            ey0 = max(0, int(by - y0))
            ex1 = min(work_w, int(bx + bw - x0))
            ey1 = min(work_h, int(by + bh - y0))
            if ex0 < ex1 and ey0 < ey1:
                mask[ey0:ey1, ex0:ex1] = 0

        # 找所有连通域
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        min_area = max(20, int(work_w * work_h * 0.003))

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = int(cv2.contourArea(contour))
            if area < min_area or w < 4 or h < 8:
                continue
            if x <= 0 or y <= 0 or x + w >= work_w - 1 or y + h >= work_h - 1:
                continue

            roi_binary = mask[y:y + h, x:x + w]
            value, score, scores = self._classify_with_scores(roi_binary)
            if value is None or score < min_score:
                continue

            candidates.append({
                "value": int(value),
                "score": float(score),
                "x": int(x + x0),
                "y": int(y + y0),
                "w": int(w),
                "h": int(h),
                "cx": int(x + x0 + w / 2),
                "cy": int(y + y0 + h / 2),
                "area": area,
            })

        # 按面积降序（优先用大块补漏）
        candidates.sort(key=lambda c: c["area"], reverse=True)
        return candidates
