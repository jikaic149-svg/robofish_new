"""Chinese OpenCV HUD and clickable toolbar for the desktop vision console."""

from dataclasses import dataclass
from functools import lru_cache
import os
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


Color = Tuple[int, int, int]  # OpenCV BGR


class UiPalette:
    PANEL: Color = (28, 32, 38)
    PANEL_ALT: Color = (39, 45, 53)
    BUTTON: Color = (48, 55, 64)
    BUTTON_ACTIVE: Color = (68, 77, 88)
    BORDER: Color = (82, 91, 104)
    TEXT: Color = (242, 244, 247)
    MUTED: Color = (174, 183, 194)
    PRIMARY: Color = (226, 157, 58)
    SUCCESS: Color = (119, 201, 59)
    WARNING: Color = (55, 173, 242)
    DANGER: Color = (92, 92, 235)


FONT_CANDIDATES = (
    os.environ.get("ROBOFISH_UI_FONT", ""),
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/simhei.ttf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
)
BOLD_FONT_CANDIDATES = (
    os.environ.get("ROBOFISH_UI_FONT_BOLD", ""),
    "C:/Windows/Fonts/msyhbd.ttc",
    "C:/Windows/Fonts/simhei.ttf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
)


def _find_font(candidates: Tuple[str, ...]) -> Optional[str]:
    for candidate in candidates:
        if candidate and os.path.exists(candidate):
            return candidate
    return None


REGULAR_FONT_PATH = _find_font(FONT_CANDIDATES)
BOLD_FONT_PATH = _find_font(BOLD_FONT_CANDIDATES) or REGULAR_FONT_PATH
_TEXT_RENDER_ENABLED = True


def set_text_render_enabled(enabled: bool) -> None:
    """Enable/disable PIL text rasterization for a preview iteration.

    Drawing shapes can remain at camera rate while the UI text follows the
    display rate.  This is deliberately a rendering switch only; it has no
    effect on tracking, control, or the data sent over the network.
    """
    global _TEXT_RENDER_ENABLED
    _TEXT_RENDER_ENABLED = bool(enabled)


@lru_cache(maxsize=32)
def _font(size_px: int, bold: bool = False):
    path = BOLD_FONT_PATH if bold else REGULAR_FONT_PATH
    if path:
        return ImageFont.truetype(path, max(8, int(size_px)))
    return ImageFont.load_default()


def measure_unicode_text(text: str, size_px: int, bold: bool = False) -> Tuple[int, int]:
    font = _font(size_px, bold)
    left, top, right, bottom = font.getbbox(str(text))
    return max(1, right - left), max(1, bottom - top)


def draw_unicode_text(
    image: np.ndarray,
    text: str,
    position: Tuple[int, int],
    size_px: int,
    color: Color = UiPalette.TEXT,
    bold: bool = False,
    stroke_width: int = 0,
    stroke_color: Color = (0, 0, 0),
) -> None:
    """Draw Chinese text by converting only its small bounding ROI through PIL."""
    if not _TEXT_RENDER_ENABLED:
        return
    text = str(text)
    if not text:
        return
    font = _font(size_px, bold)
    left, top, right, bottom = font.getbbox(text, stroke_width=stroke_width)
    text_width = max(1, right - left + stroke_width * 2)
    text_height = max(1, bottom - top + stroke_width * 2)
    x, y = int(position[0]), int(position[1])
    x1 = max(0, x)
    y1 = max(0, y)
    x2 = min(image.shape[1], x + text_width)
    y2 = min(image.shape[0], y + text_height)
    if x2 <= x1 or y2 <= y1:
        return

    roi = image[y1:y2, x1:x2]
    pil_image = Image.fromarray(cv2.cvtColor(roi, cv2.COLOR_BGR2RGB))
    painter = ImageDraw.Draw(pil_image)
    draw_x = x - x1 - left + stroke_width
    draw_y = y - y1 - top + stroke_width
    painter.text(
        (draw_x, draw_y),
        text,
        font=font,
        fill=(color[2], color[1], color[0]),
        stroke_width=stroke_width,
        stroke_fill=(stroke_color[2], stroke_color[1], stroke_color[0]),
    )
    image[y1:y2, x1:x2] = cv2.cvtColor(np.asarray(pil_image), cv2.COLOR_RGB2BGR)


def draw_rounded_rectangle(
    image: np.ndarray,
    rectangle: Tuple[int, int, int, int],
    color: Color,
    radius: int = 8,
    thickness: int = -1,
) -> None:
    x1, y1, x2, y2 = [int(value) for value in rectangle]
    radius = int(max(0, min(radius, (x2 - x1) // 2, (y2 - y1) // 2)))
    if radius <= 0:
        cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
        return
    if thickness != -1:
        cv2.line(image, (x1 + radius, y1), (x2 - radius, y1), color, thickness)
        cv2.line(image, (x1 + radius, y2), (x2 - radius, y2), color, thickness)
        cv2.line(image, (x1, y1 + radius), (x1, y2 - radius), color, thickness)
        cv2.line(image, (x2, y1 + radius), (x2, y2 - radius), color, thickness)
        cv2.ellipse(image, (x1 + radius, y1 + radius), (radius, radius), 180, 0, 90, color, thickness)
        cv2.ellipse(image, (x2 - radius, y1 + radius), (radius, radius), 270, 0, 90, color, thickness)
        cv2.ellipse(image, (x2 - radius, y2 - radius), (radius, radius), 0, 0, 90, color, thickness)
        cv2.ellipse(image, (x1 + radius, y2 - radius), (radius, radius), 90, 0, 90, color, thickness)
        return
    cv2.rectangle(image, (x1 + radius, y1), (x2 - radius, y2), color, -1)
    cv2.rectangle(image, (x1, y1 + radius), (x2, y2 - radius), color, -1)
    cv2.circle(image, (x1 + radius, y1 + radius), radius, color, -1)
    cv2.circle(image, (x2 - radius, y1 + radius), radius, color, -1)
    cv2.circle(image, (x2 - radius, y2 - radius), radius, color, -1)
    cv2.circle(image, (x1 + radius, y2 - radius), radius, color, -1)


def draw_translucent_panel(
    image: np.ndarray,
    rectangle: Tuple[int, int, int, int],
    color: Color = UiPalette.PANEL,
    alpha: float = 0.84,
    radius: int = 8,
) -> None:
    overlay = image.copy()
    draw_rounded_rectangle(overlay, rectangle, color, radius=radius, thickness=-1)
    cv2.addWeighted(overlay, alpha, image, 1.0 - alpha, 0.0, image)


def fit_preview_window_size(
    frame_width: int,
    frame_height: int,
    screen_width: int,
    screen_height: int,
) -> Tuple[int, int]:
    """Return a large aspect-preserving preview size that still fits the screen."""
    frame_width = max(1, int(frame_width))
    frame_height = max(1, int(frame_height))
    usable_width = max(frame_width, int(screen_width * 0.90))
    usable_height = max(frame_height, int(screen_height * 0.90))
    scale = min(usable_width / frame_width, usable_height / frame_height)
    return int(round(frame_width * scale)), int(round(frame_height * scale))


def enable_windows_dpi_awareness() -> bool:
    """让 Windows 鼠标坐标与 OpenCV 窗口矩形使用同一像素尺度。"""
    if os.name != "nt":
        return False
    try:
        import ctypes

        user32 = ctypes.windll.user32
        try:
            # PER_MONITOR_AWARE_V2; 必须在创建 HighGUI 窗口前调用。
            if user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4)):
                return True
        except Exception:
            pass
        try:
            return bool(user32.SetProcessDPIAware())
        except Exception:
            return False
    except Exception:
        return False


def map_screen_point_to_frame(
    screen_x: int,
    screen_y: int,
    image_rectangle: Tuple[int, int, int, int],
    frame_width: int,
    frame_height: int,
) -> Optional[Tuple[int, int]]:
    """Map an absolute Windows cursor position into the displayed camera frame."""
    image_x, image_y, image_width, image_height = image_rectangle
    if image_width <= 0 or image_height <= 0:
        return None
    relative_x = int(screen_x) - int(image_x)
    relative_y = int(screen_y) - int(image_y)
    if not (0 <= relative_x < image_width and 0 <= relative_y < image_height):
        return None
    frame_x = int(relative_x * frame_width / image_width)
    frame_y = int(relative_y * frame_height / image_height)
    return (
        int(np.clip(frame_x, 0, frame_width - 1)),
        int(np.clip(frame_y, 0, frame_height - 1)),
    )


def map_client_point_to_frame(
    client_x: int,
    client_y: int,
    client_width: int,
    client_height: int,
    frame_width: int,
    frame_height: int,
) -> Optional[Tuple[int, int]]:
    """把 Win32 客户区坐标映射到画布，同时处理等比例黑边。"""
    client_width = int(client_width)
    client_height = int(client_height)
    frame_width = max(1, int(frame_width))
    frame_height = max(1, int(frame_height))
    if client_width <= 0 or client_height <= 0:
        return None

    scale = min(client_width / frame_width, client_height / frame_height)
    image_width = frame_width * scale
    image_height = frame_height * scale
    image_x = (client_width - image_width) * 0.5
    image_y = (client_height - image_height) * 0.5
    relative_x = float(client_x) - image_x
    relative_y = float(client_y) - image_y
    if not (0.0 <= relative_x < image_width and 0.0 <= relative_y < image_height):
        return None

    return (
        int(np.clip(relative_x * frame_width / image_width, 0, frame_width - 1)),
        int(np.clip(relative_y * frame_height / image_height, 0, frame_height - 1)),
    )


class WindowsPointerPoller:
    """Win32 pointer fallback for HighGUI builds that drop mouse callbacks."""

    VK_LBUTTON = 0x01
    VK_RBUTTON = 0x02

    def __init__(self, window_name: str, frame_width: int, frame_height: int):
        self.window_name = str(window_name)
        self.frame_width = max(1, int(frame_width))
        self.frame_height = max(1, int(frame_height))
        self.available = False
        self._button_down = {self.VK_LBUTTON: False, self.VK_RBUTTON: False}
        self._started_inside = {self.VK_LBUTTON: False, self.VK_RBUTTON: False}
        self._last_point: Optional[Tuple[int, int]] = None
        self.last_screen_point: Optional[Tuple[int, int]] = None
        self.last_image_rectangle: Optional[Tuple[int, int, int, int]] = None
        self.last_mapping_source = "NONE"
        self._user32 = None
        self._point_type = None
        self._rect_type = None
        self._ctypes = None
        if os.name != "nt":
            return
        try:
            import ctypes
            from ctypes import wintypes

            self._user32 = ctypes.windll.user32
            self._point_type = wintypes.POINT
            self._rect_type = wintypes.RECT
            self._ctypes = ctypes
            self._user32.FindWindowW.argtypes = [wintypes.LPCWSTR, wintypes.LPCWSTR]
            self._user32.FindWindowW.restype = wintypes.HWND
            self._user32.WindowFromPoint.argtypes = [wintypes.POINT]
            self._user32.WindowFromPoint.restype = wintypes.HWND
            self._user32.GetAncestor.argtypes = [wintypes.HWND, wintypes.UINT]
            self._user32.GetAncestor.restype = wintypes.HWND
            self._user32.GetClientRect.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.RECT)]
            self._user32.GetClientRect.restype = wintypes.BOOL
            self._user32.ScreenToClient.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.POINT)]
            self._user32.ScreenToClient.restype = wintypes.BOOL
            self._user32.ClientToScreen.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.POINT)]
            self._user32.ClientToScreen.restype = wintypes.BOOL
            self._user32.ShowWindow.argtypes = [wintypes.HWND, ctypes.c_int]
            self._user32.ShowWindow.restype = wintypes.BOOL
            self.available = True
        except Exception:
            self.available = False

    def maximize_window(self) -> bool:
        """最大化 OpenCV 顶层窗口，保留标题栏和正确画面比例。"""
        if not self.available or self._user32 is None:
            return False
        try:
            top_window = self._user32.FindWindowW(None, self.window_name)
            if not top_window:
                return False
            self._user32.ShowWindow(top_window, 3)  # SW_MAXIMIZE
            return True
        except Exception:
            return False

    def _cursor_point(self) -> Optional[Tuple[int, int]]:
        if (
            not self.available or self._user32 is None
            or self._point_type is None or self._ctypes is None
        ):
            return None
        point = self._point_type()
        if not self._user32.GetCursorPos(self._ctypes.byref(point)):
            return None
        self.last_screen_point = (int(point.x), int(point.y))

        # OpenCV Win32UI 的顶层窗口内还有一个真正绘制图像的子窗口。
        # 直接使用鼠标下方的原生 HWND，可避免 OpenCV 返回的矩形
        # 与 Windows DPI 虚拟坐标不在同一尺度。
        try:
            top_window = self._user32.FindWindowW(None, self.window_name)
            canvas_window = self._user32.WindowFromPoint(point)
            if top_window and canvas_window:
                root_window = self._user32.GetAncestor(canvas_window, 2)  # GA_ROOT
                if root_window == top_window:
                    local_point = self._point_type(point.x, point.y)
                    client_rect = self._rect_type()
                    if (
                        self._user32.ScreenToClient(
                            canvas_window, self._ctypes.byref(local_point)
                        )
                        and self._user32.GetClientRect(
                            canvas_window, self._ctypes.byref(client_rect)
                        )
                    ):
                        client_width = int(client_rect.right - client_rect.left)
                        client_height = int(client_rect.bottom - client_rect.top)
                        mapped = map_client_point_to_frame(
                            local_point.x, local_point.y,
                            client_width, client_height,
                            self.frame_width, self.frame_height,
                        )
                        if mapped is not None:
                            client_origin = self._point_type(0, 0)
                            self._user32.ClientToScreen(
                                canvas_window, self._ctypes.byref(client_origin)
                            )
                            scale = min(
                                client_width / self.frame_width,
                                client_height / self.frame_height,
                            )
                            image_width = int(round(self.frame_width * scale))
                            image_height = int(round(self.frame_height * scale))
                            image_x = int(round(
                                client_origin.x + (client_width - image_width) * 0.5
                            ))
                            image_y = int(round(
                                client_origin.y + (client_height - image_height) * 0.5
                            ))
                            self.last_image_rectangle = (
                                image_x, image_y, image_width, image_height
                            )
                            self.last_mapping_source = "WIN32_CANVAS"
                            return mapped
        except Exception:
            pass

        # 非 Win32UI 或原生窗口未就绪时才使用 OpenCV 备用路径。
        try:
            image_rectangle = cv2.getWindowImageRect(self.window_name)
        except (cv2.error, AttributeError):
            return None
        self.last_image_rectangle = tuple(int(value) for value in image_rectangle)
        self.last_mapping_source = "OPENCV_FALLBACK"
        return map_screen_point_to_frame(
            point.x, point.y, image_rectangle,
            self.frame_width, self.frame_height,
        )

    def poll(self) -> List[Tuple[int, int, int, int]]:
        """Return pending `(event, x, y, flags)` transitions for this frame."""
        if not self.available or self._user32 is None:
            return []
        current_point = self._cursor_point()
        events: List[Tuple[int, int, int, int]] = []
        button_specs = (
            (self.VK_LBUTTON, cv2.EVENT_LBUTTONDOWN, cv2.EVENT_LBUTTONUP,
             cv2.EVENT_FLAG_LBUTTON),
            (self.VK_RBUTTON, cv2.EVENT_RBUTTONDOWN, cv2.EVENT_RBUTTONUP,
             cv2.EVENT_FLAG_RBUTTON),
        )
        for virtual_key, down_event, up_event, flag in button_specs:
            is_down = bool(self._user32.GetAsyncKeyState(virtual_key) & 0x8000)
            was_down = self._button_down[virtual_key]
            if is_down and not was_down:
                self._started_inside[virtual_key] = current_point is not None
                if current_point is not None:
                    events.append((down_event, current_point[0], current_point[1], flag))
            elif not is_down and was_down:
                if self._started_inside[virtual_key]:
                    release_point = current_point or self._last_point
                    if release_point is not None:
                        events.append((up_event, release_point[0], release_point[1], 0))
                self._started_inside[virtual_key] = False
            self._button_down[virtual_key] = is_down

        if self._button_down[self.VK_LBUTTON] and self._started_inside[self.VK_LBUTTON]:
            if current_point is not None and current_point != self._last_point:
                events.append((
                    cv2.EVENT_MOUSEMOVE,
                    current_point[0], current_point[1],
                    cv2.EVENT_FLAG_LBUTTON,
                ))
        if current_point is not None:
            self._last_point = current_point
        return events


@dataclass(frozen=True)
class ToolbarButton:
    action: str
    label: str
    role: str = "normal"


class VisionToolbar:
    # Keep the canvas toolbar usable when the browser scales the vision image.
    # Keeping it in the same canvas means it also works when OpenCV's native
    # trackbar window is unavailable (for example on the tablet preview).
    HEIGHT = 154
    SLIDER_HEIGHT = 32
    GAP = 8
    BUTTON_FONT_SIZE = 20
    ROWS: Tuple[Tuple[ToolbarButton, ...], ...] = (
        (
            ToolbarButton("START", "启动", "primary"),
            ToolbarButton("STOP", "停止", "danger"),
            ToolbarButton("CLEAR_PATH", "清除轨迹"),
            ToolbarButton("MARKER_ROI", "鱼尾标定", "primary"),
            ToolbarButton("HEAD_DIRECTION", "鱼头方向", "primary"),
            ToolbarButton("POOL_CALIB", "场地标定", "primary"),
        ),
        (
            ToolbarButton("TURN_CALIB", "转圈测量", "primary"),
            ToolbarButton("RECORD", "录像"),
            ToolbarButton("SNAPSHOT", "截图"),
            ToolbarButton("CLAHE", "画面增强"),
            ToolbarButton("EXP_DOWN", "曝光－"),
            ToolbarButton("EXP_UP", "曝光＋"),
            ToolbarButton("EXIT", "退出", "danger"),
        ),
    )

    def __init__(self):
        self._rectangles: Dict[str, Tuple[int, int, int, int]] = {}
        self._slider_rect: Tuple[int, int, int, int] = (0, 0, 0, 0)
        self._slider_dragging = False
        self.binary_threshold = 135

    @property
    def height(self) -> int:
        return self.HEIGHT

    def layout(self, frame_width: int, frame_height: int) -> Dict[str, Tuple[int, int, int, int]]:
        panel_top = max(0, frame_height - self.HEIGHT)
        button_top = panel_top + self.SLIDER_HEIGHT
        button_height = max(24, self.HEIGHT - self.SLIDER_HEIGHT)
        row_height = max(18, (button_height - self.GAP * 3) // 2)
        rectangles: Dict[str, Tuple[int, int, int, int]] = {}
        for row_index, buttons in enumerate(self.ROWS):
            y1 = button_top + self.GAP + row_index * (row_height + self.GAP)
            y2 = min(frame_height - self.GAP, y1 + row_height)
            available = frame_width - self.GAP * (len(buttons) + 1)
            button_width = max(1, available // len(buttons))
            for column, button in enumerate(buttons):
                x1 = self.GAP + column * (button_width + self.GAP)
                x2 = frame_width - self.GAP if column == len(buttons) - 1 else x1 + button_width
                rectangles[button.action] = (x1, y1, x2, y2)
        self._rectangles = rectangles
        slider_y1 = panel_top + self.GAP
        slider_y2 = min(frame_height - self.GAP, slider_y1 + self.SLIDER_HEIGHT - self.GAP)
        label_width = min(180, max(140, frame_width // 7))
        self._slider_rect = (
            self.GAP + label_width,
            slider_y1,
            max(self.GAP + label_width + 1, frame_width - self.GAP),
            slider_y2,
        )
        return dict(rectangles)

    def hit_test(
        self, x: int, y: int, frame_width: int, frame_height: int
    ) -> Optional[str]:
        rectangles = self.layout(frame_width, frame_height)
        for action, (x1, y1, x2, y2) in rectangles.items():
            if x1 <= x < x2 and y1 <= y < y2:
                return action
        return None

    def contains(self, x: int, y: int, frame_height: int) -> bool:
        return 0 <= x and y >= frame_height - self.HEIGHT

    @property
    def slider_dragging(self) -> bool:
        return self._slider_dragging

    def slider_hit_test(self, x: int, y: int, frame_width: int, frame_height: int) -> bool:
        self.layout(frame_width, frame_height)
        x1, y1, x2, y2 = self._slider_rect
        return x1 <= x < x2 and y1 <= y < y2

    def begin_slider_drag(self, x: int, frame_width: int, frame_height: int) -> int:
        self.layout(frame_width, frame_height)
        self._slider_dragging = True
        return self.update_slider(x, frame_width, frame_height)

    def update_slider(self, x: int, frame_width: int, frame_height: int) -> int:
        self.layout(frame_width, frame_height)
        x1, _, x2, _ = self._slider_rect
        usable = max(1, x2 - x1)
        self.binary_threshold = int(np.clip(round((x - x1) * 255.0 / usable), 0, 255))
        return self.binary_threshold

    def end_slider_drag(self) -> None:
        self._slider_dragging = False

    @staticmethod
    def _role_color(role: str) -> Color:
        if role == "danger":
            return UiPalette.DANGER
        if role == "primary":
            return UiPalette.PRIMARY
        return UiPalette.BORDER

    def draw(self, image: np.ndarray, active: Optional[Dict[str, bool]] = None) -> None:
        active = active or {}
        height, width = image.shape[:2]
        panel_top = max(0, height - self.HEIGHT)
        overlay = image.copy()
        cv2.rectangle(overlay, (0, panel_top), (width, height), UiPalette.PANEL, -1)
        cv2.addWeighted(overlay, 0.92, image, 0.08, 0.0, image)
        rectangles = self.layout(width, height)
        # Live binary-threshold slider.  The value is a V-channel floor used
        # by the white-body binary mask; the main loop applies it immediately.
        sx1, sy1, sx2, sy2 = self._slider_rect
        draw_unicode_text(image, f"二值化 V: {self.binary_threshold}", (self.GAP + 4, sy1 + 3), 16, UiPalette.TEXT, bold=True)
        track_y = (sy1 + sy2) // 2
        track_left = sx1 + 2
        track_right = max(track_left + 1, sx2 - 2)
        cv2.line(image, (track_left, track_y), (track_right, track_y), UiPalette.BORDER, 5, cv2.LINE_AA)
        knob_x = int(round(track_left + (track_right - track_left) * self.binary_threshold / 255.0))
        cv2.line(image, (track_left, track_y), (knob_x, track_y), UiPalette.PRIMARY, 5, cv2.LINE_AA)
        cv2.circle(image, (knob_x, track_y), 7, UiPalette.TEXT, -1, cv2.LINE_AA)
        cv2.circle(image, (knob_x, track_y), 7, UiPalette.PRIMARY, 2, cv2.LINE_AA)
        by_action = {button.action: button for row in self.ROWS for button in row}

        for action, rectangle in rectangles.items():
            x1, y1, x2, y2 = rectangle
            button = by_action[action]
            accent = self._role_color(button.role)
            fill = UiPalette.BUTTON_ACTIVE if active.get(action, False) else UiPalette.BUTTON
            if active.get(action, False):
                fill = tuple(int(0.65 * fill[i] + 0.35 * accent[i]) for i in range(3))
            draw_rounded_rectangle(image, rectangle, fill, radius=5, thickness=-1)
            draw_rounded_rectangle(image, rectangle, accent, radius=5, thickness=1)
            if active.get(action, False):
                cv2.rectangle(image, (x1 + 5, y2 - 3), (x2 - 5, y2 - 2), accent, -1)

            font_size = min(self.BUTTON_FONT_SIZE, max(12, (x2 - x1 - 12) // max(1, len(button.label))))
            text_width, text_height = measure_unicode_text(button.label, font_size, bold=True)
            text_x = x1 + max(2, (x2 - x1 - text_width) // 2)
            text_y = y1 + max(2, (y2 - y1 - text_height) // 2 - 1)
            draw_unicode_text(
                image, button.label, (text_x, text_y), font_size,
                UiPalette.TEXT, bold=True
            )


class VisionHud:
    SOURCE_NAMES = {
        "RIGID_BODY": "白色刚性鱼身",
        "MARKER": "鱼尾定位",
        "PREDICTED": "短时预测",
        "ESTIMATED_FALLBACK": "识别框降级",
        "INVALID": "位置无效",
    }
    STATUS_NAMES = {
        "READY": "待命",
        "STOPPED": "已停止",
        "EMERGENCY STOP": "紧急停止",
        "PROP TRACKING": "轨迹跟随",
        "HYBRID TRACKING": "混合轨迹跟随",
        "HYBRID BRAKING": "混合柔性制动",
        "TARGET LOST": "目标丢失",
        "ARRIVED": "已到终点",
        "UNCALIBRATED": "场地未标定",
        "CALIBRATING": "场地标定中",
        "SELECT MARKER ROI": "色标框选中",
        "MARKER READY": "色标已就绪",
        "MARKER ROI RETRY": "色标需重选",
        "SELECT HEAD DIRECTION": "鱼头方向标定",
        "HEAD DIRECTION READY": "初始方向已标定",
        "TURN CALIBRATING": "转圈测量中",
        "TURN CALIBRATED": "转圈测量完成",
        "PATH CLEARED": "轨迹已清除",
        "DEBUG OVERLAY": "调试叠加层",
        "CAMERA PAUSED": "相机等待中",
    }
    GATE_NAMES = {
        "MARKER_NOT_CALIBRATED": "鱼尾未标定",
        "YOLO_NOT_CONFIRMED": "未确认机器鱼",
        "NO_MARKER_CONTOUR": "未找到橙色鱼尾",
        "AREA_GATE": "面积异常",
        "CIRCULARITY_GATE": "轮廓形状异常",
        "EXTENT_GATE": "轮廓填充异常",
        "ASPECT_GATE": "长宽比异常",
        "COLOR_GATE": "颜色质量不足",
        "BOUNDARY_CROP_GATE": "色标被边界裁切",
        "POSITION_JUMP_GATE": "位置突变",
        "QUALITY_GATE": "综合质量不足",
        "INVALID_FRAME": "画面无效",
        "BODY_TOO_SMALL_GATE": "白色候选面积过小",
        "NO_RIGID_BODY_CONTOUR": "未找到白色刚性鱼身",
        "BODY_MARKER_DISTANCE_GATE": "白色候选不在鱼身位置",
        "BODY_POSITION_JUMP_GATE": "白色鱼身位置突变",
        "BODY_GLARE_SHAPE_GATE": "白色候选像圆形反光",
        "BODY_GLARE_EXPOSURE_GATE": "白色候选过曝",
    }

    @staticmethod
    def _source_color(source: str) -> Color:
        if source in ("RIGID_BODY", "MARKER"):
            return UiPalette.SUCCESS
        if source == "PREDICTED":
            return UiPalette.WARNING
        if source == "ESTIMATED_FALLBACK":
            return UiPalette.WARNING
        return UiPalette.DANGER

    def draw(
        self,
        image: np.ndarray,
        *,
        source: str,
        status: str,
        quality: float,
        pwm_ready: bool,
        position: Optional[Tuple[float, float]],
        speed: Optional[float],
        direction_deg: Optional[float],
        gate_reasons,
        loop_fps: float,
        camera_fps: float,
        yolo_fps: float,
        exposure: int,
        tablet_tx_hz: float,
        tablet_rx_hz: float,
        mcu_hz: float,
        tracking_error: Optional[Dict[str, float]] = None,
        prompt: Optional[str] = None,
    ) -> None:
        height, width = image.shape[:2]
        left_bottom = 108 if source == "INVALID" and gate_reasons else 88
        left_rect = (8, 8, min(314, width - 8), left_bottom)
        right_rect = (max(322, width - 206), 8, width - 8, 67)
        draw_translucent_panel(image, left_rect, alpha=0.35, radius=6)
        draw_translucent_panel(image, right_rect, alpha=0.35, radius=6)

        source_name = self.SOURCE_NAMES.get(source, source)
        status_name = self.STATUS_NAMES.get(status, status)
        source_color = self._source_color(source)
        cv2.circle(image, (20, 21), 4, source_color, -1, cv2.LINE_AA)
        draw_unicode_text(image, f"{status_name} · {source_name}", (30, 12), 14, UiPalette.TEXT, bold=True)

        if position is None:
            position_text = "位置：无效"
        else:
            position_text = f"位置：X {position[0]:.2f} m　Y {position[1]:.2f} m"
        draw_unicode_text(image, position_text, (16, 35), 11, UiPalette.TEXT)

        if speed is None or direction_deg is None:
            motion_text = "速度：无效　运动方向：无效"
        else:
            motion_text = f"速度：{speed:.2f} m/s　运动方向：{direction_deg:.1f}°"
        draw_unicode_text(image, motion_text, (16, 52), 11, UiPalette.TEXT)

        quality_text = f"跟踪质量：{quality * 100:.0f}%　精标定：{'可用' if pwm_ready else '不可用'}"
        draw_unicode_text(
            image, quality_text, (16, 69), 11,
            UiPalette.SUCCESS if pwm_ready else UiPalette.WARNING
        )

        draw_unicode_text(image, "运行状态", (right_rect[0] + 9, 12), 12, UiPalette.TEXT, bold=True)
        draw_unicode_text(
            image,
            f"循环 {loop_fps:.1f}　相机 {camera_fps:.1f}　识别 {yolo_fps:.1f}",
            (right_rect[0] + 9, 31), 10, UiPalette.MUTED
        )
        draw_unicode_text(
            image,
            f"平板 {tablet_tx_hz:.1f}/{tablet_rx_hz:.1f}　设备 {mcu_hz:.1f}　曝光 {exposure}",
            (right_rect[0] + 9, 47), 10, UiPalette.MUTED
        )

        if source == "INVALID" and gate_reasons:
            gate_name = self.GATE_NAMES.get(gate_reasons[0], gate_reasons[0])
            draw_unicode_text(image, f"原因：{gate_name}", (16, 88), 11, UiPalette.DANGER, bold=True)

        if tracking_error is not None:
            panel_bottom = height - VisionToolbar.HEIGHT - 7
            panel_top = panel_bottom - 46
            panel_right = min(width - 8, 455)
            draw_translucent_panel(image, (8, panel_top, panel_right, panel_bottom), alpha=0.88)
            heading_error = tracking_error.get('heading_error_deg')
            if heading_error is None:
                detail_text = (
                    f"轨迹偏差：{tracking_error.get('cross_m', 0.0):+.3f} m　"
                    f"剩余：{tracking_error.get('dist_m', 0.0):.3f} m　"
                    f"段号：{int(tracking_error.get('seg_index', 0))}"
                )
            else:
                detail_text = (
                    f"鱼体左右：{tracking_error.get('x_error_m', 0.0):+.3f} m　"
                    f"航向误差：{float(heading_error):+.1f}°　"
                    f"剩余：{tracking_error.get('dist_m', 0.0):.3f} m"
                )
            draw_unicode_text(
                image,
                detail_text,
                (18, panel_top + 13), 14, UiPalette.TEXT, bold=True
            )

        if prompt:
            prompt_width = min(width - 32, 520)
            prompt_left = (width - prompt_width) // 2
            prompt_rect = (prompt_left, 112, prompt_left + prompt_width, 147)
            draw_translucent_panel(image, prompt_rect, UiPalette.PANEL_ALT, 0.42, radius=6)
            text_width, _ = measure_unicode_text(prompt, 14, bold=True)
            text_x = prompt_left + max(10, (prompt_width - text_width) // 2)
            draw_unicode_text(image, prompt, (text_x, 122), 14, UiPalette.WARNING, bold=True)
