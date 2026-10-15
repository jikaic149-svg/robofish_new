import csv
import json
import math
import os
import queue
import socket
import sys
import threading
import time
import urllib.parse
import urllib.request
import cv2
import numpy as np
from flask import Flask, Response

os.environ.setdefault("YOLO_CONFIG_DIR", os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".ultralytics"
))
os.environ.setdefault("MPLCONFIGDIR", os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".matplotlib"
))

from collections import deque
from fish_detector import FishDetector
from color_pose_tracker import WhitePointTracker
from fixed_reference_tracker import FixedReferenceTracker, ReferenceSource
from path_guidance import (
    PathGuidance,
    VelocityEstimator,
    compensate_camera_latency,
)
from turn_calibration import (
    TurnCalibrationError,
    TurnCalibrationSession,
    load_turn_calibrations,
    save_turn_calibration,
)
from vision_toolbar import (
    UiPalette,
    VisionHud,
    VisionToolbar,
    draw_unicode_text as _draw_unicode_text_impl,
    set_text_render_enabled,
)

# PIL-backed Chinese text is considerably more expensive than the OpenCV
# primitives used for tracking graphics.  The preview and MJPEG outputs are
# capped at 30 Hz, so suppress text rasterization on intermediate 60 Hz loop
# iterations.  Recording explicitly keeps text on every frame.
_UI_DRAW_STATE = {'allow_text': True}


def draw_unicode_text(*args, **kwargs):
    if _UI_DRAW_STATE['allow_text']:
        return _draw_unicode_text_impl(*args, **kwargs)
    return None

RUNTIME_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runtime")
if RUNTIME_DIR not in sys.path:
    sys.path.insert(0, RUNTIME_DIR)
from yaw_filter import BodyYawKalman, BodyYawKalmanConfig
from vision_protocol import pack_vision_udp_frame

# ============= 设置工作目录（带路径安全回退） =============
PREFERRED_DIR = os.path.dirname(os.path.abspath(__file__))
WORK_DIR = PREFERRED_DIR
if not os.path.exists(WORK_DIR):
    os.makedirs(WORK_DIR)
os.chdir(WORK_DIR)

# 核心提速秘籍：禁用 OpenCV MSMF 后端的硬件转换功能
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

# ==============================================================================
# [配置区] 相机采集帧率与分辨率
# ==============================================================================
TARGET_FPS = 30
TARGET_WIDTH = 1280
TARGET_HEIGHT = 720
CAMERA_INDEX = 0                 # 外接 Global Shutter Camera（索引 1 是笔记本内置相机）
EXPOSURE_MIN = -13                # Global Shutter Camera 的 DirectShow 有效下限
EXPOSURE_MAX = -4                 # 该驱动在 1280×720/30 FPS 下的有效范围
EXPOSURE_DEFAULT = -6
EXPOSURE_DIGITAL_STEP = 2.0 ** (1.0 / 3.0)
CAMERA_LATENCY_S = 0.30             # 三次屏幕秒表测试均值约 0.31 s
CAMERA_LATENCY_MAX_PREDICTION_M = 0.12

# 是否默认开启 CLAHE（直方图均衡化，水面有强烈反光或光照不均时开启）
ENABLE_CLAHE_DEFAULT = False
# Diagnostic view: the extra semantic YOLO/colour endpoint provenance can be
# enabled independently.  The accepted reference marker and fading short
# trajectory remain visible in the normal view, matching the initial interface.
# The initial desktop view always showed the accepted fish reference marker
# and its short trajectory.  Keep that view as the default; the debug toggle
# still controls the additional endpoint provenance diagnostics.
SHOW_COLOR_AUX_OVERLAY_DEFAULT = True
# Show the actual filtered body-yaw arrow by default; the path controller's
# target-yaw arrow is drawn separately when a path is active.
SHOW_FILTERED_YAW_OVERLAY_DEFAULT = True
# Show the fish bbox in the normal desktop view, as in the initial interface.
# It is drawn in blue so it cannot be confused with the orange-tail contour.
SHOW_YOLO_BBOX_OVERLAY_DEFAULT = True

# ==============================================================================
# [配置区] 场地真实物理尺寸与标靶 ID 映射
# ==============================================================================
# World-frame dimensions used by the homography below.  The camera image is
# calibrated in the order TL -> TR -> BR -> BL, so the top image edge is the
# long edge (x = 0 .. 4.2 m) and the left image edge is the short edge
# (y = 0 .. 2.1 m).
PHYSICAL_WIDTH = 4.2     # 画面上边：水池长边（米）
PHYSICAL_HEIGHT = 2.1    # 画面左边：水池短边（米）

MARKER_TL = 1
MARKER_TR = 2
MARKER_BR = 4
MARKER_BL = 3

# ==============================================================================
# [配置区] YOLO 仿生鱼检测
# ==============================================================================
POSE_MODEL_CANDIDATE = os.path.join(
    WORK_DIR,
    "training", "fish_pose_v1", "runs",
    "yolo11n-pose-960-strict-sigma-v2", "train", "weights", "best.pt",
)
YOLO_MODEL_PATH = (
    POSE_MODEL_CANDIDATE
    if os.path.isfile(POSE_MODEL_CANDIDATE)
    else os.path.join(WORK_DIR, "last.pt")
)
# Keep low-confidence candidates so the detector can still return the
# highest-confidence fish in dim/occluded frames.  On the latest replay the
# fish score fell below 0.18 in 122/1272 frames, while the highest score was
# still >= 0.10 in every frame; filtering at 0.18 therefore looked like
# intermittent fish/head/tail loss rather than a true model miss.
YOLO_CONF_THRESHOLD = 0.10
YOLO_IMG_SIZE = 960      # 与两关键点 Pose 训练尺寸一致，减少端点抖动
YOLO_DEVICE = 0             # 0 = GPU，无显卡改为 "cpu"
YOLO_DETECT_INTERVAL_S = 0.20  # YOLO 仅做 5 Hz 身份确认，橙色色块逐帧跟踪
MARKER_PROFILE_PATH = os.path.join(WORK_DIR, "marker_profile.local.json")
TURN_CALIBRATION_PATH = os.path.join(
    WORK_DIR, "turn_calibration.local.json"
)
POOL_CALIBRATION_PATH = os.path.join(
    WORK_DIR, "pool_calibration.local.json"
)

# ==============================================================================
# [配置区] 通信参数（平板 TCP & RoboFish ESP32 HTTP）
# ==============================================================================
TABLET_TCP_HOST = "0.0.0.0"
TABLET_TCP_PORT = 9998

# RoboFish 机器鱼 AP 网络默认配置
ROBOFISH_IP = "192.168.4.1"
ROBOFISH_PORT = 80
ROBOFISH_UDP_PORT = 9000
VISION_UDP_PERIOD_S = 0.05       # 20 Hz fixed-period latest-value frame
VISION_UDP_STALE_S = 0.25
UDP_SPEED_REFERENCE_MPS = 0.36   # 100% command corresponds to this target
BODY_YAW_RESULT_TIMEOUT_S = 0.60  # 5 Hz YOLO cadence + one missed result
YAW_FILTER_PERIOD_S = 0.05       # do not collapse KF covariance at camera rate
MIN_YAW_BASELINE_M = 0.04        # shorter head-tail baselines are too noisy
# The path controller may use a filtered body yaw only while the filter is
# still fresh.  Once the visual endpoint disappears, hold the last heading for
# a short grace period before falling back to the marker-derived course.
# This prevents a single white-head miss from changing the control law on the
# very next camera frame, while avoiding a stale yaw lock during an occlusion.
BODY_YAW_CONTROL_MAX_AGE_S = 0.35
BODY_YAW_CONTROL_HOLD_S = 0.25
# Motion/command prior used as a soft constraint on the visual yaw.  It is
# intentionally weaker than a valid white-head measurement so control cannot
# blindly follow a stale command.
MOTION_PRIOR_TIMEOUT_S = 0.35
MOTION_PRIOR_MAX_BLEND = 0.35
MOTION_PRIOR_MAX_INNOVATION_DEG = 95.0
MOTION_PRIOR_CLAMP_DEG = 45.0

# --- 坐标与速度计算平滑系数 ---
# Keep only light smoothing on the marker-derived position.  A low alpha
# makes the control point visibly lag behind quick fish motion.
POS_SMOOTHING_ALPHA = 0.65

# --- MJPEG 视频推流配置 ---
MJPEG_PORT = 8090
MJPEG_STREAM_WIDTH = 960      # 缩放至 960x540，降低带宽
MJPEG_STREAM_HEIGHT = 540
MJPEG_JPEG_QUALITY = 50       # 压缩质量设为 50，提升传输顺畅度
MJPEG_MAX_FPS = 30            # 限制推流上限 30 帧

# --- 本机预览窗口刷新帧率上限 ---
DISPLAY_MAX_FPS = 30
# Keep the measured fish trail in the normal view.  Points remain fully
# bright for a short window, then fade out and are removed after this age so a
# long-running session cannot leave a permanent yellow scribble on the image.
TRAJECTORY_FADE_START_S = 2.5
TRAJECTORY_MAX_AGE_S = 8.0
TRAJECTORY_MAX_POINTS = 240

# 辅助函数：绘制高对比度加粗黑边 OSD 文字（即使水面强光反光也能清晰可见）
def draw_outlined_text(img, text, pos, font, scale, color, thickness=2):
    x, y = pos
    cv2.putText(img, text, (x, y), font, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def _wrap_angle_rad(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _blend_angles_rad(measured, prior, prior_weight):
    """Circularly blend a visual angle with a short-horizon motion prior."""
    weight = float(np.clip(prior_weight, 0.0, 1.0))
    x = (1.0 - weight) * math.cos(float(measured)) + weight * math.cos(float(prior))
    y = (1.0 - weight) * math.sin(float(measured)) + weight * math.sin(float(prior))
    if abs(x) + abs(y) <= 1e-9:
        return _wrap_angle_rad(measured)
    return _wrap_angle_rad(math.atan2(y, x))


def draw_calibration_overlay(img, points):
    """Draw the physical dimensions represented by the locked homography.

    ``points`` are image pixels in TL, TR, BR, BL order.  Keeping this overlay
    tied to the same four points used to build ``H`` makes an axis swap visible
    immediately on the live image instead of only in the numeric coordinates.
    """
    if points is None or len(points) != 4:
        return
    pts = np.asarray(points, dtype=np.float32).reshape((4, 2))
    if not np.isfinite(pts).all():
        return

    edges = (
        (0, 1, (0, 255, 0), f"TOP long edge = {PHYSICAL_WIDTH:.2f} m"),
        (0, 3, (255, 180, 0), f"LEFT short edge = {PHYSICAL_HEIGHT:.2f} m"),
    )
    for start, end, colour, label in edges:
        p0 = tuple(int(round(value)) for value in pts[start])
        p1 = tuple(int(round(value)) for value in pts[end])
        cv2.line(img, p0, p1, colour, 2, cv2.LINE_AA)
        midpoint = ((p0[0] + p1[0]) // 2, (p0[1] + p1[1]) // 2)
        # Put labels just inside the calibrated quadrilateral.  The small
        # fixed offset is intentional: it remains readable under perspective
        # without changing any coordinates used by the homography.
        if start == 0 and end == 1:
            text_pos = (midpoint[0] - 92, midpoint[1] + 24)
        else:
            text_pos = (midpoint[0] + 12, midpoint[1])
        draw_outlined_text(
            img, label, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.52, colour, 1
        )


def build_calibration_overlay(frame_shape, points):
    """Rasterize static calibration graphics once for the live UI.

    The pool is stationary, so its four corner labels and dimension lines do
    not need to invoke text/line drawing at camera rate.  A value of ``1`` is
    used as the untouched sentinel so even black outline pixels are retained
    by the mask.
    """
    if points is None or len(points) == 0:
        return None, None
    height, width = int(frame_shape[0]), int(frame_shape[1])
    overlay = np.ones((height, width, 3), dtype=np.uint8)
    for point_index, point in enumerate(points):
        px, py = int(round(point[0])), int(round(point[1]))
        cv2.circle(overlay, (px, py), 7, (0, 165, 255), -1)
        draw_outlined_text(
            overlay, str(point_index + 1), (px + 10, py - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2
        )
    if len(points) == 4:
        draw_calibration_overlay(overlay, points)
    mask = np.any(overlay != 1, axis=2).astype(np.uint8) * 255
    return overlay, mask


def apply_calibration_overlay(image, overlay, mask):
    if overlay is None or mask is None:
        return
    cv2.copyTo(overlay, mask, image)


def build_calibration_homography(points, frame_width, frame_height):
    """验证 TL→TR→BR→BL 四点并生成像素到物理坐标的单应矩阵。"""
    if len(points) != 4:
        return None, "需要四个标定点"

    src = np.asarray(points, dtype=np.float32)
    if not np.isfinite(src).all():
        return None, "标定点包含无效数值"
    if not cv2.isContourConvex(src.reshape((-1, 1, 2))):
        return None, "四点顺序错误或区域不为凸四边形"

    top_mid = (src[0] + src[1]) * 0.5
    bottom_mid = (src[2] + src[3]) * 0.5
    left_mid = (src[0] + src[3]) * 0.5
    right_mid = (src[1] + src[2]) * 0.5
    if top_mid[1] >= bottom_mid[1] or left_mid[0] >= right_mid[0]:
        return None, "请严格按左上→右上→右下→左下点击"

    area = abs(float(cv2.contourArea(src.reshape((-1, 1, 2)))))
    if area < frame_width * frame_height * 0.05:
        return None, "标定区域过小"

    dst = np.float32([
        [0, 0], [PHYSICAL_WIDTH, 0],
        [PHYSICAL_WIDTH, PHYSICAL_HEIGHT], [0, PHYSICAL_HEIGHT]
    ])
    homography, _ = cv2.findHomography(src, dst, method=0)
    if homography is None or not np.isfinite(homography).all():
        return None, "无法计算有效标定矩阵"
    if abs(float(np.linalg.det(homography))) < 1e-12:
        return None, "标定矩阵退化"
    return homography, None


def save_pool_calibration(path, points, frame_width, frame_height):
    """Persist one stationary-pool calibration for reuse on later runs."""
    src = np.asarray(points, dtype=np.float32).reshape((4, 2))
    homography, error = build_calibration_homography(
        src, frame_width, frame_height
    )
    if error:
        raise ValueError(error)
    payload = {
        "version": 1,
        "frame_width": int(frame_width),
        "frame_height": int(frame_height),
        "physical_width_m": float(PHYSICAL_WIDTH),
        "physical_height_m": float(PHYSICAL_HEIGHT),
        "points_tl_tr_br_bl": src.tolist(),
        "homography": np.asarray(homography, dtype=np.float64).tolist(),
    }
    temporary_path = f"{path}.tmp"
    with open(temporary_path, "w", encoding="utf-8") as calibration_file:
        json.dump(payload, calibration_file, ensure_ascii=False, indent=2)
        calibration_file.write("\n")
    os.replace(temporary_path, path)
    return homography


def load_pool_calibration(path, frame_width, frame_height):
    """Load a saved calibration only when it matches this camera/config."""
    if not os.path.isfile(path):
        return None, None, None
    try:
        with open(path, "r", encoding="utf-8") as calibration_file:
            payload = json.load(calibration_file)
        if int(payload.get("version", 0)) != 1:
            return None, None, "unknown calibration version"
        if int(payload.get("frame_width", -1)) != int(frame_width) or \
                int(payload.get("frame_height", -1)) != int(frame_height):
            return None, None, "camera resolution changed"
        if not np.isclose(
            float(payload.get("physical_width_m", float("nan"))),
            PHYSICAL_WIDTH,
        ) or not np.isclose(
            float(payload.get("physical_height_m", float("nan"))),
            PHYSICAL_HEIGHT,
        ):
            return None, None, "physical dimensions changed"
        points = np.asarray(
            payload.get("points_tl_tr_br_bl"), dtype=np.float32
        ).reshape((4, 2))
        homography, error = build_calibration_homography(
            points, frame_width, frame_height
        )
        if error:
            return None, None, error
        return homography, points.tolist(), None
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
        return None, None, str(error)

# ==============================================================================
# 相机多线程读取类（含曝光手动调节支持）
# ==============================================================================
class CameraStream:
    def __init__(self, src=0):
        self.src = src
        self.lock = threading.Lock()
        self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW if isinstance(src, int) else cv2.CAP_ANY)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

        self.real_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.real_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.reported_fps = self.cap.get(cv2.CAP_PROP_FPS)

        # 曝光控制
        self.exposure_val = EXPOSURE_DEFAULT
        self.exposure_actual = None
        self.digital_exposure_gain = 1.0
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure_val)
        self.exposure_actual = self.cap.get(cv2.CAP_PROP_EXPOSURE)
        self.cap.set(cv2.CAP_PROP_GAIN, 100)

        self.ret, self.frame = self.cap.read()
        self.timestamp = time.time()
        self.stopped = False

        self.measured_fps = TARGET_FPS
        self._last_ts = self.timestamp

        self.thread = threading.Thread(target=self.update, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def adjust_exposure(self, delta):
        previous = self.exposure_val
        requested = int(np.clip(
            previous + int(delta), EXPOSURE_MIN, EXPOSURE_MAX
        ))
        if requested == previous:
            print(
                f"相机曝光已在边界 {previous}，"
                "不重复写入驱动"
            )
            return
        with self.lock:
            self.exposure_val = requested
            self.digital_exposure_gain = float(np.clip(
                EXPOSURE_DIGITAL_STEP ** (requested - EXPOSURE_DEFAULT),
                0.25,
                4.0,
            ))
        print(
            f"相机数字曝光补偿: {requested}，"
            f"增益 x{self.digital_exposure_gain:.2f}（硬件曝光保持启动值）"
        )

    def update(self):
        while not self.stopped:
            # DirectShow VideoCapture is not thread-safe when read() races a
            # property set.  Serialize both operations with the same lock so
            # exposure changes cannot be lost while a frame is being read.
            with self.lock:
                ret, frame = self.cap.read()
                digital_gain = self.digital_exposure_gain
            if ret:
                if abs(digital_gain - 1.0) > 1e-6:
                    frame = cv2.convertScaleAbs(
                        frame, alpha=digital_gain, beta=0.0
                    )
                now = time.time()
                dt = now - self._last_ts
                if dt > 0:
                    inst_fps = 1.0 / dt
                    self.measured_fps = 0.9 * self.measured_fps + 0.1 * inst_fps
                self._last_ts = now

                with self.lock:
                    self.ret = ret
                    self.frame = frame
                    self.timestamp = now
            else:
                time.sleep(0.001)

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None, self.timestamp

    def release(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join()
        self.cap.release()

# ==============================================================================
# MJPEG 视频流服务器
# ==============================================================================
class MJPEGServer:
    def __init__(self, host="0.0.0.0", port=MJPEG_PORT, max_viewers=1):
        self.host = host
        self.port = port
        self.frame = None
        self.lock = threading.Lock()
        self.max_viewers = max_viewers
        self._viewer_count = 0
        self._viewer_lock = threading.Lock()
        self.last_update_t = 0.0
        self.app = Flask(__name__)

        @self.app.route('/video.mjpg')
        def video():
            with self._viewer_lock:
                if self._viewer_count >= self.max_viewers:
                    print(f"视频流已有 {self._viewer_count} 个客户端在看，拒绝新连接")
                    return Response("视频流已被占用，请先关闭其他正在观看的窗口/设备", status=503)
            return Response(
                self.generate(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

    def update(self, frame):
        if frame is None:
            return

        now = time.time()
        if now - self.last_update_t < (1.0 / MJPEG_MAX_FPS):
            return
        self.last_update_t = now

        small_frame = cv2.resize(frame, (MJPEG_STREAM_WIDTH, MJPEG_STREAM_HEIGHT))
        with self.lock:
            self.frame = small_frame

    def generate(self):
        with self._viewer_lock:
            self._viewer_count += 1
            print(f"视频流新增1个观看者，当前共 {self._viewer_count} 个")

        last_send_t = 0.0
        min_interval = 1.0 / MJPEG_MAX_FPS

        try:
            while True:
                now = time.time()
                wait = min_interval - (now - last_send_t)
                if wait > 0:
                    time.sleep(wait)

                with self.lock:
                    frame = None if self.frame is None else self.frame.copy()

                if frame is None:
                    time.sleep(0.01)
                    continue

                ok, jpg = cv2.imencode(
                    '.jpg', frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), MJPEG_JPEG_QUALITY]
                )
                if not ok:
                    continue

                last_send_t = time.time()
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n'
                    + jpg.tobytes() +
                    b'\r\n'
                )
        except GeneratorExit:
            pass
        finally:
            with self._viewer_lock:
                self._viewer_count = max(0, self._viewer_count - 1)
                print(f"视频流断开1个观看者，当前共 {self._viewer_count} 个")

    def start(self):
        threading.Thread(
            target=lambda: self.app.run(
                host=self.host,
                port=self.port,
                threaded=True,
                debug=False,
                use_reloader=False
            ),
            daemon=True
        ).start()
        print(f"MJPEG视频流已启动: http://0.0.0.0:{self.port}/video.mjpg")

# ==============================================================================
# 平板 TCP 服务器（含实时 TX/RX 传输频率计算）
# ==============================================================================
class TabletTCPServer:
    def __init__(self, host=TABLET_TCP_HOST, port=TABLET_TCP_PORT):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.client_addr = None
        self._lock = threading.Lock()
        self._stop = False

        self._accept_thread = None
        self._sender_thread = None
        self._receiver_thread = None

        self._pending_data = None
        self._pending_lock = threading.Lock()
        self._has_pending = threading.Event()

        self._latest_trajectory = None
        self._latest_tracking_err = None
        self._command_queue = queue.Queue()
        self._rx_lock = threading.Lock()

        # 频率计算参数
        self.tx_fps = 0.0
        self.rx_fps = 0.0
        self._last_tx_t = time.time()
        self._last_rx_t = time.time()

        self._init_server()

    def _init_server(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.server_socket.settimeout(1.0)
            print(f"平板 TCP 有线双向服务器已启动: {self.host}:{self.port}")

            self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
            self._accept_thread.start()
            self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
            self._sender_thread.start()
            self._receiver_thread = threading.Thread(target=self._receiver_loop, daemon=True)
            self._receiver_thread.start()
        except Exception as e:
            print(f"平板 TCP 服务器启动失败: {e}")

    def _accept_loop(self):
        while not self._stop:
            try:
                conn, addr = self.server_socket.accept()
                conn.settimeout(0.5)
                with self._lock:
                    if self.client_socket:
                        try:
                            self.client_socket.close()
                        except:
                            pass
                    self.client_socket = conn
                    self.client_addr = addr
                print(f"平板 UI 已建立连接: {addr}")
            except socket.timeout:
                continue
            except Exception:
                continue

    def _sender_loop(self):
        while not self._stop:
            if not self._has_pending.wait(timeout=0.5):
                continue
            with self._pending_lock:
                data_dict = self._pending_data
                self._has_pending.clear()

            with self._lock:
                sock = self.client_socket
            if sock is None or data_dict is None:
                continue

            try:
                json_str = json.dumps(data_dict, ensure_ascii=False, default=float) + "\n"
                sock.sendall(json_str.encode('utf-8'))

                # 计算发送频率
                now = time.time()
                dt = now - self._last_tx_t
                if dt > 0:
                    self.tx_fps = 0.9 * self.tx_fps + 0.1 * (1.0 / dt)
                self._last_tx_t = now

            except (socket.error, BrokenPipeError):
                with self._lock:
                    if self.client_socket:
                        print(f"平板 UI 连接断开: {self.client_addr}")
                        try:
                            self.client_socket.close()
                        except:
                            pass
                        self.client_socket = None

    def _receiver_loop(self):
        buffer = ""
        while not self._stop:
            with self._lock:
                sock = self.client_socket

            if sock is None:
                time.sleep(0.1)
                continue

            try:
                data = sock.recv(2048).decode('utf-8')
                if not data:
                    time.sleep(0.01)
                    continue

                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        # 计算接收频率
                        now = time.time()
                        dt = now - self._last_rx_t
                        if dt > 0:
                            self.rx_fps = 0.9 * self.rx_fps + 0.1 * (1.0 / dt)
                        self._last_rx_t = now

                        try:
                            parsed = json.loads(line)
                            msg_type = parsed.get("type")

                            if msg_type == "trajectory":
                                with self._rx_lock:
                                    self._latest_trajectory = parsed
                                num_pts = parsed.get('total_points', len(parsed.get('points', [])))
                                mode = parsed.get('motion_mode', 'bionic')
                                rev = parsed.get('reverse', False)
                                loop_flag = parsed.get('loop', False)
                                print(f" [UI 消息] 收到轨迹规划: 点数={num_pts} | 模式={mode} | 反向={rev} | 循环={loop_flag}")

                            elif msg_type == "tracking_error":
                                with self._rx_lock:
                                    self._latest_tracking_err = parsed

                            elif msg_type == "command":
                                cmd = parsed.get('cmd')
                                self._command_queue.put(parsed)
                                print(f" [UI 消息] 收到控制指令: 【{cmd.upper()}】")

                        except json.JSONDecodeError:
                            pass

            except socket.timeout:
                continue
            except (socket.error, ConnectionResetError):
                with self._lock:
                    if self.client_socket:
                        print(f"平板 UI 接收通道已断开")
                        try:
                            self.client_socket.close()
                        except:
                            pass
                        self.client_socket = None
            except Exception:
                time.sleep(0.05)

    def send(self, data_dict):
        with self._pending_lock:
            self._pending_data = data_dict
        self._has_pending.set()

    def get_comm_fps(self):
        now = time.time()
        if now - self._last_tx_t > 1.5:
            self.tx_fps *= 0.5
        if now - self._last_rx_t > 1.5:
            self.rx_fps *= 0.5
        return round(self.tx_fps, 1), round(self.rx_fps, 1)

    def get_latest_tracking_error(self):
        with self._rx_lock:
            return self._latest_tracking_err

    def get_latest_trajectory(self):
        with self._rx_lock:
            return self._latest_trajectory

    def get_next_command(self):
        try:
            return self._command_queue.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self._stop = True
        with self._lock:
            if self.client_socket:
                try:
                    self.client_socket.close()
                except:
                    pass
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass

# ==============================================================================
# RoboFish ESP32S3 异步 HTTP 通信类 (含 10Hz 限频防冲垮 + 频率统计)
# ==============================================================================
class RoboFishComm:
    """HTTP management plus a fixed-period, latest-value UDP data plane."""

    def __init__(self, ip=ROBOFISH_IP, port=ROBOFISH_PORT,
                 udp_port=ROBOFISH_UDP_PORT,
                 udp_period_s=VISION_UDP_PERIOD_S):
        self.ip = ip
        self.port = port
        self.base_url = f"http://{ip}:{port}"
        self.udp_port = int(udp_port)
        self.udp_period_s = max(0.01, float(udp_period_s))
        self.cmd_queue = queue.Queue(maxsize=1)  # 队列深度设为 1，只留最新指令
        self.stopped = False

        # 单片机通信频率统计与限频控制
        self.mcu_hz = 0.0
        self._last_send_t = time.time()
        self._last_cmd_t = 0.0                  # 发包间隔时间戳
        self._last_params = None                # 指令去重缓存
        self.vision_seq = 0                     # MCU 视觉 PID 样本序号
        self._http_lock = threading.Lock()

        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_target = (self.ip, self.udp_port)
        self._udp_lock = threading.Lock()
        self._udp_latest = None
        self._udp_latest_t = 0.0
        self._udp_stop = threading.Event()
        self.udp_tx_hz = 0.0
        self.udp_last_send_t = 0.0

        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()
        self._udp_thread = threading.Thread(
            target=self._udp_loop, name="VisionUdpSender", daemon=True
        )
        self._udp_thread.start()
        print(
            f"RoboFish 管理 HTTP + 循迹 UDP 已就绪 -> HTTP {self.base_url}, "
            f"UDP {self.ip}:{self.udp_port} @ {1.0 / self.udp_period_s:.1f}Hz"
        )

    def _request_control(self, variable, value=1, timeout=0.3):
        query_string = urllib.parse.urlencode({"var": variable, "val": value})
        url = f"{self.base_url}/control?{query_string}"
        req = urllib.request.Request(
            url, headers={
                'User-Agent': 'RoboFish-Visual-Tracker',
                'Connection': 'close',
            }
        )
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return response.read().decode('utf-8', errors='replace').strip()

    def _udp_loop(self):
        next_t = time.monotonic()
        while not self._udp_stop.is_set():
            next_t += self.udp_period_s
            wait_s = next_t - time.monotonic()
            if wait_s > 0:
                self._udp_stop.wait(wait_s)
            if self._udp_stop.is_set():
                break
            # If a send or scheduler pause overruns one or more periods,
            # discard the missed slots instead of emitting a burst of frames.
            now = time.monotonic()
            if now - next_t > self.udp_period_s:
                next_t = now
            with self._udp_lock:
                latest = self._udp_latest
                latest_t = self._udp_latest_t
            if latest is None or time.monotonic() - latest_t > VISION_UDP_STALE_S:
                continue
            try:
                self._udp_socket.sendto(
                    pack_vision_udp_frame(**latest), self._udp_target
                )
                now = time.monotonic()
                if self.udp_last_send_t > 0.0:
                    dt = now - self.udp_last_send_t
                    if dt > 0.0:
                        self.udp_tx_hz = 0.9 * self.udp_tx_hz + 0.1 / dt
                self.udp_last_send_t = now
            except OSError as error:
                # UDP is deliberately fire-and-forget; HTTP remains available
                # for management and explicit STOP if the datagram path fails.
                print(f"[UDP 发送失败] {error}")

    def set_tracking_command(self, steering_demand, speed_target_mps,
                             brake_request=False):
        """Publish one latest tracking command; a thread sends it at 20 Hz.

        ``steering_demand`` is positive for a left correction, matching
        :class:`PathGuidance`; the same polarity is carried on the UDP wire.
        """
        self.vision_seq = (self.vision_seq + 1) & 0xFFFFFFFF
        power = int(round(np.clip(
            float(speed_target_mps) / max(1e-6, UDP_SPEED_REFERENCE_MPS) * 100.0,
            0.0, 100.0
        )))
        steering = int(round(np.clip(float(steering_demand) * 100.0, -100.0, 100.0)))
        command = {
            "sequence": self.vision_seq,
            "timestamp_ms": int(time.monotonic_ns() // 1_000_000),
            "power_percent": power,
            "steering_percent": steering,
            "brake": bool(brake_request),
        }
        with self._udp_lock:
            self._udp_latest = command
            self._udp_latest_t = time.monotonic()
        return command

    def stop_tracking(self):
        with self._udp_lock:
            self._udp_latest = None
            self._udp_latest_t = 0.0

    def _request_vision_pid(
        self, sequence, x_error_m, y_error_m, speed_mps,
        curve_severity, brake_request, timeout=0.3
    ):
        query_string = urllib.parse.urlencode({
            "seq": int(sequence),
            "x_error": f"{float(x_error_m):.4f}",
            "y_error": f"{float(y_error_m):.4f}",
            "speed": f"{float(speed_mps):.4f}",
            "curve": f"{float(curve_severity):.3f}",
            "brake": 1 if brake_request else 0,
        })
        url = f"{self.base_url}/vision_pid?{query_string}"
        req = urllib.request.Request(
            url, headers={
                'User-Agent': 'RoboFish-Visual-Tracker',
                'Connection': 'close',
            }
        )
        with urllib.request.urlopen(req, timeout=timeout) as response:
            payload = json.loads(response.read().decode('utf-8'))
        if not payload.get("ok") or int(payload.get("seq", -1)) != int(sequence):
            raise RuntimeError(f"invalid vision PID ACK: {payload}")
        return payload

    def _worker_loop(self):
        while not self.stopped:
            try:
                params = self.cmd_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                if params.get("kind") == "vision_pid":
                    request_started = time.perf_counter()
                    with self._http_lock:
                        payload = self._request_vision_pid(
                            params["seq"], params["x_error"], params["y_error"],
                            params["speed"], params["curve"], params["brake"],
                            timeout=0.3
                        )
                    latency_ms = (time.perf_counter() - request_started) * 1000.0
                    print(
                        f"[VISION ACK] seq={payload['seq']} "
                        f"applied={payload.get('applied_seq', '-')} "
                        f"esc={payload.get('esc', '-')}° "
                        f"RTT={latency_ms:.1f}ms"
                    )
                else:
                    action_map = {
                        "forward": "servo_forward",
                        "backward": "servo_backward",
                        "left": "servo_left",
                        "right": "servo_right",
                        "stop": "servo_stop",
                        "mode_bionic": "mode",
                        "mode_prop": "mode",
                        "mode_hybrid": "mode",
                    }
                    with self._http_lock:
                        self._request_control(action_map[params["action"]], timeout=0.2)

                # 统计单片机交互实际成功 Hz
                now = time.time()
                dt = now - self._last_send_t
                if dt > 0:
                    self.mcu_hz = 0.9 * self.mcu_hz + 0.1 * (1.0 / dt)
                self._last_send_t = now

            except Exception as e:
                if params.get("kind") == "vision_pid":
                    print(
                        f"[VISION ACK 失败] seq={params.get('seq', '-')} "
                        f"x={float(params.get('x_error', 0.0)):+.4f}m "
                        f"y={float(params.get('y_error', 0.0)):.4f}m "
                        f"错误={e}"
                    )
                else:
                    print(f"[HTTP 发送失败] 无法连通机器鱼 ({self.base_url}): {e}")

    def get_mcu_hz(self):
        if self.udp_last_send_t <= 0.0 or time.monotonic() - self.udp_last_send_t > 1.5:
            self.udp_tx_hz *= 0.5
        return round(self.udp_tx_hz, 1)

    def _send_async(self, params):
        now = time.time()
        is_stop = params.get("action") == "stop"

        # 1. 限频控制：单片机限制最高下发频率为 10Hz (间隔>=0.1s)
        if not is_stop and (now - self._last_cmd_t < 0.1):
            return False

        self._last_cmd_t = now
        self._last_params = params

        # 2. 清空队列旧指令；相同动作仍需刷新固件的 2 秒推进租约。
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break

        try:
            self.cmd_queue.put_nowait(params)
            return True
        except queue.Full:
            return False

    def set_mode(self, motion_mode):
        mode_map = {
            "bionic": "mode_bionic",
            "propeller": "mode_prop",
            "prop": "mode_prop",
            "hybrid": "mode_hybrid"
        }
        target_mode = mode_map.get(str(motion_mode).lower(), "mode_bionic")
        print(f"[HTTP 下发] 切换机器鱼模式为: {target_mode}")
        self._send_async({"action": target_mode})

    def ensure_hybrid_mode(self):
        """Reset stale vision sequencing without cycling all drive modes.

        The firmware's /vision_pid handler enters HYBRID atomically when the
        first sample is accepted.  Calling the legacy ``mode`` endpoint here
        would cycle BIONIC -> PROP -> HYBRID and create a misleading startup
        transient on every path restart.
        """
        self.stop_tracking()
        try:
            with self._http_lock:
                self._request_control("servo_stop", timeout=0.5)
            print("[HTTP 视觉准备] 控制序号已复位，首包 PID 将直接进入 HYBRID")
            return True
        except Exception as error:
            print(f"[HTTP 视觉准备失败] {error}")
            return False

    def send_command(self, cmd_str):
        if cmd_str in ["stop", "emergency_stop"]:
            print(f"[HTTP 下发] 机器鱼急停/停止推进")
            self.stop_tracking()
            self._send_async({"action": "stop"})

    def stop_now(self):
        """同步停止，用于退出、清轨迹和失去视觉目标。"""
        self.stop_tracking()
        self._last_params = None
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break
        try:
            with self._http_lock:
                self._request_control("servo_stop", timeout=0.5)
            print("[HTTP 反馈] 推进已停止")
            return True
        except Exception as error:
            print(f"[HTTP 停止失败] {error}")
            return False

    def process_tracking_error(
        self, cross_m, along_m, dist_m, speed_mps=0.0,
        curve_severity=0.0, brake_request=False,
        cross_track_m=0.0, heading_error_deg=0.0,
        path_curvature_per_m=0.0, steering_demand=0.0,
        speed_target_mps=None,
    ):
        """Publish final PC control demand over the fixed-period UDP plane."""
        target_speed = (
            float(speed_target_mps)
            if speed_target_mps is not None and np.isfinite(float(speed_target_mps))
            else float(speed_mps)
        )
        command = self.set_tracking_command(
            steering_demand=float(steering_demand),
            speed_target_mps=target_speed,
            brake_request=bool(brake_request),
        )
        print(
            f"[VISION目标] seq={command['sequence']} "
            f"power={command['power_percent']}% "
            f"steering={command['steering_percent']:+d}% "
            f"v={float(speed_mps):.2f}/{target_speed:.2f}m/s 离={cross_track_m:.2f}m "
            f"角={heading_error_deg:+.0f}° 曲率={path_curvature_per_m:+.2f}/m "
            f"制动={'是' if brake_request else '否'}"
        )

    def close(self):
        self.stopped = True
        self.stop_tracking()
        self._udp_stop.set()
        if self._udp_thread.is_alive():
            self._udp_thread.join(timeout=1.0)
        try:
            self._udp_socket.close()
        except OSError:
            pass
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

# ==============================================================================
# 异步录像写入类
# ==============================================================================
class AsyncVideoRecorder:
    def __init__(self, path, fourcc, fps, frame_size, csv_path):
        self.video_out = cv2.VideoWriter(path, fourcc, fps, frame_size)
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'pixel_u', 'pixel_v', 'physical_x', 'physical_y',
            'yaw_deg', 'raw_yaw_deg', 'yaw_source', 'yaw_usable',
            'yolo_raw_yaw_deg', 'color_raw_yaw_deg', 'color_head_source',
            'head_color_confidence', 'tail_color_confidence',
            'head_u', 'head_v', 'tail_u', 'tail_v', 'pose_valid',
            'pose_rejection_reason',
            # Endpoint provenance fields (added after the legacy columns so
            # existing CSV readers remain compatible with the prefix).
            'yolo_bbox', 'white_bbox', 'white_area',
            'orange_bbox', 'orange_area', 'head_tail_distance_px',
            'visual_yaw_deg', 'fused_yaw_deg', 'motion_prior_deg',
            'motion_gate_applied', 'white_dynamic_value_min',
            'white_tail_distance_gate_px', 'white_rejected_components',
            'yolo_head_in_white', 'yolo_head_distance_to_white_px',
        ])

        self.queue = queue.Queue(maxsize=300)
        self.stopped = False
        self.dropped_frames = 0
        self.thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.thread.start()

    def submit(self, frame, csv_row):
        try:
            self.queue.put_nowait((frame, csv_row))
        except queue.Full:
            self.dropped_frames += 1

    def _writer_loop(self):
        while not (self.stopped and self.queue.empty()):
            try:
                frame, csv_row = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self.video_out.write(frame)
            if csv_row is not None:
                self.csv_writer.writerow(csv_row)

    def close(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join()
        self.video_out.release()
        self.csv_file.close()
        if self.dropped_frames > 0:
            print(f"录制过程中磁盘写入跟不上，丢弃了 {self.dropped_frames} 帧")


# ==============================================================================
# 主函数
# ==============================================================================
def main():
    print("\n" + "=" * 60)
    print("白色鱼头 + 橙色鱼尾 yaw + 路径控制 + 机器鱼视觉工作区正在启动...")
    print(f"工作目录: {os.path.abspath(WORK_DIR)}")
    print(f"目标捕获: {TARGET_WIDTH}x{TARGET_HEIGHT} @ {TARGET_FPS}FPS")
    print(
        f"相机延迟补偿: {CAMERA_LATENCY_S * 1000:.0f} ms "
        f"(最大预测 {CAMERA_LATENCY_MAX_PREDICTION_M:.2f} m)"
    )
    print(
        f"场地映射: 画面上边/TL→TR = {PHYSICAL_WIDTH:.2f} m, "
        f"画面左边/TL→BL = {PHYSICAL_HEIGHT:.2f} m"
    )
    print("画面与操作通过本机工作站转接；控制目标交给工作站统一仲裁。")
    print(f"YOLO 权重: {YOLO_MODEL_PATH}")
    print("=" * 60 + "\n")

    cam = CameraStream(src=CAMERA_INDEX).start()
    if not cam.ret:
        print("无法打开相机！请检查 USB 接口是否松动，或尝试修改索引。")
        cam.release()
        return

    print(f"相机流已就绪：{cam.real_width} x {cam.real_height}")
    if (cam.real_width, cam.real_height) != (TARGET_WIDTH, TARGET_HEIGHT):
        print(
            f"[WARN] camera driver selected {cam.real_width}x{cam.real_height}; "
            f"requested {TARGET_WIDTH}x{TARGET_HEIGHT}"
        )

    # 1. 启动平板 TCP 服务器
    tablet_server = TabletTCPServer(host=TABLET_TCP_HOST, port=TABLET_TCP_PORT)

    # 2. 启动 MJPEG 视频推流服务器
    mjpeg_server = MJPEGServer()
    mjpeg_server.start()

    # 3. 启动 RoboFish ESP32 HTTP 通信与 PID 控制服务
    fish_comm = RoboFishComm(ip=ROBOFISH_IP, port=ROBOFISH_PORT)

    # 4. 构造轻量检测器；模型在窗口显示后由后台线程加载。
    fish_detector = FishDetector(
        model_path=YOLO_MODEL_PATH,
        conf=YOLO_CONF_THRESHOLD,
        imgsz=YOLO_IMG_SIZE,
        device=YOLO_DEVICE,
    )
    print(
        "[VISION MODE] WHITE_HEAD + ORANGE_TAIL ONLY | "
        "YOLO keypoints: identity/search hint only | "
        "yaw requires both colour endpoints"
    )
    # The white marker is the semantic head endpoint for yaw; the orange
    # marker remains the tail anchor and position reference.
    # White marker is the semantic head endpoint.  Its local tracker runs at
    # camera rate and is intentionally responsive; YOLO only supplies a loose
    # search hint/reacquisition box and is never trusted for head polarity.
    white_head_tracker = WhitePointTracker(
        search_radius_px=84.0,
        prediction_horizon_s=0.20,
        velocity_alpha=0.80,
    )

    # 5. 固定色标参考点：本地配置存在时加载，否则明确使用降级模式。
    reference_tracker = FixedReferenceTracker(
        profile_path=MARKER_PROFILE_PATH,
        enable_rigid_body=False,
    )
    if reference_tracker.is_profiled:
        print(f"已加载橙色鱼尾模型: {MARKER_PROFILE_PATH}")
    elif reference_tracker.profile_load_error:
        print(
            "已忽略不合格的旧色标模型："
            f"{reference_tracker.profile_load_error}。请点击【鱼尾标定】重新框选。"
        )
    else:
        print("尚未标定固定色标：位置仅为 ESTIMATED_FALLBACK，不能用于 PWM—速度精标定。")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    use_clahe = ENABLE_CLAHE_DEFAULT

    # Entries are ``(pixel_x, pixel_y, wall_time)``.  Keeping the timestamp
    # with each sample lets the renderer fade old motion instead of retaining
    # an opaque trail for the whole lifetime of the process.
    trajectory = deque(maxlen=TRAJECTORY_MAX_POINTS)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    recorder = None
    is_recording = False
    current_status_text = "READY"
    is_tracing_active = False
    drawn_path_state = {
        'pixels': [],
        'drawing': False,
        'active': False,
        'segment': 0,
        'last_control_t': 0.0,
    }
    try:
        turn_calibration_results = load_turn_calibrations(
            TURN_CALIBRATION_PATH
        )
    except (OSError, ValueError, TypeError) as error:
        turn_calibration_results = {}
        print(f"转圈标定文件无法使用，将回退 0.20m 估计半径：{error}")
    left_turn_fit = turn_calibration_results.get("LEFT")
    right_turn_fit = turn_calibration_results.get("RIGHT")
    path_guidance = PathGuidance(
        spacing_m=0.03,
        lookahead_m=0.24,
        min_lookahead_m=0.10,
        full_attraction_error_m=0.18,
        curve_preview_m=0.32,
        estimated_min_turn_radius_m=0.20,
        left_turn_radius_m=(
            left_turn_fit.radius_m if left_turn_fit is not None else None
        ),
        right_turn_radius_m=(
            right_turn_fit.radius_m if right_turn_fit is not None else None
        ),
        max_x_error_m=(1.0 / 3.0),
        arrival_radius_m=0.20,
        brake_radius_m=0.40,
    )
    turn_calibration_session = TurnCalibrationSession()
    for direction, fit in turn_calibration_results.items():
        direction_name = "左转" if direction == "LEFT" else "右转"
        print(
            f"已加载{direction_name}实测半径 {fit.radius_m:.3f}m "
            f"（直径 {fit.diameter_m:.3f}m，残差 {fit.rms_residual_m:.3f}m）"
        )
    missing_turns = [
        "左转" if direction == "LEFT" else "右转"
        for direction in ("LEFT", "RIGHT")
        if direction not in turn_calibration_results
    ]
    if missing_turns:
        print(
            f"{'、'.join(missing_turns)}尚无实测转圈半径，"
            "曲率前馈暂用 0.20m 估计值。"
        )

    calib_state = {
        'H': None,
        # The pool is stationary.  Keep the inverse beside H and update it
        # only when calibration changes; np.linalg.inv() in the 60 Hz loop
        # needlessly costs CPU and creates temporary arrays.
        'inv_H': None,
        'is_calibrating': False,
        'manual_locked': False,
        'auto_locked': False,
        'auto_stable_count': 0,
        'auto_prev_points': None,
        'pts_raw': [],
        'pts_disp': [],
        'overlay': None,
        'overlay_mask': None,
        'overlay_dirty': True,
    }
    saved_homography, saved_points, calibration_error = load_pool_calibration(
        POOL_CALIBRATION_PATH, cam.real_width, cam.real_height
    )
    if saved_homography is not None:
        calib_state['H'] = saved_homography
        try:
            calib_state['inv_H'] = np.linalg.inv(saved_homography)
        except (TypeError, ValueError, np.linalg.LinAlgError):
            calib_state['H'] = None
            calib_state['inv_H'] = None
            print("保存的场地标定矩阵不可逆，将重新等待标定。")
        if calib_state['H'] is None:
            saved_homography = None
        else:
            calib_state['manual_locked'] = True
            calib_state['pts_raw'] = saved_points
            calib_state['pts_disp'] = [
                (int(round(point[0])), int(round(point[1])))
                for point in saved_points
            ]
            calib_state['overlay_dirty'] = True
            print(
                f"已加载固定水池标定：{POOL_CALIBRATION_PATH}，"
                f"上边 {PHYSICAL_WIDTH:.2f}m、左边 {PHYSICAL_HEIGHT:.2f}m"
            )
    elif calibration_error:
        print(f"固定水池标定未加载，将等待 ArUco/手动标定：{calibration_error}")

    toolbar = VisionToolbar()
    toolbar.binary_threshold = white_head_tracker.val_min
    hud = VisionHud()
    pending_ui_actions = deque()
    frame_state = {
        'latest': None,
        'reference_position': None,
        'reference_source': ReferenceSource.INVALID,
        'tail_marker_position': None,
        # Bounding box of the accepted orange contour.  Kept separate from
        # the reference/body box so the diagnostic view can prove that the
        # yaw tail really came from an orange pixel component.
        'tail_marker_bbox': None,
        'tail_marker_quality': 0.0,
    }
    marker_roi_state = {
        'selecting': False,
        'dragging': False,
        'start': None,
        'end': None,
        'confirmed_bbox': None,
        'confirmed_center': None,
        'confirmed_until': 0.0,
    }
    heading_state = {
        'selecting': False,
        'tail_point': None,
        'head_point': None,
        'unit_vector': None,
        'world_unit_vector': None,
        'control_heading': None,
        'control_heading_source': None,
        'angle_deg': None,
        'confirmed_until': 0.0,
    }
    pointer_state = {
        'pressed_toolbar_action': None,
    }

    # 像素平滑与平滑轨迹控制参数
    pixel_state = {'smooth_x': None, 'smooth_y': None}
    PIXEL_SMOOTH_ALPHA = 0.35      # 像素一阶平滑滤波系数
    MIN_TRAJ_DIST_SQ = 3.0 ** 2    # 移动阈值死区

    pos_state = {'smooth_x': None, 'smooth_y': None, 'frame_time': None}
    velocity_estimator = VelocityEstimator(
        history_seconds=0.40,
        min_span_seconds=0.12,
        blend=0.45,
        max_speed_mps=0.80,
    )
    yaw_filter = BodyYawKalman(
        BodyYawKalmanConfig(
            # Colour endpoints arrive at camera rate and are less ambiguous
            # than the 5 Hz YOLO pose.  Reduce measurement variance and allow
            # more process noise to make turns follow without visible lag.
            measurement_std_deg=2.0,
            # Allow the body-yaw state to follow a real turn between 5 Hz Pose
            # frames instead of holding the original low process noise.
            yaw_acceleration_std_rps2=20.0,
            # 0.20 s is the nominal detector period; 0.40 s covers one miss.
            max_prediction_s=0.30,
            max_innovation_deg=85.0,
        )
    )
    body_yaw_state = {
        'yaw_deg': None,
        'visual_yaw_deg': None,
        'fused_yaw_deg': None,
        'motion_prior_deg': None,
        'motion_gate_applied': False,
        'yaw_rate_rps': None,
        'unit_vector': None,
        'source': 'INVALID',
        'prediction_age_s': None,
        'usable': False,
        'last_frame_time': None,
        'last_update_wall_t': None,
        'raw_yaw_deg': None,
        'measurement_rejected': False,
        'polarity_corrected': False,
        'reacquire_pending': False,
        'reacquire_samples': 0,
        'pose_rejection_reason': None,
        'yolo_raw_yaw_deg': None,
        'color_raw_yaw_deg': None,
        'yaw_measurement_source': 'NONE',
        'color_tail_pixel': None,
        'color_head_pixel': None,
        'color_head_source': 'UNAVAILABLE',
        'head_color_confidence': 0.0,
        'white_dynamic_value_min': None,
        'yolo_head_in_white': None,
        'yolo_head_distance_to_white_px': None,
        'tail_color_confidence': 0.0,
        # A manual head-direction click establishes the 0/180-degree
        # polarity.  YOLO measurements may still be displayed before this,
        # but they are not allowed to start vehicle tracking by themselves.
        'polarity_calibrated': False,
        'tail_pixel': None,
        'head_pixel': None,
    }
    motion_prior_state = {
        'last_t': None,
        'last_r_cmd_rps': 0.0,
        'last_heading_rad': None,
        'last_fused_yaw_deg': None,
        'gate_count': 0,
    }
    # Heading used by the main path controller.  A fresh filtered body yaw
    # takes priority; after a short miss we hold that yaw briefly, then let
    # PathGuidance fall back to its marker-course estimator.
    yaw_control_state = {
        'heading': None,
        'last_valid_t': None,
        'source': 'COURSE',
        'reported_source': None,
    }

    def reset_yaw_control_state():
        yaw_control_state.update({
            'heading': None,
            'last_valid_t': None,
            'source': 'COURSE',
            'reported_source': None,
        })

    def select_yaw_for_control(now):
        """Return ``(heading, yaw_rate, source)`` for PathGuidance.

        ``body_yaw_state`` is updated asynchronously at detector/filter
        cadence.  ``usable`` alone is not enough here because it can remain
        true during a bounded prediction window; prediction age is therefore
        checked explicitly.  A short hold bridges one missed white endpoint,
        after which ``None`` selects the legacy marker-course fallback.
        """
        heading = None
        yaw_rate = 0.0
        source = 'COURSE'
        try:
            prediction_age = body_yaw_state.get('prediction_age_s')
            prediction_age = (
                float(prediction_age)
                if prediction_age is not None and math.isfinite(float(prediction_age))
                else float('inf')
            )
            raw_heading = body_yaw_state.get('unit_vector')
            if (
                bool(body_yaw_state.get('usable'))
                and raw_heading is not None
                and prediction_age <= BODY_YAW_CONTROL_MAX_AGE_S
            ):
                heading_array = np.asarray(raw_heading, dtype=np.float64).reshape(2)
                heading_norm = float(np.linalg.norm(heading_array))
                if np.isfinite(heading_array).all() and heading_norm > 1e-9:
                    heading_array /= heading_norm
                    heading = (
                        float(heading_array[0]),
                        float(heading_array[1]),
                    )
                    raw_rate = body_yaw_state.get('yaw_rate_rps')
                    if raw_rate is not None and math.isfinite(float(raw_rate)):
                        yaw_rate = float(raw_rate)
                    yaw_control_state['heading'] = heading
                    yaw_control_state['last_valid_t'] = float(now)
                    source = 'VISUAL_YAW'
        except (TypeError, ValueError, OverflowError):
            heading = None

        if heading is None:
            held_heading = yaw_control_state.get('heading')
            held_t = yaw_control_state.get('last_valid_t')
            if held_heading is not None and held_t is not None:
                try:
                    hold_age = max(0.0, float(now) - float(held_t))
                except (TypeError, ValueError, OverflowError):
                    hold_age = float('inf')
                if hold_age <= BODY_YAW_CONTROL_HOLD_S:
                    heading = held_heading
                    source = 'VISUAL_YAW_HOLD'

        yaw_control_state['source'] = source
        return heading, yaw_rate, source

    velocity_state = {'vx_smooth': 0.0, 'vy_smooth': 0.0}
    fps_state = {'last_loop_t': time.time(), 'loop_fps': 0.0}
    perf_state = {
        'last_report_t': 0.0,
        'last_endpoint_report_t': 0.0,
    }
    display_state = {
        'last_t': 0.0,
        'last_yaw_draw_t': 0.0,
        'window_width': cam.real_width,
        'window_height': cam.real_height,
    }
    show_debug_overlay = False
    # Keep the diagnostic view in full-frame binary mode until the operator
    # toggles it off with V.  The threshold slider remains visible in the
    # toolbar so the effect of each adjustment is immediately observable.
    show_binary_preview = True
    show_color_aux_overlay = SHOW_COLOR_AUX_OVERLAY_DEFAULT
    show_filtered_yaw_overlay = SHOW_FILTERED_YAW_OVERLAY_DEFAULT
    show_yolo_bbox_overlay = SHOW_YOLO_BBOX_OVERLAY_DEFAULT
    yolo_submit_state = {'last_t': float('-inf')}

    last_motion_mode = None  # 记录上一次发出的运动模式

    window_name = "AUV YOLO Tracker (RoboFish PID)"
    display_width = 960
    display_height = int(display_width * cam.real_height / cam.real_width)
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, display_width, display_height)
    display_state['window_width'] = display_width
    display_state['window_height'] = display_height

    def mouse_callback(event, x, y, flags, param):
        # Git 原版：HighGUI 直接给出原图坐标，不再二次换算。
        rx = int(np.clip(x, 0, cam.real_width - 1))
        ry = int(np.clip(y, 0, cam.real_height - 1))

        # Handle the live binary-threshold slider before buttons/path input.
        slider_hit = toolbar.slider_hit_test(
            rx, ry, cam.real_width, cam.real_height
        )
        if toolbar.slider_dragging:
            if event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONUP):
                white_head_tracker.set_binary_threshold(
                    toolbar.update_slider(rx, cam.real_width, cam.real_height)
                )
            if event == cv2.EVENT_LBUTTONUP:
                toolbar.end_slider_drag()
            drawn_path_state['drawing'] = False
            marker_roi_state['dragging'] = False
            return
        if event == cv2.EVENT_LBUTTONDOWN and slider_hit:
            white_head_tracker.set_binary_threshold(
                toolbar.begin_slider_drag(rx, cam.real_width, cam.real_height)
            )
            drawn_path_state['drawing'] = False
            marker_roi_state['dragging'] = False
            return

        if event in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_LBUTTONUP):
            toolbar_action = toolbar.hit_test(
                rx, ry, cam.real_width, cam.real_height
            )
            if toolbar_action is not None:
                if event == cv2.EVENT_LBUTTONDOWN:
                    pending_ui_actions.append(toolbar_action)
                    pointer_state['pressed_toolbar_action'] = toolbar_action
                elif pointer_state['pressed_toolbar_action'] != toolbar_action:
                    pending_ui_actions.append(toolbar_action)
                pointer_state['pressed_toolbar_action'] = (
                    toolbar_action if event == cv2.EVENT_LBUTTONDOWN else None
                )
                drawn_path_state['drawing'] = False
                marker_roi_state['dragging'] = False
                return
            if toolbar.contains(rx, ry, cam.real_height):
                return

        if marker_roi_state['selecting']:
            canvas_bottom = max(0, cam.real_height - toolbar.height - 1)
            marker_y = int(np.clip(ry, 0, canvas_bottom))
            if event == cv2.EVENT_LBUTTONDOWN:
                marker_roi_state['start'] = (rx, marker_y)
                marker_roi_state['end'] = (rx, marker_y)
                marker_roi_state['dragging'] = True
            elif event == cv2.EVENT_MOUSEMOVE and marker_roi_state['dragging']:
                marker_roi_state['end'] = (rx, marker_y)
            elif event == cv2.EVENT_LBUTTONUP and marker_roi_state['dragging']:
                marker_roi_state['end'] = (rx, marker_y)
                marker_roi_state['dragging'] = False
                start = marker_roi_state['start']
                end = marker_roi_state['end']
                if start is not None and end is not None:
                    pending_ui_actions.append((
                        'APPLY_MARKER_ROI',
                        (start[0], start[1], end[0], end[1])
                    ))
            elif event == cv2.EVENT_RBUTTONDOWN:
                marker_roi_state['selecting'] = False
                marker_roi_state['dragging'] = False
                marker_roi_state['start'] = None
                marker_roi_state['end'] = None
                print("已取消色标框选。")
            return

        if heading_state['selecting']:
            if event == cv2.EVENT_LBUTTONDOWN:
                pending_ui_actions.append(('APPLY_HEAD_DIRECTION', (rx, ry)))
            elif event == cv2.EVENT_RBUTTONDOWN:
                heading_state['selecting'] = False
                print("已取消鱼头方向标定，原有初始方向保持不变。")
            return

        if event == cv2.EVENT_LBUTTONDOWN and calib_state['is_calibrating']:
            calib_state['pts_raw'].append([rx, ry])
            calib_state['pts_disp'].append((rx, ry))
            calib_state['overlay_dirty'] = True
            if len(calib_state['pts_raw']) == 4:
                homography, error = build_calibration_homography(
                    calib_state['pts_raw'], cam.real_width, cam.real_height
                )
                if error:
                    print(f"手动标定失败：{error}，请重新按左上→右上→右下→左下点击。")
                    calib_state['pts_raw'].clear()
                    calib_state['pts_disp'].clear()
                    calib_state['overlay_dirty'] = True
                else:
                    calib_state['H'] = homography
                    calib_state['inv_H'] = np.linalg.inv(homography)
                    calib_state['overlay_dirty'] = True
                    calib_state['manual_locked'] = True
                    calib_state['is_calibrating'] = False
                    calib_state['pts_raw'] = [
                        [float(value) for value in point]
                        for point in calib_state['pts_raw']
                    ]
                    try:
                        save_pool_calibration(
                            POOL_CALIBRATION_PATH,
                            calib_state['pts_raw'],
                            cam.real_width,
                            cam.real_height,
                        )
                    except (OSError, ValueError) as save_error:
                        print(f"场地标定已生效，但保存失败：{save_error}")
                    else:
                        print(f"场地标定已保存：{POOL_CALIBRATION_PATH}")
                    print("手动标定完成并锁定！")

        elif not calib_state['is_calibrating']:
            if event == cv2.EVENT_LBUTTONDOWN:
                drawn_path_state['pixels'] = [(rx, ry)]
                drawn_path_state['drawing'] = True
                drawn_path_state['active'] = False
                drawn_path_state['segment'] = 0
                path_guidance.clear()
            elif event == cv2.EVENT_MOUSEMOVE and drawn_path_state['drawing']:
                if toolbar.contains(rx, ry, cam.real_height):
                    return
                points = drawn_path_state['pixels']
                if not points or (rx - points[-1][0]) ** 2 + (ry - points[-1][1]) ** 2 >= 5 ** 2:
                    points.append((rx, ry))
            elif event == cv2.EVENT_LBUTTONUP and drawn_path_state['drawing']:
                drawn_path_state['drawing'] = False
                if len(drawn_path_state['pixels']) < 2:
                    drawn_path_state['pixels'].clear()
            elif event == cv2.EVENT_RBUTTONDOWN:
                drawn_path_state['pixels'].clear()
                drawn_path_state['drawing'] = False
                drawn_path_state['active'] = False
                drawn_path_state['segment'] = 0
                path_guidance.clear()

    cv2.setMouseCallback(window_name, mouse_callback)
    print("\n鼠标与窗口尺寸已还原 Git 原版。")
    print(
        "底部按钮只保留常用操作；左键拖动画轨迹，右键清除。"
        "按 d 可切换调试叠加层（默认关闭）。\n"
    )

    # 先呈现相机画面，再让后台线程导入 Torch/Ultralytics 和预热 CUDA。
    startup_ret, startup_frame, _ = cam.read()
    if startup_ret and startup_frame is not None:
        cv2.imshow(window_name, startup_frame)
        cv2.waitKey(1)
    fish_detector.start()

    try:
        while True:
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                print("窗口已关闭。")
                break
            ret, frame, frame_time = cam.read()
            if ret and frame is not None:
                web_bridge.raw(frame, frame_time)
            if not ret or frame is None:
                continue

            # 始终处理采集线程提供的最新画面，不再因时间戳暂时未变化而暂停主循环。
            # ``CameraStream.read`` already returns an owned snapshot.  Keep
            # the reference for a possible calibration action instead of
            # copying the 1280x720 image a second time on every frame.
            frame_state['latest'] = frame

            # 计算主循环帧率
            now = time.time()
            render_clock = time.perf_counter()
            render_due = (
                is_recording
                or render_clock - display_state['last_t']
                >= 1.0 / DISPLAY_MAX_FPS
            )
            _UI_DRAW_STATE['allow_text'] = bool(render_due)
            set_text_render_enabled(render_due)
            loop_dt = now - fps_state['last_loop_t']
            fps_state['last_loop_t'] = now
            if loop_dt > 0:
                inst_loop_fps = 1.0 / loop_dt
                fps_state['loop_fps'] = 0.9 * fps_state['loop_fps'] + 0.1 * inst_loop_fps

            img_show = frame.copy()
            # Grayscale is only needed by CLAHE or while ArUco is still being
            # searched.  Once the stationary pool H is locked, skip this full
            # frame conversion on every iteration.
            img_gray = None

            if marker_roi_state['selecting'] and marker_roi_state['start'] is not None:
                marker_end = marker_roi_state['end'] or marker_roi_state['start']
                cv2.rectangle(
                    img_show,
                    marker_roi_state['start'],
                    marker_end,
                    (0, 165, 255),
                    2,
                )

            if (
                marker_roi_state['confirmed_bbox'] is not None
                and time.time() <= marker_roi_state['confirmed_until']
            ):
                confirm_x1, confirm_y1, confirm_x2, confirm_y2 = marker_roi_state['confirmed_bbox']
                confirm_cx, confirm_cy = marker_roi_state['confirmed_center']
                cv2.rectangle(
                    img_show, (confirm_x1, confirm_y1), (confirm_x2, confirm_y2),
                    UiPalette.SUCCESS, 2, cv2.LINE_AA
                )
                cv2.drawMarker(
                    img_show, (int(round(confirm_cx)), int(round(confirm_cy))),
                    UiPalette.SUCCESS, cv2.MARKER_CROSS, 14, 2, cv2.LINE_AA
                )
                draw_unicode_text(
                    img_show, "已自动找到橙色鱼尾中心",
                    (confirm_x1, max(3, confirm_y1 - 22)),
                    14, UiPalette.SUCCESS, bold=True, stroke_width=1
                )

            if len(drawn_path_state['pixels']) > 1:
                path_pixels = np.asarray(
                    drawn_path_state['pixels'], dtype=np.int32
                ).reshape((-1, 1, 2))
                cv2.polylines(
                    img_show, [path_pixels], False, (255, 0, 255), 3,
                    cv2.LINE_AA
                )
                cv2.circle(
                    img_show, tuple(drawn_path_state['pixels'][-1]), 7,
                    (0, 255, 0), -1
                )

            if calib_state['overlay_dirty']:
                (
                    calib_state['overlay'],
                    calib_state['overlay_mask'],
                ) = build_calibration_overlay(
                    frame.shape, calib_state['pts_raw']
                )
                calib_state['overlay_dirty'] = False
            apply_calibration_overlay(
                img_show,
                calib_state['overlay'],
                calib_state['overlay_mask'],
            )

            # 动态 CLAHE 直方图均衡化控制（应对水面强反光）
            needs_gray = use_clahe or (
                calib_state['H'] is None and not calib_state['is_calibrating']
            )
            if needs_gray:
                img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if use_clahe:
                img_gray = clahe.apply(img_gray)

            # YOLO 低频确认身份/协助重捕；橙色色块在主循环中逐帧跟踪。
            if frame_time - yolo_submit_state['last_t'] >= YOLO_DETECT_INTERVAL_S:
                fish_detector.submit_frame(frame, frame_time)
                yolo_submit_state['last_t'] = frame_time

            corner_pixels = {}
            # Manual four-point calibration and automatic ArUco locking are
            # mutually exclusive.  Running both while the user is clicking
            # makes the UI appear to accept points while H is replaced by the
            # background detector.
            if calib_state['H'] is None and not calib_state['is_calibrating']:
                corners, ids, _ = detector.detectMarkers(img_gray)

                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(img_show, corners, ids)
                    ids_flat = ids.flatten()
                    for i in range(len(ids_flat)):
                        marker_id = int(ids_flat[i])
                        cx, cy = np.mean(corners[i][0][:, 0]), np.mean(corners[i][0][:, 1])

                        if marker_id in [MARKER_TL, MARKER_TR, MARKER_BL, MARKER_BR]:
                            corner_pixels[marker_id] = [cx, cy]

            # 获取 YOLO 检测最新结果
            yolo_result = fish_detector.get_latest()
            yolo_status = fish_detector.get_status()
            yolo_pixel = yolo_result['pixel']
            yolo_bbox = yolo_result.get('bbox')
            auv_id_detected = yolo_pixel is not None
            yolo_confidence = yolo_result['confidence']
            yolo_infer_fps = yolo_result.get('infer_fps', 0.0)
            track_id = yolo_result.get('track_id', None)

            # Low-rate diagnostic only; it does not enter control.  This
            # separates camera capture, UI/control loop, and Pose inference
            # when a stale-yaw watchdog trips.
            if now - perf_state['last_report_t'] >= 1.0:
                print(
                    f"[PERF] loop={fps_state['loop_fps']:.1f}Hz "
                    f"camera={cam.measured_fps:.1f}Hz "
                    f"yolo={float(yolo_infer_fps):.1f}Hz "
                    f"pose={'valid' if yolo_result.get('pose_valid') else 'invalid'} "
                    f"pose_reason={yolo_result.get('pose_rejection_reason') or '-'} "
                    f"render={'record' if is_recording else int(DISPLAY_MAX_FPS)}Hz"
                )
                perf_state['last_report_t'] = now

            # Pose yaw uses only the orange tail plus a locally detected white
            # head. Raw YOLO endpoints remain a diagnostic overlay and are not
            # allowed to bypass the colour gates.
            yolo_frame_time = yolo_result.get('frame_time')
            yolo_pose_valid = bool(yolo_result.get('pose_valid'))
            pose_tail = yolo_result.get('tail_pixel')
            pose_head = yolo_result.get('head_pixel')

            # Update the orange tail reference for position/reacquisition.
            # It is deliberately not used as a YOLO head/yaw endpoint.
            reference_observation = reference_tracker.update(
                frame=frame,
                timestamp=frame_time,
                yolo_bbox=yolo_bbox,
                yolo_confidence=yolo_confidence,
                track_id=track_id,
                yolo_timestamp=yolo_result.get('frame_time'),
            )
            auv_pixel = (
                list(reference_observation.position)
                if reference_observation.position is not None
                else None
            )
            frame_state['reference_position'] = (
                tuple(reference_observation.position)
                if reference_observation.position is not None else None
            )
            frame_state['reference_source'] = reference_observation.source
            frame_state['tail_marker_position'] = (
                reference_observation.metrics.get('tail_marker_position')
                if reference_observation.metrics else None
            )
            frame_state['tail_marker_bbox'] = (
                reference_observation.metrics.get('tail_marker_bbox',
                                                   reference_observation.metrics.get('bbox'))
                if reference_observation.metrics else None
            )
            frame_state['tail_marker_quality'] = float(
                reference_observation.metrics.get('tail_marker_quality', 0.0)
                if reference_observation.metrics else 0.0
            )
            body_yaw_state['tail_color_confidence'] = frame_state[
                'tail_marker_quality'
            ]

            # The fish YOLO bbox and orange marker define the white search
            # domain.  A geometrically/temporally gated white component becomes
            # the effective head bbox/centroid; the tail endpoint remains
            # independently constrained to the orange marker.
            head_color_result = white_head_tracker.update(
                frame,
                frame_time,
                # YOLO's semantic head is only validated against the selected
                # white component.  It is never allowed to enlarge the search
                # ROI or bypass the white-region gate when it lands on the
                # orange tail.
                hint=None,
                bbox=yolo_bbox,
                tail=frame_state.get('tail_marker_position'),
                tail_bbox=frame_state.get('tail_marker_bbox'),
                head_hint=pose_head if yolo_pose_valid else None,
            )
            binary_mask_preview = (
                white_head_tracker.binary_preview(frame)
                if show_binary_preview else None
            )
            color_head_confidence = float(head_color_result.get('confidence', 0.0))
            color_head_pixel = head_color_result.get('position')
            body_yaw_state['color_head_source'] = head_color_result.get(
                'source', 'UNAVAILABLE'
            )
            # Store the current observation (including None) in diagnostics;
            # the tracker keeps its own internal state for the next ROI hint.
            body_yaw_state['color_head_pixel'] = color_head_pixel
            body_yaw_state['head_color_confidence'] = color_head_confidence
            body_yaw_state['yolo_head_in_white'] = head_color_result.get(
                'head_hint_inside'
            )
            body_yaw_state['yolo_head_distance_to_white_px'] = head_color_result.get(
                'head_hint_distance_px'
            )
            raw_yaw_deg = None
            yaw_measurement_ready = False
            color_raw_yaw_deg = None
            color_measurement_ready = False
            color_confidence = 0.0
            yaw_measurement_source = 'NONE'
            yaw_tail_pixel = None
            yaw_head_pixel = None
            if yolo_pose_valid and pose_tail is not None and pose_head is not None:
                body_yaw_state['tail_pixel'] = pose_tail
                body_yaw_state['head_pixel'] = pose_head
                if calib_state['H'] is not None:
                    try:
                        pose_pixels = np.float32([[pose_tail, pose_head]])
                        pose_world = cv2.perspectiveTransform(
                            pose_pixels, calib_state['H']
                        )[0]
                        world_delta = pose_world[1] - pose_world[0]
                        world_length = float(np.linalg.norm(world_delta))
                        if world_length >= MIN_YAW_BASELINE_M:
                            raw_yaw_deg = float(np.degrees(np.arctan2(
                                -float(world_delta[1]), float(world_delta[0])
                            )))
                            yaw_measurement_ready = True
                    except (ValueError, np.linalg.LinAlgError, cv2.error):
                        yaw_measurement_ready = False

            # Keep the legacy fields for diagnostics/schema compatibility.
            # White head + orange tail is the preferred measured pair; YOLO
            # pose remains a fallback when the white contour is unavailable.
            tail_color_pixel = frame_state.get('tail_marker_position')
            tail_source = frame_state.get('reference_source')
            if tail_color_pixel is None and reference_observation.source == ReferenceSource.PREDICTED:
                # With the current tail-only reference tracker, PREDICTED
                # position is the extrapolated orange marker itself.  The
                # rigid-body mode may omit this fallback in future; in that
                # case its explicit tail_marker_position metric is preferred.
                tail_color_pixel = reference_observation.position
            # A ``RIGID_BODY`` observation still carries the independently
            # measured orange marker in ``tail_marker_position``.  Treat that
            # marker as valid for yaw just like a plain ``MARKER`` result;
            # only the extrapolated/predicted tail is excluded from a new
            # measurement so a stale endpoint cannot steer the fish.
            tail_measurement_valid = tail_source in {
                ReferenceSource.MARKER,
                ReferenceSource.RIGID_BODY,
            }
            if (
                color_head_pixel is not None
                and tail_color_pixel is not None
                and tail_measurement_valid
                and float(frame_state.get('tail_marker_quality', 0.0) or 0.0) >= 0.38
                and calib_state['H'] is not None
            ):
                try:
                    colour_pixels = np.float32([[tail_color_pixel, color_head_pixel]])
                    colour_world = cv2.perspectiveTransform(
                        colour_pixels, calib_state['H']
                    )[0]
                    colour_delta = colour_world[1] - colour_world[0]
                    if float(np.linalg.norm(colour_delta)) >= MIN_YAW_BASELINE_M:
                        color_raw_yaw_deg = float(np.degrees(np.arctan2(
                            -float(colour_delta[1]), float(colour_delta[0])
                        )))
                        color_measurement_ready = (
                            head_color_result.get('source') == 'WHITE'
                        )
                        tail_quality = max(
                            0.05,
                            float(frame_state.get('tail_marker_quality', 0.0)),
                        )
                        color_confidence = float(np.sqrt(
                            max(0.05, float(head_color_result.get('confidence', 0.0)))
                            * tail_quality
                        ))
                        yaw_measurement_source = (
                            'WHITE_HEAD_TAIL'
                            if head_color_result.get('source') == 'WHITE'
                            else 'WHITE_HEAD_PREDICTION'
                        )
                        if color_measurement_ready:
                            yaw_tail_pixel = tail_color_pixel
                            yaw_head_pixel = color_head_pixel
                except (ValueError, np.linalg.LinAlgError, cv2.error, TypeError):
                    color_measurement_ready = False

            # Strict endpoint policy: yaw is measured only from a real white
            # head contour paired with the orange tail marker.  Raw YOLO pose
            # remains visible for diagnosis but cannot silently bypass the
            # colour gates when either endpoint is missing.
            selected_yaw_deg = color_raw_yaw_deg
            selected_measurement_ready = (
                color_measurement_ready
                and head_color_result.get('source') == 'WHITE'
            )
            selected_confidence = max(0.20, color_confidence) if selected_measurement_ready else 0.0
            if selected_measurement_ready:
                selected_yaw_deg = color_raw_yaw_deg
                yaw_measurement_source = 'WHITE_HEAD_TAIL'
                body_yaw_state['tail_pixel'] = tail_color_pixel
                body_yaw_state['head_pixel'] = color_head_pixel
            else:
                selected_yaw_deg = None
                yaw_measurement_source = 'WHITE_HEAD_WAIT'

            # White/tail endpoints are available at camera rate and feed the
            # filter at a bounded 20 Hz rate.  YOLO pose is lower-rate identity
            # confirmation, so only consume its timestamp when no colour pair
            # is available; this prevents repeatedly applying a stale pose
            # while preserving a clean fallback path before head calibration.
            color_filter_due = (
                color_measurement_ready
                and (
                    body_yaw_state['last_frame_time'] is None
                    or float(frame_time) - float(body_yaw_state['last_frame_time'])
                    >= YAW_FILTER_PERIOD_S
                )
            )
            filter_frame_time = (
                frame_time
                if color_filter_due
                else (None if color_measurement_ready else yolo_frame_time)
            )
            filter_measurement_time_valid = (
                filter_frame_time is not None
                and float(filter_frame_time) > 0.0
                and (
                    body_yaw_state['last_frame_time'] is None
                    or float(filter_frame_time) > float(body_yaw_state['last_frame_time'])
                )
            )
            # Compute the current endpoint distance every frame so it can be
            # persisted in CSV; the human-readable report below is throttled.
            endpoint_distance = None
            if color_head_pixel is not None and tail_color_pixel is not None:
                try:
                    endpoint_distance = float(np.linalg.norm(
                        np.asarray(color_head_pixel, dtype=np.float64)
                        - np.asarray(tail_color_pixel, dtype=np.float64)
                    ))
                except (TypeError, ValueError):
                    endpoint_distance = None
            if now - perf_state['last_endpoint_report_t'] >= 1.0:
                head_source = str(head_color_result.get('source', 'UNAVAILABLE'))
                head_conf = float(head_color_result.get('confidence', 0.0) or 0.0)
                tail_source_name = str(reference_observation.source)
                tail_quality = float(frame_state.get('tail_marker_quality', 0.0) or 0.0)
                distance_text = f"{endpoint_distance:.1f}px" if endpoint_distance is not None else "-"
                yolo_box_text = (
                    "(" + ",".join(str(int(round(float(value)))) for value in yolo_bbox) + ")"
                    if yolo_bbox is not None and len(yolo_bbox) == 4 else "-"
                )
                white_box = head_color_result.get('bbox')
                white_box_text = (
                    "(" + ",".join(str(int(round(float(value)))) for value in white_box) + ")"
                    if white_box is not None and len(white_box) == 4 else "-"
                )
                orange_box = frame_state.get('tail_marker_bbox')
                orange_box_text = (
                    "(" + ",".join(str(int(round(float(value)))) for value in orange_box) + ")"
                    if orange_box is not None and len(orange_box) == 4 else "-"
                )
                white_area = float(head_color_result.get('area', 0.0) or 0.0)
                white_gate = head_color_result.get('tail_distance_gate_px')
                white_gate_text = (
                    f"{float(white_gate):.0f}px"
                    if white_gate is not None else "-"
                )
                white_rejected = int(head_color_result.get('rejected_components', 0) or 0)
                yolo_head_in_white = head_color_result.get('head_hint_inside')
                white_selection = head_color_result.get('selection_mode') or '-'
                orange_area = float(
                    (reference_observation.metrics or {}).get(
                        'tail_marker_area',
                        (reference_observation.metrics or {}).get('area', 0.0),
                    ) or 0.0
                )
                yaw_text = (
                    f"{body_yaw_state['yaw_deg']:+.1f}deg"
                    if body_yaw_state['yaw_deg'] is not None else "-"
                )
                print(
                    f"[ENDPOINT] head={head_source}({head_conf:.2f}) "
                    f"tail={tail_source_name}({tail_quality:.2f}) "
                    f"dist={distance_text} white_area={white_area:.0f} "
                    f"white_v>={head_color_result.get('dynamic_value_min', '-')} "
                    f"white_gate<={white_gate_text} white_rej={white_rejected} "
                    f"white_select={white_selection} "
                    f"yolo_head_in_white={yolo_head_in_white if yolo_head_in_white is not None else '-'} "
                    f"orange_area={orange_area:.0f} "
                    f"yolo_box={yolo_box_text} white_box={white_box_text} "
                    f"orange_box={orange_box_text} yaw_src={yaw_measurement_source} "
                    f"yaw={yaw_text} usable={int(bool(body_yaw_state['usable']))} "
                    f"pose={int(bool(yolo_pose_valid))}"
                )
                perf_state['last_endpoint_report_t'] = now
            if (
                filter_measurement_time_valid
            ):
                filtered_yaw = yaw_filter.step(
                    float(filter_frame_time),
                    math.radians(selected_yaw_deg)
                    if selected_measurement_ready else None,
                    confidence=selected_confidence,
                    measurement_valid=selected_measurement_ready,
                )
                visual_yaw_deg = filtered_yaw['yaw_deg']
                fused_yaw_deg = visual_yaw_deg
                motion_prior_deg = None
                motion_gate_applied = False
                # Integrate the last control command for a short horizon and
                # use it as a soft prior.  This rejects an isolated 180°
                # visual flip but still lets sustained visual motion win.
                if (
                    visual_yaw_deg is not None
                    and motion_prior_state['last_heading_rad'] is not None
                    and motion_prior_state['last_t'] is not None
                ):
                    prior_dt = float(filter_frame_time) - float(
                        motion_prior_state['last_t']
                    )
                    if 0.0 < prior_dt <= MOTION_PRIOR_TIMEOUT_S:
                        prior_rad = _wrap_angle_rad(
                            motion_prior_state['last_heading_rad']
                            + float(motion_prior_state['last_r_cmd_rps'])
                            * prior_dt
                        )
                        motion_prior_deg = math.degrees(prior_rad)
                        visual_rad = math.radians(float(visual_yaw_deg))
                        innovation_rad = _wrap_angle_rad(visual_rad - prior_rad)
                        innovation_deg = abs(math.degrees(innovation_rad))
                        command_activity = min(
                            1.0,
                            abs(float(motion_prior_state['last_r_cmd_rps'])) / 0.60,
                        )
                        prior_weight = MOTION_PRIOR_MAX_BLEND * command_activity
                        if innovation_deg > MOTION_PRIOR_MAX_INNOVATION_DEG:
                            # A near-opposite visual vector is almost always a
                            # head/tail swap or a glare candidate.  Clamp only
                            # the offending sample; do not poison the filter
                            # state or permanently lock out a real turn.
                            clamped = math.radians(
                                math.copysign(
                                    MOTION_PRIOR_CLAMP_DEG, innovation_rad
                                )
                            )
                            fused_yaw_deg = math.degrees(
                                _wrap_angle_rad(prior_rad + clamped)
                            )
                            motion_gate_applied = True
                            motion_prior_state['gate_count'] += 1
                        elif prior_weight > 1e-3:
                            fused_yaw_deg = math.degrees(
                                _blend_angles_rad(
                                    visual_rad, prior_rad, prior_weight
                                )
                            )
                if motion_gate_applied:
                    yaw_source = 'WHITE_HEAD_TAIL_MOTION_GATED'
                    # The filter has already seen the raw sample. Re-seed it
                    # at the bounded fused angle so one bad visual frame does
                    # not leave an erroneous yaw/rate in the next prediction.
                    yaw_filter.reset(
                        math.radians(float(fused_yaw_deg)),
                        float(filter_frame_time),
                    )
                    yaw_filter.set_polarity_locked(
                        bool(body_yaw_state['polarity_calibrated'])
                    )
                else:
                    yaw_source = filtered_yaw['source']
                body_yaw_state.update({
                    'yaw_deg': fused_yaw_deg,
                    'visual_yaw_deg': visual_yaw_deg,
                    'fused_yaw_deg': fused_yaw_deg,
                    'motion_prior_deg': motion_prior_deg,
                    'motion_gate_applied': motion_gate_applied,
                    'yaw_rate_rps': (
                        0.0 if motion_gate_applied
                        else filtered_yaw['yaw_rate_rps']
                    ),
                    'source': yaw_source,
                    'prediction_age_s': filtered_yaw['prediction_age_s'],
                    # Do not expose an unanchored first measurement to the
                    # tracking command.  The manual head-direction click
                    # below establishes the semantic head/tail polarity.
                    'usable': bool(
                        filtered_yaw['usable']
                        and body_yaw_state['polarity_calibrated']
                    ),
                    'raw_yaw_deg': selected_yaw_deg,
                    'yolo_raw_yaw_deg': raw_yaw_deg,
                    'color_raw_yaw_deg': color_raw_yaw_deg,
                    'yaw_measurement_source': yaw_measurement_source,
                    'color_tail_pixel': tail_color_pixel,
                    'color_head_pixel': color_head_pixel,
                    'white_dynamic_value_min': head_color_result.get(
                        'dynamic_value_min'
                    ),
                    'head_color_confidence': float(
                        head_color_result.get('confidence', 0.0)
                    ),
                    'tail_color_confidence': float(
                        frame_state.get('tail_marker_quality', 0.0)
                    ),
                    'pose_rejection_reason': yolo_result.get('pose_rejection_reason'),
                    'measurement_rejected': bool(
                        filtered_yaw.get('measurement_rejected', False)
                    ),
                    'polarity_corrected': bool(
                        filtered_yaw.get('polarity_corrected', False)
                    ),
                    'reacquire_pending': bool(
                        filtered_yaw.get('reacquire_pending', False)
                    ),
                    'reacquire_samples': int(
                        filtered_yaw.get('reacquire_samples', 0)
                    ),
                    'last_frame_time': float(filter_frame_time),
                    'last_update_wall_t': time.monotonic(),
                })
                if body_yaw_state['usable'] and fused_yaw_deg is not None:
                    fused_yaw_rad = math.radians(float(fused_yaw_deg))
                    body_yaw_state['unit_vector'] = (
                        float(math.cos(fused_yaw_rad)),
                        float(-math.sin(fused_yaw_rad)),
                    )
                else:
                    body_yaw_state['unit_vector'] = None
                corrected_yaw_deg = filtered_yaw.get('yaw_deg')
                if now - perf_state['last_endpoint_report_t'] >= 1.0:
                    head_source = str(head_color_result.get('source', 'UNAVAILABLE'))
                    head_conf = float(head_color_result.get('confidence', 0.0) or 0.0)
                    head_area = float(head_color_result.get('area', 0.0) or 0.0)
                    head_bbox = head_color_result.get('bbox')
                    head_box_text = (
                        str(tuple(int(value) for value in head_bbox))
                        if head_bbox is not None else '-'
                    )
                    tail_source_name = str(reference_observation.source)
                    tail_quality = float(frame_state.get('tail_marker_quality', 0.0) or 0.0)
                    tail_area = float(
                        (reference_observation.metrics or {}).get(
                            'tail_marker_area',
                            (reference_observation.metrics or {}).get('area', 0.0),
                        ) or 0.0
                    )
                    tail_gates = ','.join(reference_observation.gate_reasons or []) or '-'
                    white_gate = head_color_result.get('tail_distance_gate_px')
                    white_gate_text = (
                        f"{float(white_gate):.0f}px"
                        if white_gate is not None else "-"
                    )
                    white_rejected = int(
                        head_color_result.get('rejected_components', 0) or 0
                    )
                    yolo_head_in_white = head_color_result.get('head_hint_inside')
                    white_selection = head_color_result.get('selection_mode') or '-'
                    endpoint_distance = None
                    if color_head_pixel is not None and tail_color_pixel is not None:
                        try:
                            endpoint_distance = float(np.linalg.norm(
                                np.asarray(color_head_pixel, dtype=np.float64)
                                - np.asarray(tail_color_pixel, dtype=np.float64)
                            ))
                        except (TypeError, ValueError):
                            endpoint_distance = None
                    distance_text = (
                        f"{endpoint_distance:.1f}px"
                        if endpoint_distance is not None else "-"
                    )
                    yaw_text = (
                        f"{body_yaw_state['yaw_deg']:+.1f}deg"
                        if body_yaw_state['yaw_deg'] is not None else "-"
                    )
                    print(
                        f"[ENDPOINT] head={head_source}({head_conf:.2f},A={head_area:.1f},B={head_box_text}) "
                        f"tail={tail_source_name}({tail_quality:.2f},A={tail_area:.1f}) "
                        f"dist={distance_text} yaw_src={yaw_measurement_source} "
                        f"white_gate<={white_gate_text} white_rej={white_rejected} "
                        f"white_select={white_selection} "
                        f"yolo_head_in_white={yolo_head_in_white if yolo_head_in_white is not None else '-'} "
                        f"motion_gate={int(bool(body_yaw_state['motion_gate_applied']))} "
                        f"visual={body_yaw_state.get('visual_yaw_deg', '-') if body_yaw_state.get('visual_yaw_deg') is not None else '-'} "
                        f"prior={body_yaw_state.get('motion_prior_deg', '-') if body_yaw_state.get('motion_prior_deg') is not None else '-'} "
                        f"yaw={yaw_text} usable={int(bool(body_yaw_state['usable']))} "
                        f"pose={int(bool(yolo_pose_valid))} tail_gate={tail_gates}"
                    )
                    perf_state['last_endpoint_report_t'] = now
                # A colour endpoint can be unavailable while the filter is
                # predicting; keep diagnostic formatting numeric in that case.
                if raw_yaw_deg is None:
                    raw_yaw_deg = float('nan')
                if (
                    filtered_yaw.get('polarity_corrected')
                    and corrected_yaw_deg is not None
                ):
                    print(
                        f"[YOLO YAW] 已纠正头尾极性翻转: "
                        f"raw={raw_yaw_deg:+.1f}° -> "
                        f"filtered={corrected_yaw_deg:+.1f}°"
                    )
                elif filtered_yaw.get('measurement_rejected'):
                    print(
                        f"[YOLO YAW] 丢弃异常测量: raw="
                        f"{raw_yaw_deg if raw_yaw_deg is not None else float('nan'):+.1f}° "
                        f"age={filtered_yaw.get('prediction_age_s')}s"
                    )
                elif filtered_yaw.get('source') == 'REACQUIRE':
                    if int(filtered_yaw.get('reacquire_samples', 0)) == 1:
                        print("[YOLO YAW] 已过期，等待连续 3 帧稳定关键点后重捕获")
                elif filtered_yaw.get('source') == 'REACQUIRED':
                    if corrected_yaw_deg is not None:
                        print(
                            f"[YOLO YAW] 重捕获完成：yaw={corrected_yaw_deg:+.1f}°"
                        )

            # The detector publishes latest-value results.  If its worker is
            # blocked or stopped, no new frame timestamp arrives to the KF;
            # use wall time as a separate watchdog so a stale body yaw cannot
            # remain usable forever.
            last_yaw_update = body_yaw_state['last_update_wall_t']
            if (
                body_yaw_state['usable']
                and last_yaw_update is not None
                and time.monotonic() - last_yaw_update > BODY_YAW_RESULT_TIMEOUT_S
            ):
                body_yaw_state.update({
                    'source': 'EXPIRED',
                    'prediction_age_s': time.monotonic() - last_yaw_update,
                    'usable': False,
                    'unit_vector': None,
                })

            # YOLO 框只表示身份与色标搜索区域；保留初版界面中的鱼框，
            # 详细的头/尾门控来源仍由“调试层”控制。
            if show_yolo_bbox_overlay and yolo_bbox is not None:
                box_x1, box_y1, box_x2, box_y2 = [int(value) for value in yolo_bbox]
                cv2.rectangle(
                    img_show, (box_x1, box_y1), (box_x2, box_y2),
                    (255, 180, 0), 1
                )
                yolo_label = f"机器鱼 #{track_id if track_id is not None else '-'}　{yolo_confidence * 100:.0f}%"
                draw_unicode_text(
                    img_show, yolo_label,
                    (box_x1, max(3, box_y1 - 20)), 13,
                    (255, 180, 0), bold=True, stroke_width=1
                )

            if show_debug_overlay:
                tail_i = (
                    tuple(int(round(value)) for value in pose_tail)
                    if pose_tail is not None else None
                )
                raw_head_i = (
                    tuple(int(round(value)) for value in pose_head)
                    if pose_head is not None else None
                )
                white_head_i = (
                    tuple(int(round(value)) for value in color_head_pixel)
                    if color_head_pixel is not None else None
                )
                white_head_source = str(
                    body_yaw_state.get('color_head_source')
                    or head_color_result.get('source', 'UNAVAILABLE')
                )
                white_head_is_current = white_head_source == 'WHITE'
                yolo_head_in_white = body_yaw_state.get('yolo_head_in_white')
                raw_head_accepted = yolo_head_in_white is True
                # The raw pose point is diagnostic only.  The semantic head is
                # the current white-region centre used by yaw; a predicted
                # white point is shown separately and must not draw the yaw
                # arrow as if it were a fresh measurement.
                semantic_head_i = (white_head_i if white_head_is_current else None) or (
                    raw_head_i if raw_head_accepted else None
                )
                if tail_i is not None:
                    cv2.circle(img_show, tail_i, 5, (0, 165, 255), -1, cv2.LINE_AA)
                    draw_unicode_text(
                        img_show,
                        f"YOLO tail {float(yolo_result.get('tail_confidence', 0.0)):.2f}",
                        (tail_i[0] + 8, tail_i[1] + 16),
                        11, (0, 165, 255), bold=True, stroke_width=1,
                    )
                else:
                    draw_unicode_text(
                        img_show, "YOLO tail MISSING", (16, 118),
                        12, UiPalette.DANGER, bold=True, stroke_width=1,
                    )
                if raw_head_i is not None:
                    # A valid YOLO point is drawn at the gated white centre so
                    # the displayed head cannot split from WHITE head.  A raw
                    # point that lands on the tail remains visible only as a
                    # rejected red diagnostic marker.
                    display_head_i = semantic_head_i if raw_head_accepted else raw_head_i
                    raw_head_colour = (255, 255, 0) if raw_head_accepted else UiPalette.DANGER
                    cv2.circle(
                        img_show, display_head_i, 5, raw_head_colour, -1, cv2.LINE_AA
                    )
                    draw_unicode_text(
                        img_show,
                        (
                            f"YOLO head gated {float(yolo_result.get('head_confidence', 0.0)):.2f}"
                            if raw_head_accepted
                            else "YOLO raw head REJECTED"
                        ),
                        (display_head_i[0] + 8, display_head_i[1] - 8),
                        11, raw_head_colour, bold=True, stroke_width=1,
                    )
                else:
                    draw_unicode_text(
                        img_show, "YOLO head MISSING", (16, 136),
                        12, UiPalette.DANGER, bold=True, stroke_width=1,
                    )
                if yolo_head_in_white is False and white_head_i is not None:
                    cv2.drawMarker(
                        img_show, white_head_i, (255, 255, 0),
                        cv2.MARKER_CROSS, 14, 1, cv2.LINE_AA,
                    )
                    draw_unicode_text(
                        img_show, "HEAD gated -> WHITE",
                        (white_head_i[0] + 9, white_head_i[1] + 20),
                        11, (255, 255, 0), bold=True, stroke_width=1,
                    )

                if tail_i is not None and semantic_head_i is not None:
                    raw_colour = (
                        UiPalette.DANGER
                        if body_yaw_state['measurement_rejected']
                        else UiPalette.MUTED
                    )
                    cv2.arrowedLine(
                        img_show, tail_i, semantic_head_i, raw_colour, 2,
                        cv2.LINE_AA, tipLength=0.22,
                    )
                    if body_yaw_state['measurement_rejected']:
                        draw_unicode_text(
                            img_show, "原始 yaw 跳变已拒绝",
                            (semantic_head_i[0] + 8, max(3, semantic_head_i[1] - 8)),
                            12, UiPalette.DANGER, bold=True, stroke_width=1,
                        )

                if white_head_i is not None:
                    white_draw_colour = (
                        (255, 255, 255)
                        if white_head_is_current else UiPalette.WARNING
                    )
                    if white_head_is_current:
                        cv2.circle(
                            img_show, white_head_i, 7, white_draw_colour, 2,
                            cv2.LINE_AA,
                        )
                    else:
                        cv2.drawMarker(
                            img_show, white_head_i, white_draw_colour,
                            cv2.MARKER_TILTED_CROSS, 14, 1, cv2.LINE_AA,
                        )
                    draw_unicode_text(
                        img_show,
                        (
                            f"WHITE head {color_head_confidence:.2f}"
                            if white_head_is_current
                            else f"WHITE {white_head_source} {color_head_confidence:.2f}"
                        ),
                        (white_head_i[0] + 9, max(3, white_head_i[1] - 20)),
                        12, white_draw_colour, bold=True, stroke_width=1,
                    )

                # Body yaw is drawn by the always-on overlay below.  Keep all
                # endpoint provenance diagnostics in this D-key block.

            # The filtered body-yaw arrow is a primary navigation cue, so keep
            # it visible in the normal view as well as in the optional debug
            # view.  The block above still owns the endpoint provenance marks;
            # this copy deliberately does not depend on the D-key toggle.
            if not show_debug_overlay or body_yaw_state.get('usable'):
                yaw_tail_source = (
                    yaw_tail_pixel
                    if yaw_tail_pixel is not None
                    else (
                        tail_color_pixel
                        if tail_color_pixel is not None
                        else body_yaw_state.get('tail_pixel')
                    )
                )
                yaw_head_source = (
                    yaw_head_pixel
                    if yaw_head_pixel is not None
                    else body_yaw_state.get('head_pixel')
                )
                yaw_draw_due = (
                    is_recording
                    or now - display_state['last_yaw_draw_t']
                    >= 1.0 / DISPLAY_MAX_FPS
                )
                if (
                    body_yaw_state['usable']
                    and body_yaw_state['unit_vector'] is not None
                    and calib_state['H'] is not None
                    and yaw_draw_due
                    and yaw_tail_source is not None
                    and yaw_head_source is not None
                ):
                    try:
                        yaw_tail_i = tuple(
                            int(round(value)) for value in yaw_tail_source
                        )
                        yaw_head_i = tuple(
                            int(round(value)) for value in yaw_head_source
                        )
                        centre_world = cv2.perspectiveTransform(
                            np.float32([[[(yaw_tail_i[0] + yaw_head_i[0]) * 0.5,
                                           (yaw_tail_i[1] + yaw_head_i[1]) * 0.5]]]),
                            calib_state['H'],
                        )[0, 0]
                        tip_world = centre_world + 0.22 * np.asarray(
                            body_yaw_state['unit_vector'], dtype=np.float32
                        )
                        filtered_pixels = cv2.perspectiveTransform(
                            np.float32([[centre_world, tip_world]]),
                            calib_state['inv_H'],
                        )[0]
                        yaw_start = tuple(
                            int(round(value)) for value in filtered_pixels[0]
                        )
                        yaw_tip = tuple(
                            int(round(value)) for value in filtered_pixels[1]
                        )
                        cv2.arrowedLine(
                            img_show, yaw_start, yaw_tip, UiPalette.SUCCESS, 3,
                            cv2.LINE_AA, tipLength=0.24,
                        )
                        yaw_deg = body_yaw_state.get('yaw_deg')
                        if yaw_deg is not None:
                            draw_unicode_text(
                                img_show,
                                f"BODY yaw={float(yaw_deg):+.1f}deg",
                                (yaw_tip[0] + 8, max(3, yaw_tip[1] - 8)),
                                12, UiPalette.SUCCESS, bold=True, stroke_width=1,
                            )
                    except (ValueError, np.linalg.LinAlgError, cv2.error, TypeError):
                        pass
                if yaw_draw_due:
                    display_state['last_yaw_draw_t'] = now

            # If YOLO has only one/no keypoint, retain the white-head diagnostic
            # point when available, but do not draw a filtered arrow from a
            # stale auxiliary endpoint.
            if (
                (pose_tail is None or pose_head is None)
                and show_debug_overlay
                and show_filtered_yaw_overlay
            ):
                if color_head_pixel is not None:
                    color_head_i = tuple(
                        int(round(value)) for value in color_head_pixel
                    )
                    cv2.circle(
                        img_show, color_head_i, 7, (255, 255, 255), 2,
                        cv2.LINE_AA,
                    )
                    draw_unicode_text(
                        img_show,
                        f"WHITE head {color_head_confidence:.2f}",
                        (color_head_i[0] + 9, max(3, color_head_i[1] - 20)),
                        12, (255, 255, 255), bold=True, stroke_width=1,
                    )
                yaw_tail_source = (
                    yaw_tail_pixel if yaw_tail_pixel is not None else pose_tail
                )
                yaw_head_source = (
                    yaw_head_pixel if yaw_head_pixel is not None else pose_head
                )
                if show_filtered_yaw_overlay and (
                    yaw_tail_source is not None
                    and yaw_head_source is not None
                    and body_yaw_state['usable']
                    and body_yaw_state['unit_vector'] is not None
                    and calib_state['H'] is not None
                ):
                    try:
                        tail_i = tuple(int(round(value)) for value in yaw_tail_source)
                        head_i = tuple(int(round(value)) for value in yaw_head_source)
                        centre_world = cv2.perspectiveTransform(
                            np.float32([[[(tail_i[0] + head_i[0]) * 0.5,
                                          (tail_i[1] + head_i[1]) * 0.5]]]),
                            calib_state['H'],
                        )[0, 0]
                        tip_world = centre_world + 0.22 * np.asarray(
                            body_yaw_state['unit_vector'], dtype=np.float32
                        )
                        filtered_pixels = cv2.perspectiveTransform(
                            np.float32([[centre_world, tip_world]]),
                            calib_state['inv_H'],
                        )[0]
                        yaw_start = tuple(int(round(value)) for value in filtered_pixels[0])
                        yaw_tip = tuple(int(round(value)) for value in filtered_pixels[1])
                        cv2.arrowedLine(
                            img_show, yaw_start, yaw_tip, UiPalette.SUCCESS, 3,
                            cv2.LINE_AA, tipLength=0.24,
                        )
                        yaw_deg = body_yaw_state.get('yaw_deg')
                        if yaw_deg is not None:
                            draw_unicode_text(
                                img_show,
                                f"楸间綋 yaw={float(yaw_deg):+.1f}掳 "
                                f"{body_yaw_state['source']}",
                                (yaw_tip[0] + 8, max(3, yaw_tip[1] - 8)),
                                12, UiPalette.SUCCESS, bold=True, stroke_width=1,
                            )
                    except (ValueError, np.linalg.LinAlgError, cv2.error, TypeError):
                        pass

            # ------------------------------------------------------------------
            # 目标平滑绘制与抗锯齿轨迹管理
            # ------------------------------------------------------------------
            if auv_pixel is not None:
                raw_cx, raw_cy = auv_pixel

                if pixel_state['smooth_x'] is None or pixel_state['smooth_y'] is None:
                    pixel_state['smooth_x'] = raw_cx
                    pixel_state['smooth_y'] = raw_cy
                else:
                    pixel_state['smooth_x'] = PIXEL_SMOOTH_ALPHA * raw_cx + (1 - PIXEL_SMOOTH_ALPHA) * pixel_state['smooth_x']
                    pixel_state['smooth_y'] = PIXEL_SMOOTH_ALPHA * raw_cy + (1 - PIXEL_SMOOTH_ALPHA) * pixel_state['smooth_y']

                cx = int(pixel_state['smooth_x'])
                cy = int(pixel_state['smooth_y'])

                source_colours = {
                    ReferenceSource.RIGID_BODY: UiPalette.SUCCESS,
                    ReferenceSource.MARKER: UiPalette.SUCCESS,
                    ReferenceSource.PREDICTED: UiPalette.WARNING,
                    ReferenceSource.ESTIMATED_FALLBACK: UiPalette.WARNING,
                }
                reference_colour = source_colours.get(
                    reference_observation.source, UiPalette.DANGER
                )
                if show_color_aux_overlay:
                    cv2.rectangle(img_show, (cx - 15, cy - 15), (cx + 15, cy + 15), reference_colour, 2)
                    cv2.circle(img_show, (cx, cy), 4, reference_colour, -1)
                    id_text = (
                        f"{hud.SOURCE_NAMES.get(reference_observation.source, reference_observation.source)}　"
                        f"质量 {reference_observation.quality * 100:.0f}%"
                    )
                    draw_unicode_text(
                        img_show, id_text, (cx + 20, max(3, cy - 24)),
                        13, reference_colour, bold=True, stroke_width=1
                    )

                if len(trajectory) == 0:
                    trajectory.append((cx, cy, now))
                else:
                    last_pt = trajectory[-1]
                    dist_sq = (cx - last_pt[0]) ** 2 + (cy - last_pt[1]) ** 2
                    if dist_sq >= MIN_TRAJ_DIST_SQ:
                        trajectory.append((cx, cy, now))
            else:
                pixel_state['smooth_x'] = None
                pixel_state['smooth_y'] = None

            # Draw the swimming trail continuously, including frames where the
            # marker detector has a brief miss.  Each segment is composited
            # with an age-dependent alpha: recent motion is bright yellow,
            # then the trail smoothly fades and is discarded after the timeout.
            while trajectory:
                try:
                    oldest_age = now - float(trajectory[0][2])
                except (IndexError, TypeError, ValueError):
                    trajectory.popleft()
                    continue
                if oldest_age <= TRAJECTORY_MAX_AGE_S:
                    break
                trajectory.popleft()
            if len(trajectory) > 1:
                trail_layer = np.zeros_like(img_show)
                trail_alpha = np.zeros(img_show.shape[:2], dtype=np.uint8)
                fade_span = max(
                    1e-6, TRAJECTORY_MAX_AGE_S - TRAJECTORY_FADE_START_S
                )
                trajectory_points = tuple(trajectory)
                for previous, current in zip(
                    trajectory_points, trajectory_points[1:]
                ):
                    try:
                        age = max(0.0, now - float(current[2]))
                        if age <= TRAJECTORY_FADE_START_S:
                            segment_alpha = 1.0
                        else:
                            segment_alpha = np.clip(
                                (TRAJECTORY_MAX_AGE_S - age) / fade_span,
                                0.0, 1.0,
                            )
                        if segment_alpha <= 0.0:
                            continue
                        start = (int(previous[0]), int(previous[1]))
                        end = (int(current[0]), int(current[1]))
                        cv2.line(
                            trail_layer, start, end, (0, 255, 255), 2,
                            cv2.LINE_AA,
                        )
                        cv2.line(
                            trail_alpha, start, end,
                            int(round(255.0 * float(segment_alpha))), 2,
                            cv2.LINE_AA,
                        )
                    except (IndexError, TypeError, ValueError, OverflowError):
                        continue
                if np.any(trail_alpha):
                    alpha = trail_alpha.astype(np.float32) / 255.0
                    alpha = alpha[..., None]
                    img_show[:] = np.clip(
                        img_show.astype(np.float32) * (1.0 - alpha)
                        + trail_layer.astype(np.float32) * alpha,
                        0.0, 255.0,
                    ).astype(np.uint8)

            if heading_state['selecting'] and frame_state['tail_marker_position'] is not None:
                tail_x, tail_y = [
                    int(round(value))
                    for value in frame_state['tail_marker_position']
                ]
                cv2.drawMarker(
                    img_show, (tail_x, tail_y), UiPalette.PRIMARY,
                    cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA
                )
                draw_unicode_text(
                    img_show, "鱼尾已自动获取，请点击鱼头中心",
                    (min(frame.shape[1] - 235, tail_x + 12), max(3, tail_y - 24)),
                    13, UiPalette.PRIMARY, bold=True, stroke_width=1
                )

            # Endpoint provenance overlay.  These boxes are deliberately
            # drawn independently: white is the largest white component
            # selected *inside* the YOLO fish box; orange is the accepted
            # orange contour used as the only valid tail source.  Seeing all
            # three boxes on the same frame makes cross-target associations
            # and near-coincident endpoints immediately observable.
            if show_debug_overlay:
                white_bbox = head_color_result.get('bbox')
                if white_bbox is not None and len(white_bbox) == 4:
                    wx1, wy1, wx2, wy2 = [int(round(value)) for value in white_bbox]
                    cv2.rectangle(
                        img_show, (wx1, wy1), (wx2, wy2),
                        (255, 255, 255), 2, cv2.LINE_AA,
                    )
                    draw_unicode_text(
                        img_show,
                        f"WHITE max area={float(head_color_result.get('area', 0.0)):.0f}px2",
                        (wx1, max(3, wy1 - 5)), 10,
                        (255, 255, 255), bold=True, stroke_width=1,
                    )
                orange_bbox = frame_state.get('tail_marker_bbox')
                if orange_bbox is not None and len(orange_bbox) == 4:
                    ox1, oy1, ox2, oy2 = [int(round(value)) for value in orange_bbox]
                    cv2.rectangle(
                        img_show, (ox1, oy1), (ox2, oy2),
                        (0, 165, 255), 2, cv2.LINE_AA,
                    )
                    draw_unicode_text(
                        img_show,
                        f"ORANGE tail area={float((reference_observation.metrics or {}).get(
                            'tail_marker_area',
                            (reference_observation.metrics or {}).get('area', 0.0),
                        ) or 0.0):.0f}px2",
                        (ox1, min(frame.shape[0] - 3, oy2 + 14)), 10,
                        (0, 165, 255), bold=True, stroke_width=1,
                    )

            if (
                heading_state['tail_point'] is not None
                and heading_state['head_point'] is not None
                and time.time() <= heading_state['confirmed_until']
            ):
                tail_point = tuple(int(round(value)) for value in heading_state['tail_point'])
                head_point = tuple(int(round(value)) for value in heading_state['head_point'])
                cv2.arrowedLine(
                    img_show, tail_point, head_point, UiPalette.PRIMARY,
                    3, cv2.LINE_AA, tipLength=0.18
                )
                draw_unicode_text(
                    img_show,
                    f"初始鱼头方向 {heading_state['angle_deg']:.1f}°",
                    (min(frame.shape[1] - 160, head_point[0] + 8), max(3, head_point[1] - 22)),
                    13, UiPalette.PRIMARY, bold=True, stroke_width=1
                )

            # 自动标定四个角点
            if len(corner_pixels) == 4 and not calib_state['manual_locked'] and not calib_state['auto_locked']:
                src_pts = np.float32([
                    corner_pixels[MARKER_TL], corner_pixels[MARKER_TR],
                    corner_pixels[MARKER_BR], corner_pixels[MARKER_BL]
                ])
                previous = calib_state['auto_prev_points']
                if previous is not None and float(np.max(np.linalg.norm(src_pts - previous, axis=1))) <= 2.0:
                    calib_state['auto_stable_count'] += 1
                else:
                    calib_state['auto_stable_count'] = 1
                calib_state['auto_prev_points'] = src_pts.copy()

                if calib_state['auto_stable_count'] >= 15:
                    homography, error = build_calibration_homography(
                        src_pts, cam.real_width, cam.real_height
                    )
                    if error:
                        print(f"自动标定未锁定：{error}")
                        calib_state['auto_stable_count'] = 0
                    else:
                        calib_state['H'] = homography
                        calib_state['inv_H'] = np.linalg.inv(homography)
                        calib_state['overlay_dirty'] = True
                        calib_state['auto_locked'] = True
                        # Keep the image points used for H so the dimension
                        # overlay remains visible after auto-calibration locks.
                        calib_state['pts_raw'] = src_pts.tolist()
                        calib_state['pts_disp'] = [
                            (int(round(point[0])), int(round(point[1])))
                            for point in src_pts
                        ]
                        try:
                            save_pool_calibration(
                                POOL_CALIBRATION_PATH,
                                src_pts,
                                cam.real_width,
                                cam.real_height,
                            )
                        except (OSError, ValueError) as save_error:
                            print(f"自动场地标定已生效，但保存失败：{save_error}")
                        else:
                            print(f"自动场地标定已保存：{POOL_CALIBRATION_PATH}")
                        print("自动标定成功！四个场地角点已稳定并锁定。")
            elif len(corner_pixels) < 4:
                calib_state['auto_stable_count'] = 0
                calib_state['auto_prev_points'] = None

            # 解算物理坐标与平滑滤波
            current_x, current_y = None, None
            direct_marker_world_position = None
            control_reference_available = (
                reference_observation.source == ReferenceSource.MARKER
                or reference_observation.source == ReferenceSource.PREDICTED
            )
            if (
                calib_state['H'] is not None
                and auv_pixel is not None
                and control_reference_available
            ):
                pt_real = cv2.perspectiveTransform(
                    np.float32([[[auv_pixel[0], auv_pixel[1]]]]), calib_state['H']
                )
                raw_x, raw_y = float(pt_real[0][0][0]), float(pt_real[0][0][1])
                if reference_observation.source == ReferenceSource.MARKER:
                    direct_marker_world_position = (raw_x, raw_y)

                if pos_state['smooth_x'] is None or pos_state['smooth_y'] is None:
                    pos_state['smooth_x'] = raw_x
                    pos_state['smooth_y'] = raw_y
                elif pos_state['frame_time'] != frame_time:
                    a_pos = POS_SMOOTHING_ALPHA
                    pos_state['smooth_x'] = a_pos * raw_x + (1 - a_pos) * pos_state['smooth_x']
                    pos_state['smooth_y'] = a_pos * raw_y + (1 - a_pos) * pos_state['smooth_y']
                pos_state['frame_time'] = frame_time

                current_x = float(pos_state['smooth_x'])
                current_y = float(pos_state['smooth_y'])
            else:
                pos_state['smooth_x'] = None
                pos_state['smooth_y'] = None
                pos_state['frame_time'] = None

            # 转圈测量只接收橙色色块的直接观测；PREDICTED/YOLO 降级位置
            # 不进入圆拟合，避免“看起来连续”的假轨迹污染半径。
            if (
                turn_calibration_session.active
                and direct_marker_world_position is not None
            ):
                turn_calibration_session.add(
                    direct_marker_world_position, frame_time
                )

            # 短窗口轨迹拟合速度：比逐帧差分更能抑制鱼尾摆动和单帧尖峰。
            speed = None
            direction_deg = None
            velocity = None
            if current_x is not None and current_y is not None:
                velocity = velocity_estimator.update(
                    (current_x, current_y), frame_time
                )
                if velocity is not None:
                    velocity_state['vx_smooth'] = float(velocity[0])
                    velocity_state['vy_smooth'] = float(velocity[1])
                    speed = float(np.linalg.norm(velocity))
                    if speed >= 0.04:
                        direction_deg = float(
                            np.degrees(np.arctan2(velocity[1], velocity[0])) % 360
                        )
            else:
                velocity_estimator.reset()
                velocity_state['vx_smooth'] = 0.0
                velocity_state['vy_smooth'] = 0.0

            control_x, control_y = current_x, current_y
            if current_x is not None and current_y is not None and velocity is not None:
                compensated_position = compensate_camera_latency(
                    (current_x, current_y),
                    velocity,
                    CAMERA_LATENCY_S,
                    CAMERA_LATENCY_MAX_PREDICTION_M,
                )
                control_x = float(compensated_position[0])
                control_y = float(compensated_position[1])

            # ------------------------------------------------------------------
            # 1. 处理 UI 传来的控制指令 (急停/停止)
            # ------------------------------------------------------------------
            cmd_msg = tablet_server.get_next_command()
            if cmd_msg:
                cmd = cmd_msg.get("cmd", "")
                if cmd in ["stop", "emergency_stop"]:
                    current_status_text = "EMERGENCY STOP" if cmd == "emergency_stop" else "STOPPED"
                    is_tracing_active = False
                    motion_prior_state['last_r_cmd_rps'] = 0.0
                    fish_comm.send_command(cmd)

            # ------------------------------------------------------------------
            # 2. 处理 UI 传来的轨迹规划，仅在模式改变时下发一次模式切换
            # ------------------------------------------------------------------
            traj_msg = tablet_server.get_latest_trajectory()
            if traj_msg:
                plan_pts = traj_msg.get("points", [])
                if len(plan_pts) > 1 and calib_state['H'] is not None:
                    try:
                        inv_H = calib_state['inv_H']
                        pts_phys = np.float32([[pt] for pt in plan_pts])
                        pts_pix = cv2.perspectiveTransform(pts_phys, inv_H)

                        for k in range(1, len(pts_pix)):
                            pt1 = (int(pts_pix[k-1][0][0]), int(pts_pix[k-1][0][1]))
                            pt2 = (int(pts_pix[k][0][0]), int(pts_pix[k][0][1]))
                            cv2.line(img_show, pt1, pt2, (255, 0, 255), 2, cv2.LINE_AA)
                            cv2.circle(img_show, pt1, 3, (0, 255, 255), -1)
                    except Exception:
                        pass

            # 平板误差仅保留显示；本机鼠标轨迹独占推进控制。
            err_msg = tablet_server.get_latest_tracking_error()

            # 本机绘制轨迹：START 时已完成米制等距采样；这里更新单向
            # 路径进度、离轨自适应前视点和鱼体坐标左右误差。
            local_path_error = None
            if drawn_path_state['active']:
                if calib_state['H'] is None or len(drawn_path_state['pixels']) < 2:
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    path_guidance.clear()
                    current_status_text = "UNCALIBRATED"
                elif current_x is None or current_y is None:
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    current_status_text = "TARGET LOST"
                    yolo_frame_t = float(yolo_result.get('frame_time') or 0.0)
                    yolo_age_s = (
                        max(0.0, frame_time - yolo_frame_t)
                        if yolo_frame_t > 0.0 else float('inf')
                    )
                    gate_text = ','.join(
                        reference_observation.gate_reasons
                    ) or '无'
                    rejected = reference_observation.metrics.get(
                        'rejected_candidate', {}
                    )
                    rejected_text = ''
                    if rejected:
                        rejected_text = (
                            f"，候选面积比={rejected.get('area_ratio', '-')}"
                            f"，形状比={rejected.get('aspect', '-')}"
                            f"，颜色={rejected.get('color_coverage', '-')}"
                            f"，跳变={rejected.get('jump_px', '-')}px"
                        )
                    print(
                        "橙色色块及短时预测均已失效，轨迹追踪已取消；"
                        f"门控={gate_text}{rejected_text}。"
                    )
                    print(
                        f"[YOLO 诊断] 置信度={yolo_confidence:.2f}，"
                        f"框={'有' if yolo_bbox is not None else '无'}，"
                        f"结果年龄={yolo_age_s:.2f}s。"
                    )
                elif not path_guidance.prepared:
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    current_status_text = "PATH INVALID"
                    print("轨迹引导器尚未准备，请重新点击【启动】。")
                else:
                    body_heading, body_yaw_rate, body_heading_source = (
                        select_yaw_for_control(now)
                    )
                    local_path_error = path_guidance.update(
                        (control_x, control_y),
                        frame_time,
                        # 初版循迹：人工尾→头方向作为启动航向，橙色尾点
                        # 稳定移动后由短窗位移 course 平滑跟随真实转弯。
                        # 白色区域/YOLO 仍负责端点验证和显示，不会因一帧
                        # yaw 丢失而把已经启动的路径追踪强制停掉。
                        allow_course_update=(
                            reference_observation.source == ReferenceSource.MARKER
                        ),
                        speed_mps=(speed if speed is not None else 0.0),
                        velocity=velocity,
                        body_heading=body_heading,
                        body_yaw_rate_rps=body_yaw_rate,
                    )
                    # PathGuidance reports VISUAL_YAW for both a fresh visual
                    # heading and the short hold window.  Preserve the actual
                    # source in the runtime record for diagnostics.
                    local_path_error['control_heading_source'] = (
                        body_heading_source
                    )
                    if body_heading_source != yaw_control_state.get(
                        'reported_source'
                    ):
                        yaw_value = body_yaw_state.get('yaw_deg')
                        age_value = body_yaw_state.get('prediction_age_s')
                        print(
                            f"[HEADING] control={body_heading_source} "
                            f"yaw={yaw_value if yaw_value is not None else '-'} "
                            f"age={age_value if age_value is not None else '-'}"
                        )
                        yaw_control_state['reported_source'] = (
                            body_heading_source
                        )
                    if local_path_error['heading_source'] == 'COURSE':
                        heading_state['control_heading'] = tuple(
                            float(value) for value in local_path_error['heading']
                        )
                        heading_state['control_heading_source'] = 'COURSE'
                    drawn_path_state['segment'] = local_path_error['seg_index']
                    motion_prior_state.update({
                        'last_t': float(frame_time),
                        'last_r_cmd_rps': float(local_path_error.get('r_cmd_rps', 0.0)),
                        'last_heading_rad': math.radians(
                            float(body_yaw_state['yaw_deg'])
                        ) if body_yaw_state.get('yaw_deg') is not None else None,
                        'last_fused_yaw_deg': body_yaw_state.get('yaw_deg'),
                    })

                    # 引导几何属于算法模块；像素绘制仍由主 UI 编排。
                    try:
                        inv_h = calib_state['inv_H']
                        psi_ref_rad = float(local_path_error['psi_ref_rad'])
                        target_heading_world = np.asarray(
                            (math.cos(psi_ref_rad), -math.sin(psi_ref_rad)),
                            dtype=np.float32,
                        )
                        target_heading_tip = (
                            np.asarray((control_x, control_y), dtype=np.float32)
                            + target_heading_world * 0.22
                        )
                        guide_world = np.float32([[
                            local_path_error['lookahead_point'],
                            local_path_error['closest_point'],
                            target_heading_tip,
                        ]])
                        guide_pixels = cv2.perspectiveTransform(
                            guide_world, inv_h
                        )[0]
                        lookahead_pixel = tuple(
                            int(round(value)) for value in guide_pixels[0]
                        )
                        closest_pixel = tuple(
                            int(round(value)) for value in guide_pixels[1]
                        )
                        target_heading_pixel = tuple(
                            int(round(value)) for value in guide_pixels[2]
                        )
                        tail_pixel = tuple(
                            int(round(value)) for value in reference_observation.position
                        )
                        cv2.line(
                            img_show, tail_pixel, lookahead_pixel,
                            (255, 255, 0), 2, cv2.LINE_AA
                        )
                        cv2.circle(img_show, lookahead_pixel, 7, (255, 255, 0), 2)
                        cv2.circle(img_show, closest_pixel, 4, (255, 0, 255), -1)
                        # Target yaw is the path reference (tangent plus
                        # cross-track correction).  The actual body yaw is
                        # drawn separately in green by the endpoint overlay.
                        cv2.arrowedLine(
                            img_show, tail_pixel, target_heading_pixel,
                            UiPalette.PRIMARY, 3, cv2.LINE_AA, tipLength=0.20
                        )
                        draw_unicode_text(
                            img_show,
                            f"TARGET yaw={math.degrees(psi_ref_rad):+.1f}deg",
                            (target_heading_pixel[0] + 8,
                             max(3, target_heading_pixel[1] - 8)),
                            11, UiPalette.PRIMARY, bold=True, stroke_width=1,
                        )
                    except (ValueError, np.linalg.LinAlgError, cv2.error):
                        pass

                    if local_path_error['settled']:
                        fish_comm.stop_now()
                        drawn_path_state['active'] = False
                        is_tracing_active = False
                        reset_yaw_control_state()
                        current_status_text = "ARRIVED"
                        print(
                            "已进入终点 0.20 m 停止圈，"
                            "已发送渐变 STOP。"
                        )
                    elif now - drawn_path_state['last_control_t'] >= VISION_UDP_PERIOD_S:
                        fish_comm.process_tracking_error(
                            local_path_error['x_error_m'],
                            local_path_error['along_m'],
                            local_path_error['drive_distance_m'],
                            speed_mps=(speed if speed is not None else 0.0),
                            curve_severity=local_path_error['curve_severity'],
                            brake_request=local_path_error['brake_request'],
                            cross_track_m=local_path_error['cross_track_m'],
                            heading_error_deg=local_path_error['heading_error_deg'],
                            path_curvature_per_m=(
                                local_path_error['path_curvature_per_m']
                            ),
                            steering_demand=local_path_error['steering_demand'],
                            speed_target_mps=local_path_error['speed_target_mps'],
                        )
                        drawn_path_state['last_control_t'] = now
                        is_tracing_active = True
                        current_status_text = (
                            "HYBRID BRAKING"
                            if local_path_error['brake_request']
                            else "HYBRID TRACKING"
                        )

            # 获取当前实时通信频率
            tab_tx_hz, tab_rx_hz = tablet_server.get_comm_fps()
            mcu_hz = fish_comm.get_mcu_hz()

            # ==============================================================================
            # 统一中文 HUD：紧凑半透明卡片，不再用超大英文覆盖画面。
            display_error = local_path_error if local_path_error is not None else err_msg
            hud_prompt = None
            if turn_calibration_session.active:
                hud_prompt = (
                    "转圈测量：请在网页保持 HYBRID 满左或满右，"
                    f"至少半圈；当前 {turn_calibration_session.sample_count} 点，"
                    "再点【转圈测量】完成"
                )
            elif calib_state['is_calibrating']:
                hud_prompt = (
                    f"场地标定 {len(calib_state['pts_raw'])}/4："
                    "依次点击 左上 → 右上 → 右下 → 左下"
                )
            elif marker_roi_state['selecting']:
                hud_prompt = "鱼尾标定：框住鱼尾附近，程序自动寻找橙色中心；右键取消"
            elif heading_state['selecting']:
                hud_prompt = "鱼头方向：鱼尾自动取点，请点击鱼头中心；右键取消"
            elif (
                calib_state['H'] is not None
                and heading_state['world_unit_vector'] is None
            ):
                hud_prompt = "运行前请点击【鱼头方向】标定初始朝向（消除 180° 歧义）"
            elif yolo_status['loading']:
                hud_prompt = "YOLO / CUDA 正在后台加载，窗口与标定功能可先使用"
            elif yolo_status['error']:
                hud_prompt = f"YOLO 加载失败：{yolo_status['error']}"

            if show_debug_overlay and (
                reference_observation.search_roi is not None
                and reference_observation.source == ReferenceSource.INVALID
            ):
                sx1, sy1, sx2, sy2 = reference_observation.search_roi
                cv2.rectangle(
                    img_show, (sx1, sy1), (sx2, sy2),
                    UiPalette.WARNING, 1, cv2.LINE_AA
                )

            hud.draw(
                img_show,
                source=reference_observation.source,
                status=current_status_text,
                quality=reference_observation.quality,
                pwm_ready=reference_observation.usable_for_pwm_speed,
                position=(current_x, current_y) if current_x is not None and current_y is not None else None,
                speed=speed,
                direction_deg=direction_deg,
                gate_reasons=reference_observation.gate_reasons,
                loop_fps=fps_state['loop_fps'],
                camera_fps=cam.measured_fps,
                yolo_fps=yolo_infer_fps,
                exposure=cam.exposure_val,
                tablet_tx_hz=tab_tx_hz,
                tablet_rx_hz=tab_rx_hz,
                mcu_hz=mcu_hz,
                tracking_error=display_error,
                prompt=hud_prompt,
            )

            if is_recording:
                cv2.circle(img_show, (frame.shape[1] - 22, 107), 5, UiPalette.DANGER, -1)
                draw_unicode_text(
                    img_show, "录像中", (frame.shape[1] - 78, 98),
                    13, UiPalette.DANGER, bold=True
                )

            # ==============================================================================
            # 构建 JSON 数据并推送/保存完整的画面
            # ==============================================================================
            guidance_json = None
            if local_path_error is not None:
                guidance_json = {
                    "x_error_m": round(float(local_path_error['x_error_m']), 4),
                    "body_left_m": round(float(local_path_error['body_left_m']), 4),
                    "heading_error_deg": round(
                        float(local_path_error['heading_error_deg']), 2
                    ),
                    "steering_error_deg": round(
                        float(local_path_error['steering_error_deg']), 2
                    ),
                    "curve_severity": round(
                        float(local_path_error['curve_severity']), 3
                    ),
                    "path_curve_severity": round(
                        float(local_path_error['path_curve_severity']), 3
                    ),
                    "path_curve_angle_deg": round(
                        float(local_path_error['path_curve_angle_deg']), 2
                    ),
                    "path_curvature_per_m": round(
                        float(local_path_error['path_curvature_per_m']), 3
                    ),
                    "curve_feedforward": round(
                        float(local_path_error['curve_feedforward']), 3
                    ),
                    "heading_feedback": round(
                        float(local_path_error['heading_feedback']), 3
                    ),
                    "cross_track_feedback": round(
                        float(local_path_error['cross_track_feedback']), 3
                    ),
                    "steering_demand": round(
                        float(local_path_error['steering_demand']), 3
                    ),
                    "psi_path_deg": round(
                        math.degrees(float(local_path_error['psi_path_rad'])), 2
                    ),
                    "psi_body_deg": round(
                        math.degrees(float(local_path_error['psi_body_rad'])), 2
                    ),
                    "psi_ref_deg": round(
                        math.degrees(float(local_path_error['psi_ref_rad'])), 2
                    ),
                    "r_cmd_rps": round(float(local_path_error['r_cmd_rps']), 3),
                    "cross_track_rate_mps": round(
                        float(local_path_error['cross_track_rate_mps']), 4
                    ),
                    "speed_target_mps": round(
                        float(local_path_error['speed_target_mps']), 4
                    ),
                    "turn_radius_m": round(
                        float(local_path_error['turn_radius_m']), 4
                    ),
                    "turn_radius_source": local_path_error[
                        'turn_radius_source'
                    ],
                    "cross_track_m": round(
                        float(local_path_error['cross_track_m']), 4
                    ),
                    "signed_cross_track_error_m": round(
                        float(local_path_error['signed_cross_track_error_m']), 4
                    ),
                    "off_path_ratio": round(
                        float(local_path_error['off_path_ratio']), 3
                    ),
                    "control_severity": round(
                        float(local_path_error['control_severity']), 3
                    ),
                    "curve_direction": local_path_error['curve_direction'],
                    "lookahead_m": round(
                        float(local_path_error['lookahead_m']), 3
                    ),
                    "brake_request": bool(local_path_error['brake_request']),
                    "heading_source": local_path_error['heading_source'],
                    "control_heading_source": local_path_error.get(
                        'control_heading_source',
                        local_path_error['heading_source'],
                    ),
                    "heading": [
                        round(float(value), 5)
                        for value in local_path_error['heading']
                    ],
                    "lookahead": [
                        round(float(value), 4)
                        for value in local_path_error['lookahead_point']
                    ],
                    "progress_m": round(float(local_path_error['along_m']), 4),
                    "remaining_m": round(float(local_path_error['dist_m']), 4),
                    "drive_distance_m": round(
                        float(local_path_error['drive_distance_m']), 4
                    ),
                    "end_distance_m": round(
                        float(local_path_error['end_distance_m']), 4
                    ),
                    "segment": int(local_path_error['seg_index']),
                }
            json_data = {
                "timestamp": frame_time,
                "track_id": track_id,
                "auv_detected": auv_id_detected,
                "pixel": {
                    "u": float(auv_pixel[0]) if auv_pixel else None,
                    "v": float(auv_pixel[1]) if auv_pixel else None
                },
                "reference": reference_observation.to_dict(),
                "initial_heading": {
                    "calibrated": heading_state['angle_deg'] is not None,
                    "tail_pixel": heading_state['tail_point'],
                    "head_pixel": heading_state['head_point'],
                    "angle_deg": heading_state['angle_deg'],
                    "world_unit": heading_state['world_unit_vector'],
                },
                "body_yaw": {
                    "yaw_deg": body_yaw_state['yaw_deg'],
                    "visual_yaw_deg": body_yaw_state['visual_yaw_deg'],
                    "fused_yaw_deg": body_yaw_state['fused_yaw_deg'],
                    "motion_prior_deg": body_yaw_state['motion_prior_deg'],
                    "motion_gate_applied": bool(
                        body_yaw_state['motion_gate_applied']
                    ),
                    "motion_gate_count": int(motion_prior_state['gate_count']),
                    "raw_yaw_deg": body_yaw_state['raw_yaw_deg'],
                    "yaw_rate_rps": body_yaw_state['yaw_rate_rps'],
                    "source": body_yaw_state['source'],
                    "prediction_age_s": body_yaw_state['prediction_age_s'],
                    "usable": bool(body_yaw_state['usable']),
                    "measurement_rejected": bool(
                        body_yaw_state['measurement_rejected']
                    ),
                    "polarity_corrected": bool(
                        body_yaw_state['polarity_corrected']
                    ),
                    "reacquire_pending": bool(
                        body_yaw_state['reacquire_pending']
                    ),
                    "reacquire_samples": int(
                        body_yaw_state['reacquire_samples']
                    ),
                    "yolo_raw_yaw_deg": body_yaw_state['yolo_raw_yaw_deg'],
                    "color_raw_yaw_deg": body_yaw_state['color_raw_yaw_deg'],
                    "yaw_measurement_source": body_yaw_state[
                        'yaw_measurement_source'
                    ],
                    "color_tail_pixel": body_yaw_state['color_tail_pixel'],
                    "color_head_pixel": body_yaw_state['color_head_pixel'],
                    "color_head_source": body_yaw_state['color_head_source'],
                    "yolo_head_in_white": body_yaw_state['yolo_head_in_white'],
                    "yolo_head_distance_to_white_px": body_yaw_state[
                        'yolo_head_distance_to_white_px'
                    ],
                    "filter_frame_time": body_yaw_state['last_frame_time'],
                    "head_color_confidence": round(
                        float(body_yaw_state['head_color_confidence']), 4
                    ),
                    "white_dynamic_value_min": body_yaw_state[
                        'white_dynamic_value_min'
                    ],
                    "tail_color_confidence": round(
                        float(body_yaw_state['tail_color_confidence']), 4
                    ),
                    "polarity_calibrated": bool(
                        body_yaw_state['polarity_calibrated']
                    ),
                    "tail_pixel": body_yaw_state['tail_pixel'],
                    "head_pixel": body_yaw_state['head_pixel'],
                    "pose_rejection_reason": body_yaw_state['pose_rejection_reason'],
                },
                "turn_calibration": {
                    "active": turn_calibration_session.active,
                    "samples": turn_calibration_session.sample_count,
                    "saved": {
                        direction: fit.to_dict()
                        for direction, fit in turn_calibration_results.items()
                    },
                },
                "guidance": guidance_json,
                "yolo_bbox": [round(float(value), 2) for value in yolo_bbox] if yolo_bbox else None,
                "physical": {
                    "x": round(current_x, 4) if current_x is not None else None,
                    "y": round(current_y, 4) if current_y is not None else None,
                    "control_x": round(control_x, 4) if control_x is not None else None,
                    "control_y": round(control_y, 4) if control_y is not None else None,
                    "camera_latency_s": CAMERA_LATENCY_S,
                },
                "velocity": {
                    "vx": float(round(velocity_state['vx_smooth'], 4)) if speed is not None else None,
                    "vy": float(round(velocity_state['vy_smooth'], 4)) if speed is not None else None,
                    "speed": float(round(speed, 4)) if speed is not None else None,
                    "direction_deg": float(round(direction_deg, 2)) if direction_deg is not None else None
                },
                "calibrated": calib_state['H'] is not None,
                "system": {
                    "loop_fps": round(fps_state['loop_fps'], 1),
                    "camera_fps": round(cam.measured_fps, 1),
                    "yolo_fps": round(yolo_infer_fps, 1),
                    "yolo_confidence": round(yolo_confidence, 3),
                    "yolo_ready": bool(yolo_status['ready']),
                    "yolo_loading": bool(yolo_status['loading']),
                    "yolo_load_seconds": yolo_status['load_seconds'],
                    "clahe_enabled": use_clahe,
                    "exposure": cam.exposure_val,
                    "tablet_tx_hz": tab_tx_hz,
                    "tablet_rx_hz": tab_rx_hz,
                    "mcu_hz": mcu_hz,
                    "udp_tx_hz": mcu_hz,
                    "udp_period_s": VISION_UDP_PERIOD_S,
                }
            }

            if show_binary_preview and binary_mask_preview is not None:
                # Replace the camera view with the full-frame binary mask. The
                # toolbar is drawn afterwards, so threshold adjustment remains
                # available while this diagnostic mode is active.
                img_show = cv2.cvtColor(binary_mask_preview, cv2.COLOR_GRAY2BGR)
                draw_unicode_text(
                    img_show,
                    f"全画面二值化 V={toolbar.binary_threshold}",
                    (12, 120),
                    14,
                    UiPalette.PRIMARY,
                    bold=True,
                )

            toolbar.draw(img_show, active={
                'START': drawn_path_state['active'],
                'STOP': current_status_text in ('STOPPED', 'EMERGENCY STOP'),
                'MARKER_ROI': marker_roi_state['selecting'],
                'HEAD_DIRECTION': heading_state['selecting'],
                'POOL_CALIB': calib_state['is_calibrating'],
                'TURN_CALIB': turn_calibration_session.active,
                'RECORD': is_recording,
                'CLAHE': use_clahe,
                'DEBUG': show_debug_overlay,
            })

            json_data['detector'] = fish_detector.get_status()
            tablet_server.send(json_data)
            if render_due:
                mjpeg_server.update(img_show)

            if is_recording and recorder is not None:
                csv_row = [
                    f"{frame_time:.4f}",
                    f"{auv_pixel[0]:.2f}" if auv_pixel else "",
                    f"{auv_pixel[1]:.2f}" if auv_pixel else "",
                    f"{current_x:.4f}" if current_x is not None else "",
                    f"{current_y:.4f}" if current_y is not None else "",
                    f"{body_yaw_state['yaw_deg']:.3f}"
                    if body_yaw_state['yaw_deg'] is not None else "",
                    f"{body_yaw_state['raw_yaw_deg']:.3f}"
                    if body_yaw_state['raw_yaw_deg'] is not None else "",
                    body_yaw_state['source'],
                    int(bool(body_yaw_state['usable'])),
                    f"{body_yaw_state['yolo_raw_yaw_deg']:.3f}"
                    if body_yaw_state['yolo_raw_yaw_deg'] is not None else "",
                    f"{body_yaw_state['color_raw_yaw_deg']:.3f}"
                    if body_yaw_state['color_raw_yaw_deg'] is not None else "",
                    body_yaw_state['color_head_source'],
                    f"{body_yaw_state['head_color_confidence']:.3f}",
                    f"{body_yaw_state['tail_color_confidence']:.3f}",
                    f"{color_head_pixel[0]:.2f}" if color_head_pixel else "",
                    f"{color_head_pixel[1]:.2f}" if color_head_pixel else "",
                    f"{tail_color_pixel[0]:.2f}" if tail_color_pixel else "",
                    f"{tail_color_pixel[1]:.2f}" if tail_color_pixel else "",
                    int(bool(yolo_pose_valid)),
                    yolo_result.get('pose_rejection_reason') or '',
                    ';'.join(
                        f"{float(value):.1f}" for value in yolo_bbox
                    ) if yolo_bbox is not None and len(yolo_bbox) == 4 else '',
                    ';'.join(
                        f"{float(value):.1f}" for value in color_head_result.get('bbox', ())
                    ) if color_head_result.get('bbox') is not None else '',
                    f"{float(color_head_result.get('area', 0.0) or 0.0):.1f}",
                    ';'.join(
                        f"{float(value):.1f}" for value in frame_state.get('tail_marker_bbox', ())
                    ) if frame_state.get('tail_marker_bbox') is not None else '',
                    f"{float((reference_observation.metrics or {}).get(
                        'tail_marker_area',
                        (reference_observation.metrics or {}).get('area', 0.0),
                    ) or 0.0):.1f}",
                    f"{float(endpoint_distance):.1f}" if endpoint_distance is not None else '',
                    f"{body_yaw_state['visual_yaw_deg']:.3f}"
                    if body_yaw_state['visual_yaw_deg'] is not None else "",
                    f"{body_yaw_state['fused_yaw_deg']:.3f}"
                    if body_yaw_state['fused_yaw_deg'] is not None else "",
                    f"{body_yaw_state['motion_prior_deg']:.3f}"
                    if body_yaw_state['motion_prior_deg'] is not None else "",
                    int(bool(body_yaw_state['motion_gate_applied'])),
                    body_yaw_state['white_dynamic_value_min']
                    if body_yaw_state['white_dynamic_value_min'] is not None else "",
                    head_color_result.get('tail_distance_gate_px')
                    if head_color_result.get('tail_distance_gate_px') is not None else "",
                    int(head_color_result.get('rejected_components', 0) or 0),
                    (int(bool(body_yaw_state['yolo_head_in_white']))
                     if body_yaw_state['yolo_head_in_white'] is not None else ""),
                    body_yaw_state['yolo_head_distance_to_white_px']
                    if body_yaw_state['yolo_head_distance_to_white_px'] is not None else "",
                ]
                recorder.submit(img_show, csv_row)

            _t3 = time.perf_counter()
            if render_due:
                cv2.imshow(window_name, img_show)
                display_state['last_t'] = _t3

            web_bridge.dispatch(mouse_callback, pending_ui_actions)
            # 键盘按键响应
            key = cv2.waitKey(1) & 0xFF
            keyboard_actions = {
                ord('q'): 'EXIT',
                ord(' '): 'STOP',
                ord('['): 'EXP_DOWN',
                ord(']'): 'EXP_UP',
                ord('a'): 'RECORD',
                ord('b'): 'SNAPSHOT',
                ord('t'): 'POOL_CALIB',
                ord('c'): 'CLAHE',
                ord('m'): 'MARKER_ROI',
                ord('d'): 'DEBUG',
                ord('v'): 'BINARY',
            }
            if key in (10, 13):
                pending_ui_actions.append('START')
            elif key in keyboard_actions:
                pending_ui_actions.append(keyboard_actions[key])

            exit_requested = False
            while pending_ui_actions:
                action_item = pending_ui_actions.popleft()
                if isinstance(action_item, tuple):
                    action, action_payload = action_item
                else:
                    action, action_payload = action_item, None

                if action == 'EXIT':
                    print("\n已点击退出。")
                    exit_requested = True
                    break

                if action == 'STOP':
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    current_status_text = "STOPPED"
                    reset_yaw_control_state()
                    fish_comm.stop_now()

                elif action == 'CLEAR_PATH':
                    drawn_path_state['pixels'].clear()
                    drawn_path_state['drawing'] = False
                    drawn_path_state['active'] = False
                    drawn_path_state['segment'] = 0
                    path_guidance.clear()
                    is_tracing_active = False
                    current_status_text = "PATH CLEARED"
                    reset_yaw_control_state()

                elif action == 'START':
                    if turn_calibration_session.active:
                        print("无法启动循迹：请先再次点击【转圈测量】完成本次测量。")
                    elif calib_state['H'] is None:
                        print("无法启动：请先点击【场地标定】完成四角标定。")
                    elif len(drawn_path_state['pixels']) < 2:
                        print("无法启动：请先用鼠标左键拖动画出轨迹。")
                    elif current_x is None or current_y is None:
                        reasons = ','.join(reference_observation.gate_reasons) or '无'
                        print(
                            "无法启动：橙色鱼尾未锁定。"
                            f"来源={reference_observation.source}，门控={reasons}。"
                            "请先重新框选【鱼尾标定】。"
                        )
                    elif heading_state['world_unit_vector'] is None:
                        print("无法启动：请先点击【鱼头方向】并标定当前鱼头。")
                    else:
                        if reference_observation.source == ReferenceSource.MARKER:
                            print("当前使用橙色鱼尾中心追踪。")
                        if reference_observation.source == ReferenceSource.ESTIMATED_FALLBACK:
                            print("当前是无色标降级位置，只允许测试，数据不能用于 PWM—速度精标定。")
                        path_input = np.float32([[
                            [point[0], point[1]]
                            for point in drawn_path_state['pixels']
                        ]])
                        path_world = cv2.perspectiveTransform(
                            path_input,
                            calib_state['H']
                        )[0]
                        # A new run must not inherit a held yaw from the
                        # previous path.  Prefer a fresh calibrated/filtered
                        # body yaw when available, then fall back to the
                        # restored course seed and finally the manual seed.
                        reset_yaw_control_state()
                        startup_visual_heading, _, startup_visual_source = (
                            select_yaw_for_control(now)
                        )
                        startup_heading = (
                            startup_visual_heading
                            or heading_state['control_heading']
                            or heading_state['world_unit_vector']
                        )
                        startup_heading_source = (
                            startup_visual_source
                            if startup_visual_heading is not None
                            else (
                                heading_state['control_heading_source']
                                or 'CALIBRATED'
                            )
                        )
                        try:
                            initial_guidance = path_guidance.start(
                                path_world,
                                (control_x, control_y),
                                frame_time,
                                startup_heading,
                            )
                        except (ValueError, RuntimeError) as error:
                            path_guidance.clear()
                            print(f"无法准备轨迹：{error}")
                        else:
                            start_distance = float(np.linalg.norm(
                                np.asarray((control_x, control_y))
                                - path_guidance.path[0]
                            ))
                            if start_distance > 0.40:
                                path_guidance.clear()
                                print(
                                    f"无法启动：轨迹起点距离鱼 {start_distance:.2f}m，"
                                    "请从鱼附近开始画。"
                                )
                            elif fish_comm.ensure_hybrid_mode():
                                fish_comm.vision_seq = 0
                                drawn_path_state['active'] = True
                                drawn_path_state['segment'] = initial_guidance['seg_index']
                                drawn_path_state['last_control_t'] = 0.0
                                is_tracing_active = True
                                current_status_text = "HYBRID TRACKING"
                                print(
                                    "HYBRID 前视轨迹引导已启动："
                                    f"{len(path_guidance.path)} 个等距点，"
                                    "真实曲率前馈 + 航向/离轨反馈，"
                                    "前视 0.10~0.24 m，"
                                    "UDP 固定 20 Hz。"
                                )
                                first_error = float(
                                    initial_guidance['steering_error_deg']
                                )
                                first_turn = (
                                    '左'
                                    if first_error > 3.0
                                    else '右'
                                    if first_error < -3.0
                                    else '直行'
                                )
                                source_text = (
                                    '上次运动航向'
                                    if startup_heading_source == 'COURSE'
                                    else '人工鱼头方向'
                                )
                                print(
                                    f"起步使用{source_text}，"
                                    f"首个引导={first_turn} {abs(first_error):.1f}°。"
                                )
                            else:
                                path_guidance.clear()
                                print("无法确认机器鱼进入 HYBRID 模式，未启动追踪。")

                elif action == 'TURN_CALIB':
                    if turn_calibration_session.active:
                        try:
                            turn_fit = turn_calibration_session.finish()
                        except TurnCalibrationError as error:
                            current_status_text = "READY"
                            print(f"转圈测量未通过：{error}")
                            print("本次结果未保存，请重新点击【转圈测量】再测。")
                        else:
                            try:
                                save_turn_calibration(
                                    TURN_CALIBRATION_PATH, turn_fit
                                )
                            except OSError as error:
                                current_status_text = "READY"
                                print(f"转圈结果保存失败：{error}")
                            else:
                                turn_calibration_results[
                                    turn_fit.direction
                                ] = turn_fit
                                path_guidance.set_turn_radius(
                                    turn_fit.direction, turn_fit.radius_m
                                )
                                current_status_text = "TURN CALIBRATED"
                                direction_name = (
                                    "左转"
                                    if turn_fit.direction == "LEFT"
                                    else "右转"
                                )
                                print(
                                    f"{direction_name}满转测量完成："
                                    f"直径={turn_fit.diameter_m:.3f}m，"
                                    f"半径={turn_fit.radius_m:.3f}m，"
                                    f"曲率={turn_fit.curvature_per_m:.2f}/m"
                                )
                                print(
                                    f"质量：有效点={turn_fit.inlier_count}/"
                                    f"{turn_fit.sample_count}，"
                                    f"圆弧={turn_fit.arc_coverage_deg:.1f}°，"
                                    f"RMS残差={turn_fit.rms_residual_m:.3f}m。"
                                )
                                print(
                                    "该方向的曲率前馈已改用实测半径；"
                                    "另一方向仍保持原实测值或 0.20m 估计值。"
                                )
                    elif calib_state['H'] is None:
                        print("无法测量：请先点击【场地标定】完成四角标定。")
                    elif (
                        direct_marker_world_position is None
                    ):
                        print(
                            "无法测量：当前必须直接锁定橙色鱼尾；"
                            "PREDICTED 和 YOLO 降级位置不会用于拟合。"
                        )
                    else:
                        if drawn_path_state['active'] or is_tracing_active:
                            drawn_path_state['active'] = False
                            is_tracing_active = False
                            fish_comm.stop_now()
                            print("已结束视觉循迹，准备被动测量转圈轨迹。")
                        calib_state['is_calibrating'] = False
                        marker_roi_state['selecting'] = False
                        heading_state['selecting'] = False
                        turn_calibration_session.start(
                            direct_marker_world_position, frame_time
                        )
                        current_status_text = "TURN CALIBRATING"
                        print(
                            "转圈测量已开始：请在网页端选择 HYBRID，"
                            "持续满左或满右转至少半圈。"
                        )
                        print(
                            "程序只记录橙色色块直接观测，不会自动控制鱼；"
                            "完成后再次点击【转圈测量】。"
                        )

                elif action == 'MARKER_ROI':
                    if turn_calibration_session.active:
                        turn_calibration_session.cancel()
                        print("已取消未完成的转圈测量。")
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    calib_state['is_calibrating'] = False
                    heading_state['selecting'] = False
                    marker_roi_state['selecting'] = not marker_roi_state['selecting']
                    marker_roi_state['dragging'] = False
                    marker_roi_state['start'] = None
                    marker_roi_state['end'] = None
                    if marker_roi_state['selecting']:
                        current_status_text = "SELECT MARKER ROI"
                        print("请框住鱼尾附近区域，程序会自动寻找橙色中心；右键或再次点击【鱼尾标定】取消。")
                    else:
                        current_status_text = "READY"
                        print("已取消色标框选，原有色标模型保持不变。")

                elif action == 'APPLY_MARKER_ROI':
                    latest_frame = frame_state['latest']
                    try:
                        profile = reference_tracker.calibrate_from_roi(
                            latest_frame, action_payload, save=True
                        )
                        calibration_result = reference_tracker.last_calibration
                        marker_roi_state['selecting'] = False
                        marker_roi_state['dragging'] = False
                        marker_roi_state['start'] = None
                        marker_roi_state['end'] = None
                        marker_roi_state['confirmed_bbox'] = calibration_result['marker_bbox']
                        marker_roi_state['confirmed_center'] = calibration_result['marker_center']
                        marker_roi_state['confirmed_until'] = time.time() + 3.0
                        current_status_text = "MARKER READY"
                        print(
                            "已自动找到橙色鱼尾中心并保存模型："
                            f"center=({calibration_result['marker_center'][0]:.1f},"
                            f"{calibration_result['marker_center'][1]:.1f}), "
                            f"H={profile.hue_center:.1f}±{profile.hue_tolerance:.1f}, "
                            f"area={profile.expected_area:.1f}px²"
                        )
                    except Exception as error:
                        marker_roi_state['selecting'] = True
                        marker_roi_state['dragging'] = False
                        marker_roi_state['start'] = None
                        marker_roi_state['end'] = None
                        current_status_text = "MARKER ROI RETRY"
                        print(f"色标框选失败：{error}。请调整范围后重新框选。")

                elif action == 'HEAD_DIRECTION':
                    if turn_calibration_session.active:
                        turn_calibration_session.cancel()
                        print("已取消未完成的转圈测量。")
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    calib_state['is_calibrating'] = False
                    marker_roi_state['selecting'] = False
                    heading_state['selecting'] = not heading_state['selecting']
                    if heading_state['selecting']:
                        current_status_text = "SELECT HEAD DIRECTION"
                        print("鱼尾中心将自动获取，请直接点击鱼头中心；右键或再次点击【鱼头方向】取消。")
                    else:
                        current_status_text = "READY"
                        print("已取消鱼头方向标定，原有初始方向保持不变。")

                elif action == 'APPLY_HEAD_DIRECTION':
                    tail_point = frame_state['tail_marker_position']
                    if calib_state['H'] is None:
                        current_status_text = "SELECT HEAD DIRECTION"
                        print("请先完成场地标定，再标定鱼头方向。")
                    elif (
                        tail_point is None
                    ):
                        current_status_text = "SELECT HEAD DIRECTION"
                        print("当前没有可靠的橙色鱼尾中心，请先完成鱼尾标定并让鱼尾保持可见。")
                    else:
                        head_point = tuple(float(value) for value in action_payload)
                        tail_point = tuple(float(value) for value in tail_point)
                        print("已采用手动点击的白色鱼头位置作为航向极性初始化点。")
                        dx = head_point[0] - tail_point[0]
                        dy = head_point[1] - tail_point[1]
                        length = float(np.hypot(dx, dy))
                        if length <= 1e-6:
                            current_status_text = "SELECT HEAD DIRECTION"
                            print("鱼头点离鱼尾太近，请准确点击鱼头中心。")
                        else:
                            world_points = cv2.perspectiveTransform(
                                np.float32([[tail_point, head_point]]),
                                calib_state['H'],
                            )[0]
                            world_delta = world_points[1] - world_points[0]
                            world_length = float(np.linalg.norm(world_delta))
                            if world_length <= 1e-6:
                                current_status_text = "SELECT HEAD DIRECTION"
                                print("鱼头方向映射失败，请重新点击。")
                            else:
                                world_unit = world_delta / world_length
                                heading_state['tail_point'] = tail_point
                                heading_state['head_point'] = head_point
                                heading_state['unit_vector'] = (dx / length, dy / length)
                                heading_state['world_unit_vector'] = (
                                    float(world_unit[0]), float(world_unit[1])
                                )
                                heading_state['control_heading'] = (
                                    float(world_unit[0]), float(world_unit[1])
                                )
                                heading_state['control_heading_source'] = 'CALIBRATED'
                                heading_state['angle_deg'] = float(
                                    (
                                        np.degrees(np.arctan2(-world_unit[1], world_unit[0]))
                                        + 360.0
                                    ) % 360.0
                                )
                                # Manual click is only an initialization
                                # seed; subsequent control uses the colour
                                # endpoint pair (white head + orange tail).
                                # whenever a valid semantic pair is available.
                                yaw_filter.reset(
                                    math.radians(heading_state['angle_deg']),
                                    frame_time,
                                )
                                yaw_filter.set_polarity_locked(True)
                                body_yaw_state.update({
                                    'yaw_deg': heading_state['angle_deg'],
                                    'yaw_rate_rps': 0.0,
                                    'unit_vector': (
                                        float(world_unit[0]),
                                        float(world_unit[1]),
                                    ),
                                    'source': 'CALIBRATED_SEED',
                                    'prediction_age_s': 0.0,
                                    'usable': True,
                                    'measurement_rejected': False,
                                    'polarity_corrected': False,
                                    'reacquire_pending': False,
                                    'reacquire_samples': 0,
                                    'pose_rejection_reason': None,
                                    'polarity_calibrated': True,
                                    'last_frame_time': float(frame_time),
                                    'last_update_wall_t': time.monotonic(),
                                    'raw_yaw_deg': heading_state['angle_deg'],
                                })
                                heading_state['confirmed_until'] = time.time() + 5.0
                                heading_state['selecting'] = False
                                current_status_text = "HEAD DIRECTION READY"
                                print(
                                    "初始鱼头方向标定完成："
                                    f"尾点=({tail_point[0]:.1f},{tail_point[1]:.1f})，"
                                    f"头点=({head_point[0]:.1f},{head_point[1]:.1f})，"
                                    f"方向={heading_state['angle_deg']:.1f}°（右为 0°，上为 90°）。"
                                )

                elif action == 'POOL_CALIB':
                    if turn_calibration_session.active:
                        turn_calibration_session.cancel()
                        print("已取消未完成的转圈测量。")
                    calib_state['is_calibrating'] = not calib_state['is_calibrating']
                    reset_yaw_control_state()
                    marker_roi_state['selecting'] = False
                    heading_state['selecting'] = False
                    if calib_state['is_calibrating']:
                        white_head_tracker.reset()
                        drawn_path_state['active'] = False
                        drawn_path_state['pixels'].clear()
                        path_guidance.clear()
                        heading_state['tail_point'] = None
                        heading_state['head_point'] = None
                        heading_state['unit_vector'] = None
                        heading_state['world_unit_vector'] = None
                        heading_state['angle_deg'] = None
                        yaw_filter.reset()
                        yaw_filter.set_polarity_locked(False)
                        motion_prior_state.update({
                            'last_t': None,
                            'last_r_cmd_rps': 0.0,
                            'last_heading_rad': None,
                            'last_fused_yaw_deg': None,
                            'gate_count': 0,
                        })
                        body_yaw_state.update({
                            'yaw_deg': None,
                            'visual_yaw_deg': None,
                            'fused_yaw_deg': None,
                            'motion_prior_deg': None,
                            'motion_gate_applied': False,
                            'yaw_rate_rps': None,
                            'unit_vector': None,
                            'source': 'INVALID',
                            'prediction_age_s': None,
                            'usable': False,
                            'measurement_rejected': False,
                            'polarity_corrected': False,
                            'reacquire_pending': False,
                            'reacquire_samples': 0,
                            'pose_rejection_reason': None,
                            'polarity_calibrated': False,
                            'last_frame_time': None,
                            'last_update_wall_t': None,
                            'raw_yaw_deg': None,
                            'yolo_raw_yaw_deg': None,
                            'color_raw_yaw_deg': None,
                            'color_tail_pixel': None,
                            'color_head_pixel': None,
                            'color_head_source': 'UNAVAILABLE',
                            'head_color_confidence': 0.0,
                            'white_dynamic_value_min': None,
                            'tail_color_confidence': 0.0,
                            'yaw_measurement_source': 'NONE',
                        })
                        is_tracing_active = False
                        current_status_text = "CALIBRATING"
                        calib_state['H'] = None
                        calib_state['inv_H'] = None
                        calib_state['overlay_dirty'] = True
                        calib_state['manual_locked'] = False
                        calib_state['auto_locked'] = False
                        calib_state['auto_stable_count'] = 0
                        calib_state['auto_prev_points'] = None
                        calib_state['pts_raw'].clear()
                        calib_state['pts_disp'].clear()
                        calib_state['overlay_dirty'] = True
                        pos_state['smooth_x'] = None
                        pos_state['smooth_y'] = None
                        print("请按左上→右上→右下→左下依次点击四个场地角点。")
                    else:
                        calib_state['pts_raw'].clear()
                        calib_state['pts_disp'].clear()
                        calib_state['overlay_dirty'] = True
                        current_status_text = "UNCALIBRATED"
                        print("已取消场地标定；当前没有有效标定矩阵。")

                elif action == 'RECORD':
                    is_recording = not is_recording
                    if is_recording:
                        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
                        record_fps = max(1.0, round(cam.measured_fps, 1))
                        recorder = AsyncVideoRecorder(
                            path=f'{RECORD_DIR}/auv_record_{timestamp_str}.mp4',
                            fourcc=fourcc,
                            fps=record_fps,
                            frame_size=(cam.real_width, cam.real_height),
                            csv_path=f'{RECORD_DIR}/auv_data_{timestamp_str}.csv'
                        )
                        print(f"开始录制... 帧率: {record_fps}fps  数据保存在: {WORK_DIR}")
                    else:
                        if recorder:
                            recorder.close()
                            recorder = None
                        print("停止录制并保存文件。")

                elif action == 'SNAPSHOT':
                    ms = int((time.time() * 1000) % 1000)
                    img_filename = f"{RECORD_DIR}/auv_frame_{time.strftime('%Y%m%d_%H%M%S')}_{ms:03d}.jpg"
                    cv2.imwrite(img_filename, img_show)
                    print(f"抓拍成功！已保存为: {img_filename}")

                elif action == 'CLAHE':
                    use_clahe = not use_clahe
                    print(f"CLAHE 状态切换: {'【开启】' if use_clahe else '【关闭】'}")

                elif action == 'DEBUG':
                    show_debug_overlay = not show_debug_overlay
                    current_status_text = (
                        "DEBUG OVERLAY" if show_debug_overlay else "READY"
                    )
                    print(
                        f"调试层{'开启' if show_debug_overlay else '关闭'}："
                        "仅影响画面叠加，不影响检测、滤波或控制"
                    )

                elif action == 'BINARY':
                    show_binary_preview = not show_binary_preview
                    print(
                        f"全画面二值化{'开启' if show_binary_preview else '关闭'}"
                    )

                elif action == 'EXP_DOWN':
                    cam.adjust_exposure(-1)

                elif action == 'EXP_UP':
                    cam.adjust_exposure(1)

            web_bridge.complete(current_status_text, drawn_path_state['active'])
            if exit_requested:
                break

    except Exception as e:
        print(f"运行过程中发生未捕获异常: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n正在清理系统资源并关闭网络服务...")
        fish_comm.close()
        tablet_server.close()
        fish_detector.close()
        cam.release()
        if recorder:
            recorder.close()
        cv2.destroyAllWindows()
        print("所有通信接口与相机资源已安全释放，程序优雅退出。")

if __name__ == "__main__":
    main()
