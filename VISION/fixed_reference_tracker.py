"""Fixed body-reference tracking for the RoboFish vision application.

The physical centre of a rigid colour marker is the only precise position
measurement produced by this module.  YOLO boxes are used for identity and
search-region management; their centres are exposed only as an explicitly
degraded fallback when no marker profile has been calibrated.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


class ReferenceSource:
    RIGID_BODY = "RIGID_BODY"
    MARKER = "MARKER"
    PREDICTED = "PREDICTED"
    ESTIMATED_FALLBACK = "ESTIMATED_FALLBACK"
    INVALID = "INVALID"


@dataclass
class MarkerProfile:
    """Lighting-specific colour and contour model created from a marker ROI."""

    version: int = 1
    hue_center: float = 0.0
    hue_tolerance: float = 10.0
    sat_min: int = 70
    sat_max: int = 255
    val_min: int = 50
    val_max: int = 255
    expected_area: float = 100.0
    expected_circularity: float = 0.75
    expected_extent: float = 0.75
    expected_aspect: float = 1.0
    sample_roi_size: Tuple[int, int] = (0, 0)
    reference_role: str = "TAIL_MARKER"

    def to_dict(self) -> Dict[str, object]:
        data = asdict(self)
        data["sample_roi_size"] = list(self.sample_roi_size)
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, object]) -> "MarkerProfile":
        values = dict(data)
        values["sample_roi_size"] = tuple(values.get("sample_roi_size", (0, 0)))
        return cls(**values)


@dataclass
class MarkerTrackerConfig:
    """Conservative provisional gates; tune them with real camera images."""

    min_sample_side_px: int = 8
    min_sample_area_px: float = 25.0
    min_sample_fill_ratio: float = 0.28
    min_sample_median_saturation: float = 60.0
    min_profile_saturation: int = 45
    min_sample_circularity: float = 0.18
    min_sample_extent: float = 0.30
    max_sample_aspect: float = 3.50
    calibration_sat_min: int = 70
    calibration_val_min: int = 70
    forbidden_tail_hue_min: int = 4
    forbidden_tail_hue_max: int = 38
    forbidden_pool_hue_min: int = 86
    forbidden_pool_hue_max: int = 129
    calibration_ambiguity_ratio: float = 0.65
    # Match the runtime YOLO gate so a low-score frame can still seed the
    # local orange-marker search; the highest-confidence fish is retained.
    min_yolo_confidence: float = 0.10
    yolo_hold_s: float = 0.90
    search_expand_ratio: float = 0.30
    # The local marker ROI is intentionally a union of the previous marker
    # prediction and the YOLO box so a brief detector miss does not interrupt
    # tracking.  Once a YOLO identity is confirmed, however, an accepted
    # orange contour must still lie close to that fish box; otherwise a pool
    # corner marker/reflection can win the colour score and become a false
    # tail endpoint.  This gate is wider than ``search_expand_ratio`` because
    # the physical tail LED may sit just outside the detector box.
    yolo_gate_expand_ratio: float = 0.80
    local_search_radius_scale: float = 4.0
    min_local_search_radius_px: float = 50.0
    # The orange tail LED can occupy only 8--16 px² at the far side of the
    # pool view.  Keep the absolute contour floor below, but do not
    # reject those valid small observations solely because the calibration
    # sample was larger.
    min_area_ratio: float = 0.08
    max_area_ratio: float = 4.50
    max_circularity_delta: float = 0.60
    max_extent_delta: float = 0.60
    max_aspect_log_delta: float = 1.25
    min_color_coverage: float = 0.50
    min_quality: float = 0.38
    border_margin_px: int = 2
    base_jump_px: float = 18.0
    jump_diameter_scale: float = 1.5
    jump_speed_scale: float = 2.5
    prediction_horizon_s: float = 0.45
    reacquire_after_s: float = 0.25
    velocity_alpha: float = 0.45
    morph_kernel_px: int = 3
    body_sat_max: int = 105
    body_val_min: int = 130
    body_min_area_px: float = 35.0
    body_min_marker_area_ratio: float = 0.55
    body_min_yolo_area_ratio: float = 0.005
    body_min_aspect: float = 1.15
    body_max_overexposed_ratio: float = 0.72
    body_min_marker_distance_scale: float = 0.55
    body_max_marker_distance_scale: float = 7.0
    body_min_quality: float = 0.42
    body_morph_kernel_px: int = 5


@dataclass
class ReferenceObservation:
    timestamp: float
    source: str
    position: Optional[Tuple[float, float]]
    confidence: float
    quality: float
    marker_profiled: bool
    yolo_confirmed: bool
    usable_for_pwm_speed: bool
    gate_reasons: List[str] = field(default_factory=list)
    metrics: Dict[str, object] = field(default_factory=dict)
    search_roi: Optional[Tuple[int, int, int, int]] = None

    @property
    def valid(self) -> bool:
        return self.position is not None and self.source != ReferenceSource.INVALID

    def to_dict(self) -> Dict[str, object]:
        return {
            "source": self.source,
            "valid": self.valid,
            "u": float(self.position[0]) if self.position is not None else None,
            "v": float(self.position[1]) if self.position is not None else None,
            "confidence": round(float(self.confidence), 4),
            "quality": round(float(self.quality), 4),
            "marker_profiled": self.marker_profiled,
            "yolo_confirmed": self.yolo_confirmed,
            "usable_for_pwm_speed": self.usable_for_pwm_speed,
            "gate_reasons": list(self.gate_reasons),
            "metrics": dict(self.metrics),
            "search_roi": list(self.search_roi) if self.search_roi is not None else None,
        }


def _clip_roi(
    roi: Sequence[float], frame_width: int, frame_height: int
) -> Optional[Tuple[int, int, int, int]]:
    if roi is None or len(roi) != 4:
        return None
    x1, y1, x2, y2 = [int(round(float(value))) for value in roi]
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    x1 = max(0, min(frame_width, x1))
    y1 = max(0, min(frame_height, y1))
    x2 = max(0, min(frame_width, x2))
    y2 = max(0, min(frame_height, y2))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def _hue_distance(hue: np.ndarray, centre: float) -> np.ndarray:
    delta = np.abs(hue.astype(np.float32) - float(centre))
    return np.minimum(delta, 180.0 - delta)


def _circular_hue_mean(hues: np.ndarray) -> float:
    angles = hues.astype(np.float64) * (2.0 * math.pi / 180.0)
    sin_mean = float(np.mean(np.sin(angles)))
    cos_mean = float(np.mean(np.cos(angles)))
    if abs(sin_mean) < 1e-12 and abs(cos_mean) < 1e-12:
        return float(np.median(hues))
    angle = math.atan2(sin_mean, cos_mean)
    return float((angle * 180.0 / (2.0 * math.pi)) % 180.0)


def _contour_geometry(contour: np.ndarray) -> Dict[str, float]:
    area = float(cv2.contourArea(contour))
    perimeter = float(cv2.arcLength(contour, True))
    x, y, width, height = cv2.boundingRect(contour)
    circularity = 0.0
    if perimeter > 1e-6:
        circularity = float(4.0 * math.pi * area / (perimeter * perimeter))
    extent = float(area / max(1, width * height))
    aspect = float(max(width, height) / max(1, min(width, height)))
    moments = cv2.moments(contour)
    if abs(float(moments["m00"])) <= 1e-8:
        centre_x = float(x + width * 0.5)
        centre_y = float(y + height * 0.5)
    else:
        centre_x = float(moments["m10"] / moments["m00"])
        centre_y = float(moments["m01"] / moments["m00"])
    return {
        "area": area,
        "perimeter": perimeter,
        "circularity": circularity,
        "extent": extent,
        "aspect": aspect,
        "centre_x": centre_x,
        "centre_y": centre_y,
        "bbox_x": float(x),
        "bbox_y": float(y),
        "bbox_w": float(width),
        "bbox_h": float(height),
    }


class FixedReferenceTracker:
    """HSV marker tracker with explicit degraded and invalid states."""

    def __init__(
        self,
        profile_path: Optional[str] = None,
        config: Optional[MarkerTrackerConfig] = None,
        enable_rigid_body: bool = True,
    ):
        self.profile_path = profile_path
        self.config = config or MarkerTrackerConfig()
        self.enable_rigid_body = bool(enable_rigid_body)
        self.profile: Optional[MarkerProfile] = None
        self.profile_load_error: Optional[str] = None
        self.last_calibration: Optional[Dict[str, object]] = None
        self._last_marker_position: Optional[np.ndarray] = None
        self._last_marker_time: Optional[float] = None
        self._velocity = np.zeros(2, dtype=np.float64)
        self._last_body_position: Optional[np.ndarray] = None
        self._last_body_time: Optional[float] = None
        self._body_velocity = np.zeros(2, dtype=np.float64)
        self._expected_body_marker_distance_px: Optional[float] = None
        self._last_yolo_bbox: Optional[Tuple[int, int, int, int]] = None
        self._last_yolo_time: Optional[float] = None
        self._last_track_id: Optional[int] = None
        if profile_path and os.path.exists(profile_path):
            try:
                self.load_profile(profile_path)
            except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
                self.profile = None
                self.profile_load_error = str(error)

    @property
    def is_profiled(self) -> bool:
        return self.profile is not None

    def reset_motion(self) -> None:
        self._last_marker_position = None
        self._last_marker_time = None
        self._velocity[:] = 0.0
        self._last_body_position = None
        self._last_body_time = None
        self._body_velocity[:] = 0.0
        self._expected_body_marker_distance_px = None

    def seed_rigid_body(
        self,
        body_position: Sequence[float],
        timestamp: float,
        marker_position: Optional[Sequence[float]] = None,
    ) -> None:
        """Seed the rigid-body lock from the user's existing fish-head click."""
        position = np.asarray(body_position, dtype=np.float64).reshape(2)
        if not np.isfinite(position).all() or not math.isfinite(float(timestamp)):
            raise ValueError("白色鱼身初始点无效")
        self._last_body_position = position.copy()
        self._last_body_time = float(timestamp)
        self._body_velocity[:] = 0.0
        self._expected_body_marker_distance_px = None
        if marker_position is not None:
            marker = np.asarray(marker_position, dtype=np.float64).reshape(2)
            distance = float(np.linalg.norm(position - marker))
            if np.isfinite(marker).all() and distance >= 8.0:
                self._expected_body_marker_distance_px = distance

    def clear_profile(self, delete_saved: bool = False) -> None:
        self.profile = None
        self.profile_load_error = None
        self.last_calibration = None
        self.reset_motion()
        if delete_saved and self.profile_path and os.path.exists(self.profile_path):
            os.remove(self.profile_path)

    def save_profile(self, path: Optional[str] = None) -> None:
        if self.profile is None:
            raise ValueError("没有可保存的色标模型")
        target = path or self.profile_path
        if not target:
            raise ValueError("未设置色标模型保存路径")
        os.makedirs(os.path.dirname(os.path.abspath(target)), exist_ok=True)
        with open(target, "w", encoding="utf-8") as profile_file:
            json.dump(self.profile.to_dict(), profile_file, ensure_ascii=False, indent=2)

    def load_profile(self, path: Optional[str] = None) -> MarkerProfile:
        target = path or self.profile_path
        if not target:
            raise ValueError("未设置色标模型路径")
        with open(target, "r", encoding="utf-8") as profile_file:
            data = json.load(profile_file)
        profile = MarkerProfile.from_dict(data)
        # Upper saturation/value limits change with exposure and gain.  Keep
        # the calibrated lower bounds but allow bright marker pixels through.
        profile.sat_max = 255
        profile.val_max = 255
        # Widen the persisted orange profile for the current camera's exposure
        # variation.  The marker is still constrained by its local ROI and
        # contour/temporal gates, so this does not enable full-frame orange
        # matching.
        profile.hue_tolerance = float(np.clip(
            max(float(profile.hue_tolerance), 20.0), 4.0, 35.0
        ))
        profile.sat_min = int(min(int(profile.sat_min), 45))
        profile.val_min = int(min(int(profile.val_min), 40))
        if profile.version != 1 or profile.expected_area <= 0:
            raise ValueError("色标模型版本或面积无效")
        validation_errors = self.profile_validation_errors(profile)
        if validation_errors:
            raise ValueError("旧色标模型质量不合格：" + "、".join(validation_errors))
        self.profile = profile
        self.profile_load_error = None
        self.reset_motion()
        return profile

    def profile_validation_errors(self, profile: MarkerProfile) -> List[str]:
        width, height = profile.sample_roi_size
        roi_area = max(1.0, float(width * height))
        fill_ratio = float(profile.expected_area / roi_area)
        errors: List[str] = []
        if profile.sat_min < self.config.min_profile_saturation:
            errors.append("饱和度下限过低，可能把白色反光当色标")
        if self.config.forbidden_pool_hue_min <= profile.hue_center <= self.config.forbidden_pool_hue_max:
            errors.append("颜色与蓝色水池背景重合")
        if fill_ratio < self.config.min_sample_fill_ratio:
            errors.append("有效色标占 ROI 比例过小，请贴边框选")
        if (
            profile.expected_circularity < self.config.min_sample_circularity
            and profile.expected_extent < self.config.min_sample_extent
        ):
            errors.append("色标轮廓过于破碎或包含鱼身背景")
        if profile.expected_aspect > self.config.max_sample_aspect:
            errors.append("色标轮廓过长，请只框圆形或方形标记")
        return errors

    def _raw_colour_mask(self, hsv: np.ndarray, profile: MarkerProfile) -> np.ndarray:
        hue_ok = _hue_distance(hsv[:, :, 0], profile.hue_center) <= profile.hue_tolerance
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]
        mask = (
            hue_ok
            & (sat >= profile.sat_min)
            & (sat <= profile.sat_max)
            & (val >= profile.val_min)
            & (val <= profile.val_max)
        )
        return (mask.astype(np.uint8) * 255)

    def calibrate_from_roi(
        self,
        frame: np.ndarray,
        roi: Sequence[float],
        save: bool = True,
    ) -> MarkerProfile:
        """Find the orange tail block inside a rough ROI and model its centre."""
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("需要有效的 BGR 彩色画面")
        clipped = _clip_roi(roi, frame.shape[1], frame.shape[0])
        if clipped is None:
            raise ValueError("色标 ROI 无效")
        x1, y1, x2, y2 = clipped
        width, height = x2 - x1, y2 - y1
        if width < self.config.min_sample_side_px or height < self.config.min_sample_side_px:
            raise ValueError("色标 ROI 过小")

        sample = frame[y1:y2, x1:x2]
        hsv = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]
        saturated = (
            (sat >= self.config.calibration_sat_min)
            & (val >= self.config.calibration_val_min)
        )
        tail_orange = (
            (hue >= self.config.forbidden_tail_hue_min)
            & (hue <= self.config.forbidden_tail_hue_max)
            & saturated
        )
        raw_mask = tail_orange.astype(np.uint8) * 255
        kernel_size = max(1, int(self.config.morph_kernel_px) | 1)
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        clean_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, kernel)
        clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [
            contour for contour in contours
            if cv2.contourArea(contour) >= self.config.min_sample_area_px
        ]
        if not contours:
            raise ValueError("框选范围内没有找到高饱和橙色鱼尾")

        roi_centre = np.asarray([width * 0.5, height * 0.5], dtype=np.float64)
        roi_diagonal = max(1.0, float(np.hypot(width, height)))
        ranked = []
        for candidate in contours:
            candidate_geometry = _contour_geometry(candidate)
            candidate_centre = np.asarray([
                candidate_geometry["centre_x"], candidate_geometry["centre_y"]
            ])
            centre_weight = 1.5 - min(
                1.0, float(np.linalg.norm(candidate_centre - roi_centre)) / roi_diagonal
            )
            ranked.append((candidate_geometry["area"] * centre_weight, candidate))
        ranked.sort(key=lambda item: item[0], reverse=True)
        if (
            len(ranked) > 1
            and ranked[1][0] >= ranked[0][0] * self.config.calibration_ambiguity_ratio
        ):
            raise ValueError("框选范围内有多个相近橙色块，请缩小范围只包含鱼尾")
        contour = ranked[0][1]
        geometry = _contour_geometry(contour)
        marker_box_width = int(geometry["bbox_w"])
        marker_box_height = int(geometry["bbox_h"])
        border_margin = self.config.border_margin_px
        if (
            geometry["bbox_x"] <= border_margin
            or geometry["bbox_y"] <= border_margin
            or geometry["bbox_x"] + geometry["bbox_w"] >= width - border_margin
            or geometry["bbox_y"] + geometry["bbox_h"] >= height - border_margin
        ):
            raise ValueError("橙色鱼尾贴到框选边缘，请把标定范围稍微扩大")

        contour_fill = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(contour_fill, [contour], -1, 255, -1)
        marker_pixels = hsv[(contour_fill > 0) & (raw_mask > 0)]
        if len(marker_pixels) < 8:
            raise ValueError("橙色鱼尾有效颜色像素过少")
        hue_centre = _circular_hue_mean(marker_pixels[:, 0])
        hue_dist = _hue_distance(marker_pixels[:, 0], hue_centre)
        fill_ratio = geometry["area"] / max(
            1.0, float(marker_box_width * marker_box_height)
        )
        median_saturation = float(np.median(marker_pixels[:, 1]))
        sample_errors: List[str] = []
        if fill_ratio < self.config.min_sample_fill_ratio:
            sample_errors.append(
                f"鱼尾橙色轮廓内部有效占比仅 {fill_ratio * 100:.0f}%"
            )
        if median_saturation < self.config.min_sample_median_saturation:
            sample_errors.append("颜色饱和度太低，请避开白色鱼身和灯光反射")
        if (
            geometry["circularity"] < self.config.min_sample_circularity
            and geometry["extent"] < self.config.min_sample_extent
        ):
            sample_errors.append("轮廓过于破碎，请只框完整的圆形或方形色标")
        if geometry["aspect"] > self.config.max_sample_aspect:
            sample_errors.append("轮廓过长，请不要把鱼身一起框入")
        if sample_errors:
            raise ValueError("；".join(sample_errors))

        profile = MarkerProfile(
            hue_center=hue_centre,
            hue_tolerance=float(np.clip(np.percentile(hue_dist, 98) + 4.0, 4.0, 30.0)),
            sat_min=int(max(
                self.config.min_profile_saturation,
                np.clip(np.percentile(marker_pixels[:, 1], 2) - 20, 0, 255),
            )),
            # Saturation/brightness upper bounds are not stable across the
            # camera's exposure compensation; use only lower bounds for the
            # persistent orange-light profile.
            sat_max=255,
            val_min=int(np.clip(np.percentile(marker_pixels[:, 2], 2) - 20, 0, 255)),
            val_max=255,
            expected_area=geometry["area"],
            expected_circularity=geometry["circularity"],
            expected_extent=geometry["extent"],
            expected_aspect=geometry["aspect"],
            sample_roi_size=(marker_box_width, marker_box_height),
            reference_role="TAIL_MARKER",
        )
        self.profile = profile
        self.last_calibration = {
            "selection_roi": clipped,
            "marker_bbox": (
                int(round(x1 + geometry["bbox_x"])),
                int(round(y1 + geometry["bbox_y"])),
                int(round(x1 + geometry["bbox_x"] + geometry["bbox_w"])),
                int(round(y1 + geometry["bbox_y"] + geometry["bbox_h"])),
            ),
            "marker_center": (
                float(x1 + geometry["centre_x"]),
                float(y1 + geometry["centre_y"]),
            ),
        }
        self.reset_motion()
        if save and self.profile_path:
            self.save_profile()
        return profile

    def _update_yolo(
        self,
        frame_shape: Tuple[int, ...],
        timestamp: float,
        yolo_bbox: Optional[Sequence[float]],
        yolo_confidence: float,
        track_id: Optional[int],
        yolo_timestamp: Optional[float],
    ) -> Tuple[bool, bool]:
        identity_changed = False
        accepted = (
            yolo_bbox is not None
            and yolo_confidence >= self.config.min_yolo_confidence
        )
        if accepted:
            clipped = _clip_roi(yolo_bbox, frame_shape[1], frame_shape[0])
            accepted = clipped is not None
            if accepted:
                measurement_time = float(timestamp if yolo_timestamp is None else yolo_timestamp)
                if (
                    track_id is not None
                    and self._last_track_id is not None
                    and track_id != self._last_track_id
                ):
                    identity_changed = True
                    self.reset_motion()
                self._last_yolo_bbox = clipped
                self._last_yolo_time = measurement_time
                if track_id is not None:
                    self._last_track_id = track_id

        yolo_confirmed = (
            self._last_yolo_bbox is not None
            and self._last_yolo_time is not None
            and 0.0 <= timestamp - self._last_yolo_time <= self.config.yolo_hold_s
        )
        return yolo_confirmed, identity_changed

    def _build_search_roi(
        self,
        frame_shape: Tuple[int, ...],
        yolo_confirmed: bool,
        timestamp: float,
    ) -> Optional[Tuple[int, int, int, int]]:
        """Build the union ROI used for local tracking and reacquisition.

        YOLO is deliberately not the only source of pixels in this window;
        the candidate-level gate below still requires an accepted contour to
        be near the confirmed fish box, which prevents an unrelated corner
        marker from winning the union search.
        """
        frame_height, frame_width = frame_shape[:2]
        regions = []

        # A previously found marker owns the high-rate local search.  Its
        # predicted position remains usable even while YOLO is temporarily
        # missing or between low-frequency identity checks.
        if self._last_marker_position is not None and self.profile is not None:
            marker_radius = math.sqrt(max(1.0, self.profile.expected_area) / math.pi)
            radius = max(
                self.config.min_local_search_radius_px,
                marker_radius * self.config.local_search_radius_scale,
            )
            elapsed = (
                0.0
                if self._last_marker_time is None
                else max(0.0, float(timestamp) - self._last_marker_time)
            )
            predicted = self._last_marker_position + self._velocity * min(
                elapsed, self.config.prediction_horizon_s
            )
            px, py = predicted
            regions.append([px - radius, py - radius, px + radius, py + radius])

        # YOLO supplies the initial/wide re-acquisition area only.
        if yolo_confirmed and self._last_yolo_bbox is not None:
            x1, y1, x2, y2 = self._last_yolo_bbox
            margin_x = max(4.0, (x2 - x1) * self.config.search_expand_ratio)
            margin_y = max(4.0, (y2 - y1) * self.config.search_expand_ratio)
            regions.append([x1 - margin_x, y1 - margin_y, x2 + margin_x, y2 + margin_y])

        if not regions:
            return None

        union = [
            min(region[0] for region in regions),
            min(region[1] for region in regions),
            max(region[2] for region in regions),
            max(region[3] for region in regions),
        ]
        return _clip_roi(union, frame_width, frame_height)

    def _candidate_metrics(
        self,
        contour: np.ndarray,
        raw_mask: np.ndarray,
        search_roi: Tuple[int, int, int, int],
        timestamp: float,
        yolo_confirmed: bool,
    ) -> Tuple[Dict[str, object], List[str]]:
        assert self.profile is not None
        profile = self.profile
        config = self.config
        geometry = _contour_geometry(contour)
        area_ratio = geometry["area"] / max(1.0, profile.expected_area)

        contour_fill = np.zeros_like(raw_mask)
        cv2.drawContours(contour_fill, [contour], -1, 255, -1)
        contour_pixels = int(np.count_nonzero(contour_fill))
        colour_pixels = int(np.count_nonzero((raw_mask > 0) & (contour_fill > 0)))
        colour_coverage = float(colour_pixels / max(1, contour_pixels))

        area_score = float(math.exp(-abs(math.log(max(area_ratio, 1e-6))) / 0.70))
        circularity_delta = abs(geometry["circularity"] - profile.expected_circularity)
        extent_delta = abs(geometry["extent"] - profile.expected_extent)
        aspect_log_delta = abs(math.log(max(geometry["aspect"], 1e-6) / max(profile.expected_aspect, 1e-6)))
        shape_score = float(np.clip(
            1.0
            - 0.40 * circularity_delta / max(config.max_circularity_delta, 1e-6)
            - 0.30 * extent_delta / max(config.max_extent_delta, 1e-6)
            - 0.30 * aspect_log_delta / max(config.max_aspect_log_delta, 1e-6),
            0.0,
            1.0,
        ))

        local_x = geometry["centre_x"]
        local_y = geometry["centre_y"]
        global_position = np.asarray(
            [local_x + search_roi[0], local_y + search_roi[1]], dtype=np.float64
        )
        jump_px = 0.0
        jump_limit_px = float("inf")
        reacquire = False
        jump_score = 1.0
        if self._last_marker_position is not None and self._last_marker_time is not None:
            elapsed = max(0.0, timestamp - self._last_marker_time)
            predicted = self._last_marker_position + self._velocity * elapsed
            jump_px = float(np.linalg.norm(global_position - predicted))
            diameter = 2.0 * math.sqrt(max(1.0, profile.expected_area) / math.pi)
            jump_limit_px = (
                config.base_jump_px
                + config.jump_diameter_scale * diameter
                + config.jump_speed_scale * float(np.linalg.norm(self._velocity)) * elapsed
            )
            reacquire = elapsed >= config.reacquire_after_s and yolo_confirmed
            if not reacquire:
                jump_score = float(np.clip(1.0 - jump_px / max(jump_limit_px, 1e-6), 0.0, 1.0))

        # ``_build_search_roi`` is a union by design: it has to include the
        # previous marker prediction while also allowing YOLO-assisted
        # reacquisition.  Treating every contour in that union as a valid
        # tail would let a static orange pool corner replace the fish marker.
        # Keep the gate wide enough for a marker mounted just beyond a tight
        # YOLO box, but reject contours that are clearly from another object.
        yolo_region_gate = False
        if yolo_confirmed and self._last_yolo_bbox is not None:
            yolo_x1, yolo_y1, yolo_x2, yolo_y2 = self._last_yolo_bbox
            expand = max(0.0, float(config.yolo_gate_expand_ratio))
            gate_margin_x = max(8.0, (yolo_x2 - yolo_x1) * expand)
            gate_margin_y = max(8.0, (yolo_y2 - yolo_y1) * expand)
            yolo_region_gate = not (
                yolo_x1 - gate_margin_x <= global_position[0] <= yolo_x2 + gate_margin_x
                and yolo_y1 - gate_margin_y <= global_position[1] <= yolo_y2 + gate_margin_y
            )

        bx = int(geometry["bbox_x"])
        by = int(geometry["bbox_y"])
        bw = int(geometry["bbox_w"])
        bh = int(geometry["bbox_h"])
        search_width = search_roi[2] - search_roi[0]
        search_height = search_roi[3] - search_roi[1]
        margin = config.border_margin_px
        boundary_cropped = (
            bx <= margin
            or by <= margin
            or bx + bw >= search_width - margin
            or by + bh >= search_height - margin
        )

        quality = float(np.clip(
            0.30 * area_score
            + 0.30 * shape_score
            + 0.25 * colour_coverage
            + 0.15 * jump_score,
            0.0,
            1.0,
        ))
        reasons: List[str] = []
        if not config.min_area_ratio <= area_ratio <= config.max_area_ratio:
            reasons.append("AREA_GATE")
        if circularity_delta > config.max_circularity_delta:
            reasons.append("CIRCULARITY_GATE")
        if extent_delta > config.max_extent_delta:
            reasons.append("EXTENT_GATE")
        if aspect_log_delta > config.max_aspect_log_delta:
            reasons.append("ASPECT_GATE")
        if colour_coverage < config.min_color_coverage:
            reasons.append("COLOR_GATE")
        if boundary_cropped:
            reasons.append("BOUNDARY_CROP_GATE")
        if yolo_region_gate:
            reasons.append("YOLO_REGION_GATE")
        if jump_px > jump_limit_px and not reacquire:
            reasons.append("POSITION_JUMP_GATE")
        if quality < config.min_quality:
            reasons.append("QUALITY_GATE")

        metrics: Dict[str, object] = {
            "position": (float(global_position[0]), float(global_position[1])),
            # Global contour box, retained for endpoint diagnostics.  This is
            # the actual orange component accepted by the marker gates, not
            # the broad search ROI or the YOLO fish box.
            "bbox": (
                int(search_roi[0] + bx),
                int(search_roi[1] + by),
                int(search_roi[0] + bx + bw),
                int(search_roi[1] + by + bh),
            ),
            "area": round(geometry["area"], 3),
            "area_ratio": round(area_ratio, 4),
            "circularity": round(geometry["circularity"], 4),
            "extent": round(geometry["extent"], 4),
            "aspect": round(geometry["aspect"], 4),
            "color_coverage": round(colour_coverage, 4),
            "jump_px": round(jump_px, 3),
            "jump_limit_px": None if math.isinf(jump_limit_px) else round(jump_limit_px, 3),
            "reacquire_gate": reacquire,
            "quality": round(quality, 4),
        }
        return metrics, reasons

    def _detect_marker(
        self,
        frame: np.ndarray,
        timestamp: float,
        search_roi: Tuple[int, int, int, int],
        yolo_confirmed: bool,
    ) -> Tuple[Optional[Dict[str, object]], List[str], Dict[str, object]]:
        assert self.profile is not None
        x1, y1, x2, y2 = search_roi
        search = frame[y1:y2, x1:x2]
        hsv = cv2.cvtColor(search, cv2.COLOR_BGR2HSV)
        raw_mask = self._raw_colour_mask(hsv, self.profile)
        kernel_size = max(1, int(self.config.morph_kernel_px) | 1)
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        accepted: List[Dict[str, object]] = []
        rejected_reasons: List[str] = []
        best_rejected: Dict[str, object] = {}
        for contour in contours:
            if cv2.contourArea(contour) < max(4.0, self.profile.expected_area * 0.10):
                continue
            metrics, reasons = self._candidate_metrics(
                contour, raw_mask, search_roi, timestamp, yolo_confirmed
            )
            if not reasons:
                accepted.append(metrics)
            else:
                rejected_reasons.extend(reasons)
                if not best_rejected or metrics["quality"] > best_rejected.get("quality", -1.0):
                    best_rejected = metrics

        if not accepted:
            reasons = sorted(set(rejected_reasons)) if rejected_reasons else ["NO_MARKER_CONTOUR"]
            return None, reasons, best_rejected
        best = max(accepted, key=lambda item: float(item["quality"]))
        return best, [], {}

    def _detect_rigid_body(
        self,
        frame: np.ndarray,
        marker_position: Sequence[float],
        timestamp: float,
    ) -> Tuple[Optional[Dict[str, object]], List[str]]:
        """Lock the bright rigid front body inside the confirmed YOLO box.

        The orange tail remains the identity/search anchor, but its oscillating
        centre is not used as the control position when a valid rigid-body
        contour is available.
        """
        if self._last_yolo_bbox is None or self.profile is None:
            return None, ["BODY_NO_YOLO_ROI"]
        x1, y1, x2, y2 = self._last_yolo_bbox
        search = frame[y1:y2, x1:x2]
        if search.size == 0:
            return None, ["BODY_EMPTY_ROI"]

        hsv = cv2.cvtColor(search, cv2.COLOR_BGR2HSV)
        raw_mask = (
            (hsv[:, :, 1] <= self.config.body_sat_max)
            & (hsv[:, :, 2] >= self.config.body_val_min)
        ).astype(np.uint8) * 255
        kernel_size = max(1, int(self.config.body_morph_kernel_px) | 1)
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        marker = np.asarray(marker_position, dtype=np.float64)
        marker_diameter = 2.0 * math.sqrt(
            max(1.0, self.profile.expected_area) / math.pi
        )
        min_distance = (
            self.config.body_min_marker_distance_scale * marker_diameter
        )
        max_distance = (
            self.config.body_max_marker_distance_scale * marker_diameter
        )
        if self._expected_body_marker_distance_px is not None:
            min_distance = max(
                min_distance, self._expected_body_marker_distance_px * 0.50
            )
            max_distance = min(
                max_distance, self._expected_body_marker_distance_px * 1.65
            )
        roi_area = float(max(1, (x2 - x1) * (y2 - y1)))
        minimum_body_area = max(
            self.config.body_min_area_px,
            self.profile.expected_area * self.config.body_min_marker_area_ratio,
            roi_area * self.config.body_min_yolo_area_ratio,
        )
        candidates: List[Dict[str, object]] = []
        rejected: List[str] = []
        for contour in contours:
            geometry = _contour_geometry(contour)
            if geometry["area"] < minimum_body_area:
                rejected.append("BODY_TOO_SMALL_GATE")
                continue
            position = np.asarray([
                x1 + geometry["centre_x"], y1 + geometry["centre_y"]
            ], dtype=np.float64)
            marker_distance = float(np.linalg.norm(position - marker))
            if marker_distance < min_distance or marker_distance > max_distance:
                rejected.append("BODY_MARKER_DISTANCE_GATE")
                continue
            area_ratio = geometry["area"] / roi_area
            if area_ratio > 0.70:
                rejected.append("BODY_AREA_GATE")
                continue

            temporal_score = 1.0
            jump_px = 0.0
            trusted_body_neighbourhood = False
            if self._last_body_position is not None and self._last_body_time is not None:
                elapsed = max(0.0, timestamp - self._last_body_time)
                predicted = self._last_body_position + self._body_velocity * elapsed
                jump_px = float(np.linalg.norm(position - predicted))
                jump_limit = max(20.0, marker_diameter * 1.5)
                trusted_body_neighbourhood = jump_px <= jump_limit
                temporal_score = float(np.clip(
                    1.0 - jump_px / max(jump_limit, 1e-6), 0.0, 1.0
                ))
                if not trusted_body_neighbourhood:
                    rejected.append("BODY_POSITION_JUMP_GATE")
                    continue

            if (
                geometry["aspect"] < self.config.body_min_aspect
                and not trusted_body_neighbourhood
            ):
                rejected.append("BODY_GLARE_SHAPE_GATE")
                continue

            contour_fill = np.zeros_like(raw_mask)
            cv2.drawContours(contour_fill, [contour], -1, 255, -1)
            white_coverage = float(np.count_nonzero(
                (raw_mask > 0) & (contour_fill > 0)
            ) / max(1, np.count_nonzero(contour_fill)))
            contour_pixels = contour_fill > 0
            overexposed_ratio = float(np.count_nonzero(
                (hsv[:, :, 2] >= 250) & contour_pixels
            ) / max(1, np.count_nonzero(contour_pixels)))
            if (
                overexposed_ratio > self.config.body_max_overexposed_ratio
                and not trusted_body_neighbourhood
            ):
                rejected.append("BODY_GLARE_EXPOSURE_GATE")
                continue

            # Prefer the largest stable bright component; the distance term
            # rejects white pool glare close to the orange tail.
            size_score = float(np.clip(area_ratio / 0.18, 0.0, 1.0))
            quality = float(np.clip(
                0.25 * size_score
                + 0.25 * white_coverage
                + 0.50 * temporal_score,
                0.0,
                1.0,
            ))
            if quality < self.config.body_min_quality:
                rejected.append("BODY_QUALITY_GATE")
                continue
            candidates.append({
                "position": (float(position[0]), float(position[1])),
                "quality": round(quality, 4),
                "area": round(geometry["area"], 3),
                "minimum_area": round(minimum_body_area, 3),
                "area_ratio": round(area_ratio, 4),
                "white_coverage": round(white_coverage, 4),
                "overexposed_ratio": round(overexposed_ratio, 4),
                "aspect": round(geometry["aspect"], 4),
                "marker_distance_px": round(marker_distance, 3),
                "jump_px": round(jump_px, 3),
                "seeded_neighbourhood": trusted_body_neighbourhood,
                "bbox": (
                    int(round(x1 + geometry["bbox_x"])),
                    int(round(y1 + geometry["bbox_y"])),
                    int(round(x1 + geometry["bbox_x"] + geometry["bbox_w"])),
                    int(round(y1 + geometry["bbox_y"] + geometry["bbox_h"])),
                ),
            })

        if not candidates:
            return None, sorted(set(rejected)) or ["NO_RIGID_BODY_CONTOUR"]
        return max(candidates, key=lambda item: float(item["quality"])), []

    def _update_body_motion(self, position: np.ndarray, timestamp: float) -> None:
        if self._last_body_position is not None and self._last_body_time is not None:
            dt = timestamp - self._last_body_time
            if 1e-4 < dt <= 1.0:
                raw_velocity = (position - self._last_body_position) / dt
                alpha = self.config.velocity_alpha
                self._body_velocity = (
                    alpha * raw_velocity + (1.0 - alpha) * self._body_velocity
                )
            elif dt > 1.0:
                self._body_velocity[:] = 0.0
        self._last_body_position = position
        self._last_body_time = timestamp

    def update(
        self,
        frame: np.ndarray,
        timestamp: float,
        yolo_bbox: Optional[Sequence[float]] = None,
        yolo_confidence: float = 0.0,
        track_id: Optional[int] = None,
        yolo_timestamp: Optional[float] = None,
    ) -> ReferenceObservation:
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            return ReferenceObservation(
                timestamp=float(timestamp),
                source=ReferenceSource.INVALID,
                position=None,
                confidence=0.0,
                quality=0.0,
                marker_profiled=self.is_profiled,
                yolo_confirmed=False,
                usable_for_pwm_speed=False,
                gate_reasons=["INVALID_FRAME"],
            )

        timestamp = float(timestamp)
        yolo_confirmed, identity_changed = self._update_yolo(
            frame.shape,
            timestamp,
            yolo_bbox,
            float(yolo_confidence),
            track_id,
            yolo_timestamp,
        )

        if self.profile is None:
            if yolo_confirmed and self._last_yolo_bbox is not None:
                x1, y1, x2, y2 = self._last_yolo_bbox
                position = ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
                return ReferenceObservation(
                    timestamp=timestamp,
                    source=ReferenceSource.ESTIMATED_FALLBACK,
                    position=position,
                    confidence=float(np.clip(yolo_confidence, 0.0, 1.0)),
                    quality=float(np.clip(yolo_confidence, 0.0, 1.0)),
                    marker_profiled=False,
                    yolo_confirmed=True,
                    usable_for_pwm_speed=False,
                    gate_reasons=["MARKER_NOT_CALIBRATED"],
                    metrics={"identity_changed": identity_changed},
                    search_roi=self._last_yolo_bbox,
                )
            return ReferenceObservation(
                timestamp=timestamp,
                source=ReferenceSource.INVALID,
                position=None,
                confidence=0.0,
                quality=0.0,
                marker_profiled=False,
                yolo_confirmed=False,
                usable_for_pwm_speed=False,
                gate_reasons=["MARKER_NOT_CALIBRATED", "YOLO_NOT_CONFIRMED"],
            )

        search_roi = self._build_search_roi(
            frame.shape, yolo_confirmed, timestamp
        )
        marker: Optional[Dict[str, object]] = None
        reasons: List[str] = []
        rejected_metrics: Dict[str, object] = {}
        if search_roi is None:
            reasons = ["YOLO_NOT_CONFIRMED"]
        else:
            marker, reasons, rejected_metrics = self._detect_marker(
                frame, timestamp, search_roi, yolo_confirmed
            )

        if marker is not None:
            position = np.asarray(marker["position"], dtype=np.float64)
            if self._last_marker_position is not None and self._last_marker_time is not None:
                dt = timestamp - self._last_marker_time
                if 1e-4 < dt <= 1.0:
                    raw_velocity = (position - self._last_marker_position) / dt
                    alpha = self.config.velocity_alpha
                    self._velocity = alpha * raw_velocity + (1.0 - alpha) * self._velocity
                elif dt > 1.0:
                    self._velocity[:] = 0.0
            self._last_marker_position = position
            self._last_marker_time = timestamp
            quality = float(marker["quality"])
            if self.enable_rigid_body and yolo_confirmed:
                body, body_reasons = self._detect_rigid_body(
                    frame, position, timestamp
                )
            elif self.enable_rigid_body:
                body, body_reasons = None, ["BODY_YOLO_NOT_CONFIRMED"]
            else:
                body, body_reasons = None, []
            if body is not None:
                body_position = np.asarray(body["position"], dtype=np.float64)
                self._update_body_motion(body_position, timestamp)
                body_quality = float(body["quality"])
                combined_quality = min(quality, body_quality)
                return ReferenceObservation(
                    timestamp=timestamp,
                    source=ReferenceSource.RIGID_BODY,
                    position=(float(body_position[0]), float(body_position[1])),
                    confidence=combined_quality,
                    quality=combined_quality,
                    marker_profiled=True,
                    yolo_confirmed=yolo_confirmed,
                    usable_for_pwm_speed=True,
                    gate_reasons=[],
                    metrics={
                        **body,
                        "reference_role": "RIGID_WHITE_BODY",
                        "tail_marker_position": (
                            float(position[0]), float(position[1])
                        ),
                        "tail_marker_bbox": marker.get("bbox"),
                        "tail_marker_area": marker.get("area"),
                        "tail_marker_quality": round(quality, 4),
                    },
                    search_roi=search_roi,
                )

            if (
                self.enable_rigid_body
                and self._last_body_position is not None
                and self._last_body_time is not None
            ):
                elapsed = timestamp - self._last_body_time
                if 0.0 <= elapsed <= self.config.prediction_horizon_s:
                    predicted = self._last_body_position + self._body_velocity * elapsed
                    decay = max(
                        0.0, 1.0 - elapsed / self.config.prediction_horizon_s
                    )
                    return ReferenceObservation(
                        timestamp=timestamp,
                        source=ReferenceSource.PREDICTED,
                        position=(float(predicted[0]), float(predicted[1])),
                        confidence=0.45 * decay,
                        quality=0.45 * decay,
                        marker_profiled=True,
                        yolo_confirmed=yolo_confirmed,
                        usable_for_pwm_speed=False,
                        gate_reasons=body_reasons,
                        metrics={
                            "reference_role": "RIGID_WHITE_BODY",
                            "prediction_age_s": round(elapsed, 4),
                            "tail_marker_position": (
                                float(position[0]), float(position[1])
                            ),
                        },
                        search_roi=search_roi,
                    )
            return ReferenceObservation(
                timestamp=timestamp,
                source=ReferenceSource.MARKER,
                position=(float(position[0]), float(position[1])),
                confidence=quality,
                quality=quality,
                marker_profiled=True,
                yolo_confirmed=yolo_confirmed,
                usable_for_pwm_speed=self.profile.reference_role == "RIGID_BODY_MARKER",
                gate_reasons=body_reasons,
                metrics={
                    **marker,
                    "reference_role": self.profile.reference_role,
                    "tail_marker_area": marker.get("area"),
                    "tail_marker_quality": round(quality, 4),
                    "tail_marker_position": (
                        float(position[0]), float(position[1])
                    ),
                },
                search_roi=search_roi,
            )

        if (
            self.enable_rigid_body
            and self._last_body_position is not None
            and self._last_body_time is not None
        ):
            elapsed = timestamp - self._last_body_time
            if 0.0 <= elapsed <= self.config.prediction_horizon_s:
                predicted = self._last_body_position + self._body_velocity * elapsed
                decay = max(0.0, 1.0 - elapsed / self.config.prediction_horizon_s)
                return ReferenceObservation(
                    timestamp=timestamp,
                    source=ReferenceSource.PREDICTED,
                    position=(float(predicted[0]), float(predicted[1])),
                    confidence=0.45 * decay,
                    quality=0.45 * decay,
                    marker_profiled=True,
                    yolo_confirmed=yolo_confirmed,
                    usable_for_pwm_speed=False,
                    gate_reasons=reasons,
                    metrics={
                        "reference_role": "RIGID_WHITE_BODY",
                        "prediction_age_s": round(elapsed, 4),
                        "rejected_candidate": rejected_metrics,
                    },
                    search_roi=search_roi,
                )

        if self._last_marker_position is not None and self._last_marker_time is not None:
            elapsed = timestamp - self._last_marker_time
            if 0.0 <= elapsed <= self.config.prediction_horizon_s:
                predicted = self._last_marker_position + self._velocity * elapsed
                decay = max(0.0, 1.0 - elapsed / self.config.prediction_horizon_s)
                return ReferenceObservation(
                    timestamp=timestamp,
                    source=ReferenceSource.PREDICTED,
                    position=(float(predicted[0]), float(predicted[1])),
                    confidence=0.45 * decay,
                    quality=0.45 * decay,
                    marker_profiled=True,
                    yolo_confirmed=yolo_confirmed,
                    usable_for_pwm_speed=False,
                    gate_reasons=reasons,
                    metrics={
                        "prediction_age_s": round(elapsed, 4),
                        "rejected_candidate": rejected_metrics,
                    },
                    search_roi=search_roi,
                )

        return ReferenceObservation(
            timestamp=timestamp,
            source=ReferenceSource.INVALID,
            position=None,
            confidence=0.0,
            quality=0.0,
            marker_profiled=True,
            yolo_confirmed=yolo_confirmed,
            usable_for_pwm_speed=False,
            gate_reasons=reasons or ["MARKER_INVALID"],
            metrics={"rejected_candidate": rejected_metrics},
            search_roi=search_roi,
        )
