"""Local colour tracking helpers for illuminated fish endpoints.

The orange tail marker has a dedicated position tracker.  The white head
tracker in this module is independent from that tracker and from YOLO pose
inference.  When YOLO supplies a fish box, the tracker searches only inside
that box, selects its largest valid white component, and validates the YOLO
head keypoint against the selected component.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import os
from typing import Dict, Optional, Sequence, Tuple

import cv2
import numpy as np


def _hue_distance(hue: np.ndarray, centre: float) -> np.ndarray:
    distance = np.abs(hue.astype(np.float32) - float(centre))
    return np.minimum(distance, 180.0 - distance)


def _circular_hue_mean(hues: np.ndarray) -> float:
    if hues.size == 0:
        return 0.0
    radians = hues.astype(np.float64) * (2.0 * math.pi / 180.0)
    angle = math.atan2(float(np.sin(radians).mean()), float(np.cos(radians).mean()))
    return (math.degrees(angle) / 2.0) % 180.0


@dataclass
class ColorPointProfile:
    version: int = 1
    role: str = "HEAD_MARKER"
    hue_center: float = 150.0
    hue_tolerance: float = 18.0
    sat_min: int = 70
    sat_max: int = 255
    val_min: int = 45
    val_max: int = 255
    expected_area: float = 20.0

    @classmethod
    def from_dict(cls, data: Dict[str, object]) -> "ColorPointProfile":
        values = dict(data)
        return cls(**values)


class ColorPointTracker:
    """Track one coloured endpoint in a local ROI."""

    def __init__(
        self,
        profile_path: Optional[str] = None,
        *,
        search_radius_px: float = 70.0,
        prediction_horizon_s: float = 0.30,
        velocity_alpha: float = 0.55,
        morph_kernel_px: int = 3,
    ):
        self.profile_path = profile_path
        self.search_radius_px = max(12.0, float(search_radius_px))
        self.prediction_horizon_s = max(0.05, float(prediction_horizon_s))
        self.velocity_alpha = float(np.clip(velocity_alpha, 0.0, 1.0))
        self.morph_kernel_px = max(1, int(morph_kernel_px) | 1)
        self.profile: Optional[ColorPointProfile] = None
        self.profile_load_error: Optional[str] = None
        self._position: Optional[np.ndarray] = None
        self._velocity = np.zeros(2, dtype=np.float64)
        self._last_time: Optional[float] = None
        if profile_path and os.path.isfile(profile_path):
            try:
                self.load_profile(profile_path)
            except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
                self.profile_load_error = str(error)

    @property
    def is_profiled(self) -> bool:
        return self.profile is not None

    def reset_motion(self) -> None:
        self._position = None
        self._velocity[:] = 0.0
        self._last_time = None

    def seed(self, position: Sequence[float], timestamp: Optional[float] = None) -> None:
        point = np.asarray(position, dtype=np.float64).reshape(2)
        if not np.isfinite(point).all():
            raise ValueError("head marker seed is not finite")
        self._position = point.copy()
        self._velocity[:] = 0.0
        self._last_time = None if timestamp is None else float(timestamp)

    def load_profile(self, path: Optional[str] = None) -> ColorPointProfile:
        target = path or self.profile_path
        if not target:
            raise ValueError("head colour profile path is not configured")
        with open(target, "r", encoding="utf-8") as handle:
            profile = ColorPointProfile.from_dict(json.load(handle))
        if profile.version != 1 or profile.expected_area <= 0.0:
            raise ValueError("unsupported or invalid head colour profile")
        profile.hue_tolerance = float(np.clip(profile.hue_tolerance, 3.0, 45.0))
        profile.sat_min = int(np.clip(profile.sat_min, 0, 255))
        profile.val_min = int(np.clip(profile.val_min, 0, 255))
        # Brightness changes with exposure; an upper V/S gate would turn a
        # valid illuminated marker into a miss.  Keep only lower bounds.
        profile.sat_max = 255
        profile.val_max = 255
        self.profile = profile
        self.profile_load_error = None
        self.reset_motion()
        return profile

    def save_profile(self, path: Optional[str] = None) -> None:
        target = path or self.profile_path
        if not target or self.profile is None:
            raise ValueError("head colour profile is not available")
        parent = os.path.dirname(os.path.abspath(target))
        os.makedirs(parent, exist_ok=True)
        temporary = f"{target}.tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(asdict(self.profile), handle, ensure_ascii=False, indent=2)
            handle.write("\n")
        os.replace(temporary, target)

    def calibrate_from_point(
        self,
        frame: np.ndarray,
        point: Sequence[float],
        *,
        sample_radius_px: int = 4,
        save: bool = True,
    ) -> ColorPointProfile:
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("invalid frame for head colour calibration")
        px, py = [int(round(value)) for value in point]
        radius = max(4, int(sample_radius_px))
        x1, x2 = max(0, px - radius), min(frame.shape[1], px + radius + 1)
        y1, y2 = max(0, py - radius), min(frame.shape[0], py + radius + 1)
        sample = frame[y1:y2, x1:x2]
        if sample.size == 0:
            raise ValueError("head calibration ROI is empty")
        hsv_image = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV)
        hsv = hsv_image.reshape(-1, 3)
        saturation = hsv[:, 1].astype(np.float32)
        value = hsv[:, 2].astype(np.float32)

        # The LED is small at the working 640x480 resolution.  A plain mean
        # over the whole click patch is easily biased by the grey body, blue
        # water, or a UI glyph that happens to touch the patch.  Weight pixels
        # by distance to the click and by chroma/brightness, then estimate the
        # hue from the strongest 30% only.  The radial term still tolerates a
        # one-pixel click error while strongly suppressing the patch boundary.
        yy, xx = np.mgrid[y1:y2, x1:x2]
        distance = np.hypot(xx.astype(np.float32) - float(px),
                            yy.astype(np.float32) - float(py))
        sigma = max(1.0, radius * 0.55)
        radial = np.exp(-(distance * distance) / (2.0 * sigma * sigma))
        chroma = np.clip((saturation.reshape(sample.shape[:2]) - 25.0) / 230.0, 0.0, 1.0)
        brightness = np.clip((value.reshape(sample.shape[:2]) - 20.0) / 235.0, 0.0, 1.0)
        weights = radial * (0.10 + 0.90 * chroma) * (0.20 + 0.80 * brightness)
        flat_weights = weights.reshape(-1)
        keep_count = max(4, int(round(flat_weights.size * 0.30)))
        keep_count = min(flat_weights.size, keep_count)
        keep_indices = np.argpartition(flat_weights, -keep_count)[-keep_count:]
        selected = hsv[keep_indices]
        selected_weights = flat_weights[keep_indices]
        if selected.shape[0] < 4:
            selected = hsv
            selected_weights = np.ones(selected.shape[0], dtype=np.float32)
        hues = selected[:, 0]
        hue_radians = hues.astype(np.float64) * (2.0 * math.pi / 180.0)
        weighted_sin = float(np.sum(selected_weights * np.sin(hue_radians)))
        weighted_cos = float(np.sum(selected_weights * np.cos(hue_radians)))
        if abs(weighted_sin) < 1e-12 and abs(weighted_cos) < 1e-12:
            centre = _circular_hue_mean(hues)
        else:
            centre = (math.degrees(math.atan2(weighted_sin, weighted_cos)) / 2.0) % 180.0
        distances = _hue_distance(hues, centre)
        tolerance = float(np.clip(np.percentile(distances, 90) + 8.0, 8.0, 35.0))
        profile = ColorPointProfile(
            hue_center=centre,
            hue_tolerance=tolerance,
            sat_min=int(np.clip(np.percentile(selected[:, 1], 5) - 20.0, 30.0, 220.0)),
            sat_max=255,
            val_min=int(np.clip(np.percentile(selected[:, 2], 5) - 20.0, 20.0, 220.0)),
            val_max=255,
            expected_area=float(max(4, selected.shape[0])),
        )
        self.profile = profile
        self.profile_load_error = None
        self.seed((float(px), float(py)))
        if save and self.profile_path:
            self.save_profile()
        return profile

    def _roi(
        self,
        frame_shape: Tuple[int, ...],
        hint: Optional[Sequence[float]],
        bbox: Optional[Sequence[float]],
        timestamp: float,
    ) -> Tuple[Optional[Tuple[int, int, int, int]], Optional[np.ndarray]]:
        height, width = int(frame_shape[0]), int(frame_shape[1])
        centres = []
        if self._position is not None and self._last_time is not None:
            elapsed = max(0.0, float(timestamp) - self._last_time)
            centres.append(self._position + self._velocity * min(elapsed, self.prediction_horizon_s))
        if hint is not None:
            point = np.asarray(hint, dtype=np.float64).reshape(2)
            if np.isfinite(point).all():
                centres.append(point)
        regions = []
        if bbox is not None and len(bbox) == 4:
            values = [float(value) for value in bbox]
            if np.isfinite(values).all():
                margin_x = max(8.0, (values[2] - values[0]) * 0.35)
                margin_y = max(8.0, (values[3] - values[1]) * 0.35)
                regions.append((values[0] - margin_x, values[1] - margin_y,
                                values[2] + margin_x, values[3] + margin_y))
        if centres:
            for centre in centres:
                regions.append((centre[0] - self.search_radius_px,
                                centre[1] - self.search_radius_px,
                                centre[0] + self.search_radius_px,
                                centre[1] + self.search_radius_px))
        if not regions:
            return None, None
        x1 = int(np.clip(min(region[0] for region in regions), 0, width - 1))
        y1 = int(np.clip(min(region[1] for region in regions), 0, height - 1))
        x2 = int(np.clip(max(region[2] for region in regions) + 1, x1 + 1, width))
        y2 = int(np.clip(max(region[3] for region in regions) + 1, y1 + 1, height))
        predicted = centres[0] if centres else None
        return (x1, y1, x2, y2), predicted

    def update(
        self,
        frame: np.ndarray,
        timestamp: float,
        *,
        hint: Optional[Sequence[float]] = None,
        bbox: Optional[Sequence[float]] = None,
    ) -> Dict[str, object]:
        result: Dict[str, object] = {
            "position": None,
            "confidence": 0.0,
            "source": "UNAVAILABLE",
            "valid": False,
            "search_roi": None,
            "area": 0.0,
            "score": 0.0,
        }
        if self.profile is None or frame is None or frame.ndim != 3:
            return result
        now = float(timestamp)
        search_roi, predicted = self._roi(frame.shape, hint, bbox, now)
        if search_roi is None:
            return result
        x1, y1, x2, y2 = search_roi
        result["search_roi"] = search_roi
        hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        frame_height, frame_width = int(frame.shape[0]), int(frame.shape[1])
        profile = self.profile
        mask = (
            (_hue_distance(hsv[:, :, 0], profile.hue_center) <= profile.hue_tolerance)
            & (hsv[:, :, 1] >= profile.sat_min)
            & (hsv[:, :, 2] >= profile.val_min)
        ).astype(np.uint8) * 255
        kernel = np.ones((self.morph_kernel_px, self.morph_kernel_px), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < max(2.0, profile.expected_area * 0.06):
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 1e-6:
                continue
            centre = np.asarray([
                x1 + moments["m10"] / moments["m00"],
                y1 + moments["m01"] / moments["m00"],
            ], dtype=np.float64)
            distance = 0.0 if predicted is None else float(np.linalg.norm(centre - predicted))
            distance_score = math.exp(-distance / max(self.search_radius_px, 1.0))
            area_score = math.exp(-abs(math.log(max(area, 1e-6) / max(profile.expected_area, 1e-6))))
            contour_mask = np.zeros_like(mask)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            pixels = hsv[contour_mask > 0]
            colour_score = float(np.clip(
                0.5 * np.mean(pixels[:, 1]) / 255.0
                + 0.5 * np.mean(pixels[:, 2]) / 255.0,
                0.0, 1.0,
            )) if pixels.size else 0.0
            score = float(np.clip(
                0.45 * distance_score + 0.30 * area_score + 0.25 * colour_score,
                0.0, 1.0,
            ))
            candidates.append((score, centre, area))
        if candidates:
            score, centre, area = max(candidates, key=lambda item: item[0])
            if self._position is not None and self._last_time is not None:
                elapsed = now - self._last_time
                if 1e-4 < elapsed <= 1.0:
                    velocity = (centre - self._position) / elapsed
                    self._velocity = (
                        self.velocity_alpha * velocity
                        + (1.0 - self.velocity_alpha) * self._velocity
                    )
            self._position = centre
            self._last_time = now
            result.update({
                "position": (float(centre[0]), float(centre[1])),
                "confidence": score,
                "source": "COLOR",
                "valid": True,
                "area": area,
                "score": score,
            })
            return result
        if self._position is not None and self._last_time is not None:
            elapsed = max(0.0, now - self._last_time)
            if elapsed <= self.prediction_horizon_s:
                predicted_position = self._position + self._velocity * elapsed
                decay = max(0.0, 1.0 - elapsed / self.prediction_horizon_s)
                result.update({
                    "position": (float(predicted_position[0]), float(predicted_position[1])),
                    "confidence": 0.35 * decay,
                    "source": "PREDICTION",
                    "valid": True,
                    "score": 0.35 * decay,
                })
        return result


class WhitePointTracker:
    """Track the best bright, low-saturation component near the fish body.

    White has no useful hue, so hue-based profiles are unsuitable here.  This
    tracker deliberately uses only saturation/value gates and a local ROI. The
    ROI is the YOLO fish box whenever available; a previous point is used only
    to keep prediction alive during a detector dropout.  The selected white
    region is the largest valid component in that box.  A component must also
    be compact and separated from the orange marker blob itself.
    """

    def __init__(
        self,
        *,
        search_radius_px: float = 72.0,
        prediction_horizon_s: float = 0.30,
        velocity_alpha: float = 0.55,
        sat_max: int = 95,
        val_min: int = 135,
        morph_kernel_px: int = 3,
        max_component_area_px: float = 2500.0,
        min_component_area_px: float = 12.0,
        max_component_aspect_ratio: float = 3.5,
        min_component_extent: float = 0.30,
        frame_edge_margin_px: int = 2,
        dynamic_value_sigma: float = 1.6,
        reacquire_after_misses: int = 5,
        bbox_expand_ratio: float = 0.0,
        tail_distance_ratio: float = 2.85,
        tail_distance_margin_px: float = 16.0,
        min_tail_separation_px: float = 10.0,
        min_tail_separation_ratio: float = 0.24,
        hinted_tail_separation_ratio: float = 0.18,
        tail_bbox_expand_px: float = 8.0,
    ):
        self.search_radius_px = max(16.0, float(search_radius_px))
        self.prediction_horizon_s = max(0.05, float(prediction_horizon_s))
        self.velocity_alpha = float(np.clip(velocity_alpha, 0.0, 1.0))
        self.sat_max = int(np.clip(sat_max, 0, 255))
        self.val_min = int(np.clip(val_min, 0, 255))
        self.morph_kernel_px = max(1, int(morph_kernel_px) | 1)
        self.max_component_area_px = max(16.0, float(max_component_area_px))
        self.min_component_area_px = max(3.0, float(min_component_area_px))
        self.max_component_aspect_ratio = max(1.0, float(max_component_aspect_ratio))
        self.min_component_extent = float(np.clip(min_component_extent, 0.05, 1.0))
        self.frame_edge_margin_px = max(0, int(frame_edge_margin_px))
        self.dynamic_value_sigma = max(0.25, float(dynamic_value_sigma))
        self.reacquire_after_misses = max(1, int(reacquire_after_misses))
        self.bbox_expand_ratio = max(0.0, float(bbox_expand_ratio))
        self.tail_distance_ratio = max(0.5, float(tail_distance_ratio))
        self.tail_distance_margin_px = max(0.0, float(tail_distance_margin_px))
        self.min_tail_separation_px = max(0.0, float(min_tail_separation_px))
        self.min_tail_separation_ratio = float(np.clip(
            min_tail_separation_ratio, 0.0, 1.0
        ))
        self.hinted_tail_separation_ratio = float(np.clip(
            hinted_tail_separation_ratio, 0.0, 1.0
        ))
        self.tail_bbox_expand_px = max(0.0, float(tail_bbox_expand_px))
        self._position: Optional[np.ndarray] = None
        self._velocity = np.zeros(2, dtype=np.float64)
        self._last_time: Optional[float] = None
        self._miss_count = 0
        self._last_binary_mask: Optional[np.ndarray] = None
        self._last_binary_roi: Optional[Tuple[int, int, int, int]] = None

    def reset(self) -> None:
        self._position = None
        self._velocity[:] = 0.0
        self._last_time = None
        self._miss_count = 0
        self._last_binary_mask = None
        self._last_binary_roi = None

    def set_binary_threshold(self, value: int) -> None:
        """Set the live V-channel floor used by the white binary mask.

        The per-frame adaptive threshold is still computed from the local ROI;
        this value acts as its absolute lower bound, allowing the UI slider to
        trade sensitivity for rejection of dim background/reflections.
        """
        self.val_min = int(np.clip(value, 0, 255))

    def binary_preview(self, frame) -> np.ndarray:
        """Return a full-frame binary mask using the live V threshold.

        Passing the current BGR frame produces a true camera-wide mask for
        diagnostics.  A shape tuple is still accepted as a fallback for code
        that only needs the most recent ROI mask.
        """
        if isinstance(frame, np.ndarray) and frame.ndim == 3 and frame.shape[2] == 3:
            height, width = int(frame.shape[0]), int(frame.shape[1])
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = ((hsv[:, :, 1] <= self.sat_max)
                    & (hsv[:, :, 2] >= self.val_min)).astype(np.uint8) * 255
            kernel = np.ones((self.morph_kernel_px, self.morph_kernel_px), dtype=np.uint8)
            return cv2.morphologyEx(
                cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel),
                cv2.MORPH_CLOSE,
                kernel,
            )

        frame_shape = frame
        height, width = int(frame_shape[0]), int(frame_shape[1])
        preview = np.zeros((height, width), dtype=np.uint8)
        if self._last_binary_mask is None or self._last_binary_roi is None:
            return preview
        x1, y1, x2, y2 = self._last_binary_roi
        roi_height = max(0, min(height, y2) - max(0, y1))
        roi_width = max(0, min(width, x2) - max(0, x1))
        if roi_height <= 0 or roi_width <= 0:
            return preview
        preview[max(0, y1):max(0, y1) + roi_height,
                max(0, x1):max(0, x1) + roi_width] = self._last_binary_mask[:roi_height, :roi_width]
        return preview

    def _build_roi(
        self,
        frame_shape: Tuple[int, ...],
        hint: Optional[Sequence[float]],
        bbox: Optional[Sequence[float]],
        timestamp: float,
        tail: Optional[Sequence[float]] = None,
    ) -> Tuple[Optional[Tuple[int, int, int, int]], Optional[np.ndarray]]:
        height, width = int(frame_shape[0]), int(frame_shape[1])
        regions = []
        centres = []
        if self._position is not None and self._last_time is not None:
            elapsed = max(0.0, float(timestamp) - self._last_time)
            centres.append(
                self._position
                + self._velocity * min(elapsed, self.prediction_horizon_s)
            )
        if hint is not None:
            point = np.asarray(hint, dtype=np.float64).reshape(2)
            if np.isfinite(point).all():
                centres.append(point)
        # The orange marker is a useful fallback anchor during a YOLO dropout;
        # it does not enlarge a valid YOLO fish-box search region.
        if tail is not None:
            tail_point = np.asarray(tail, dtype=np.float64).reshape(2)
            if np.isfinite(tail_point).all():
                centres.append(tail_point)
        bbox_valid = False
        if bbox is not None and len(bbox) == 4:
            values = np.asarray(bbox, dtype=np.float64)
            if np.isfinite(values).all() and values[2] > values[0] and values[3] > values[1]:
                bbox_region = (values[0], values[1], values[2], values[3])
                bbox_valid = True
        # When YOLO has a valid fish box, it is the complete white-search
        # domain.  Do not union it with a large tail window: that makes pool
        # reflections and the small white detail on the tail eligible again.
        if bbox_valid:
            regions.append(bbox_region)
        else:
            # Before the first YOLO result (or during a detector dropout), a
            # small temporal ROI keeps prediction alive.  It is deliberately
            # not used for a fresh, semantic head measurement once a YOLO box
            # is available.
            for centre in centres:
                radius = self.search_radius_px
                regions.append((centre[0] - radius,
                                centre[1] - radius,
                                centre[0] + radius,
                                centre[1] + radius))
        if not regions:
            return None, None
        x1 = int(np.clip(min(r[0] for r in regions), 0, width - 1))
        y1 = int(np.clip(min(r[1] for r in regions), 0, height - 1))
        x2 = int(np.clip(max(r[2] for r in regions) + 1, x1 + 1, width))
        y2 = int(np.clip(max(r[3] for r in regions) + 1, y1 + 1, height))
        return (x1, y1, x2, y2), (centres[0] if centres else None)

    def update(
        self,
        frame: np.ndarray,
        timestamp: float,
        *,
        hint: Optional[Sequence[float]] = None,
        bbox: Optional[Sequence[float]] = None,
        tail: Optional[Sequence[float]] = None,
        tail_bbox: Optional[Sequence[float]] = None,
        head_hint: Optional[Sequence[float]] = None,
    ) -> Dict[str, object]:
        result: Dict[str, object] = {
            "position": None,
            "confidence": 0.0,
            "source": "UNAVAILABLE",
            "valid": False,
            "search_roi": None,
            "area": 0.0,
            "score": 0.0,
            "bbox": None,
            "dynamic_value_min": None,
            "tail_distance_gate_px": None,
            "rejected_components": 0,
            "head_hint_inside": None,
            "head_hint_distance_px": None,
            "selection_mode": None,
        }
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            return result
        search_roi, predicted = self._build_roi(
            frame.shape, hint, bbox, float(timestamp), tail
        )
        if search_roi is None:
            return result
        x1, y1, x2, y2 = search_roi
        result["search_roi"] = search_roi
        hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        frame_height, frame_width = int(frame.shape[0]), int(frame.shape[1])
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2].astype(np.float32)
        # Outdoor illumination changes can move the same white marker by more
        # than the fixed V threshold. Estimate a local floor from the ROI
        # background while retaining the absolute safety floor. MAD is robust
        # against a small bright marker inside the ROI.
        # A 4x subsample is sufficient for illumination statistics and keeps
        # the camera-rate tracker inexpensive on a full fish-box ROI.
        value_flat = value[::4, ::4].reshape(-1)
        median_v = float(np.median(value_flat)) if value_flat.size else 0.0
        mad_v = float(np.median(np.abs(value_flat - median_v))) if value_flat.size else 0.0
        high_v = float(np.percentile(value_flat, 98.0)) if value_flat.size else 0.0
        robust_sigma = max(1.0, 1.4826 * mad_v)
        dynamic_candidate = max(
            float(self.val_min),
            median_v + self.dynamic_value_sigma * robust_sigma,
            # Even a perfectly uniform bright background has MAD=0; keep
            # a minimum contrast margin so the whole ROI is not selected.
            median_v + 12.0,
        )
        # On a strong outdoor gradient MAD can be large enough to push the
        # threshold above the marker. Cap it just below the bright tail of the
        # local histogram, but only when that tail is genuinely separated from
        # the median (otherwise a tiny marker would be mistaken for background).
        if high_v - median_v > 24.0:
            dynamic_candidate = min(dynamic_candidate, high_v - 5.0)
        dynamic_val_min = int(np.clip(dynamic_candidate, self.val_min, 245.0))
        result["dynamic_value_min"] = dynamic_val_min
        mask = ((saturation <= self.sat_max)
                & (value >= dynamic_val_min)).astype(np.uint8) * 255
        kernel = np.ones((self.morph_kernel_px, self.morph_kernel_px), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # ``mask`` is no longer mutated after this point; retain the array for
        # the optional debug preview without an extra full-ROI allocation.
        self._last_binary_mask = mask
        self._last_binary_roi = search_roi
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        tail_point = None
        if tail is not None:
            candidate_tail = np.asarray(tail, dtype=np.float64).reshape(2)
            if np.isfinite(candidate_tail).all():
                tail_point = candidate_tail
        tail_bbox_values = None
        if tail_bbox is not None and len(tail_bbox) == 4:
            candidate_tail_bbox = np.asarray(tail_bbox, dtype=np.float64)
            if (
                np.isfinite(candidate_tail_bbox).all()
                and candidate_tail_bbox[2] > candidate_tail_bbox[0]
                and candidate_tail_bbox[3] > candidate_tail_bbox[1]
            ):
                tail_bbox_values = candidate_tail_bbox

        # The bbox is the semantic fish region.  The distance gate and tail
        # separation below remain secondary checks for malformed boxes and for
        # white fragments attached to the orange tail.
        bbox_values = None
        bbox_inside_expand = None
        tail_distance_gate = None
        if bbox is not None and len(bbox) == 4:
            values = np.asarray(bbox, dtype=np.float64)
            if np.isfinite(values).all() and values[2] > values[0] and values[3] > values[1]:
                bbox_values = values
                bbox_width = float(values[2] - values[0])
                bbox_height = float(values[3] - values[1])
                bbox_diag = math.hypot(bbox_width, bbox_height)
                expand_x = bbox_width * self.bbox_expand_ratio
                expand_y = bbox_height * self.bbox_expand_ratio
                bbox_inside_expand = (
                    values[0] - expand_x,
                    values[1] - expand_y,
                    values[2] + expand_x,
                    values[3] + expand_y,
                )
                if tail_point is not None:
                    tail_distance_gate = (
                        self.tail_distance_ratio * bbox_diag
                        + self.tail_distance_margin_px
                    )
        if tail_distance_gate is None and tail_point is not None:
            # Without a YOLO box use a conservative pixel-scale fallback.  A
            # valid head may still be separated from the marker by a couple of
            # tracker radii, but a distant reflection should not be accepted.
            tail_distance_gate = max(
                self.search_radius_px * 3.0,
                220.0,
            )
        result["tail_distance_gate_px"] = tail_distance_gate
        head_hint_point = None
        if head_hint is not None:
            candidate_head_hint = np.asarray(head_hint, dtype=np.float64).reshape(2)
            if np.isfinite(candidate_head_hint).all():
                head_hint_point = candidate_head_hint
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            # Large connected regions are usually water-surface glare, not
            # the illuminated head marker. Without this gate a glare patch at
            # the image border can become a plausible yaw endpoint.
            if area < self.min_component_area_px or area > self.max_component_area_px:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 1e-6:
                continue
            centre = np.asarray([
                x1 + moments["m10"] / moments["m00"],
                y1 + moments["m01"] / moments["m00"],
            ], dtype=np.float64)
            bx, by, bw, bh = cv2.boundingRect(contour)
            if bw < 3 or bh < 3:
                result["rejected_components"] = int(result["rejected_components"]) + 1
                continue
            aspect_ratio = max(float(bw) / float(bh), float(bh) / float(bw))
            if aspect_ratio > self.max_component_aspect_ratio:
                # Thin white bars and single-pixel highlights are common near
                # the orange tail.  The illuminated head is a compact blob,
                # so reject this shape before brightness can make it win.
                result["rejected_components"] = int(result["rejected_components"]) + 1
                continue
            extent = area / max(float(bw * bh), 1.0)
            if extent < self.min_component_extent:
                result["rejected_components"] = int(result["rejected_components"]) + 1
                continue

            head_hint_match = False
            head_hint_distance = None
            if head_hint_point is not None:
                signed_distance = float(cv2.pointPolygonTest(
                    contour,
                    (
                        float(head_hint_point[0] - x1),
                        float(head_hint_point[1] - y1),
                    ),
                    True,
                ))
                head_hint_distance = max(0.0, -signed_distance)
                # YOLO's point can be a few pixels away from the illuminated
                # blob due to pose quantisation.  Treat a nearby point as an
                # in-region match, but never use a point on the orange tail as
                # the white endpoint.
                head_hint_match = signed_distance >= -8.0

            bbox_inside = True
            bbox_score = 0.55
            if bbox_inside_expand is not None:
                bbox_inside = (
                    bbox_inside_expand[0] <= centre[0] <= bbox_inside_expand[2]
                    and bbox_inside_expand[1] <= centre[1] <= bbox_inside_expand[3]
                )
                # Keep in-box observations preferred if a caller deliberately
                # supplies a small bbox tolerance.
                bbox_score = 1.0 if bbox_inside else 0.18

            distance = 0.0 if predicted is None else float(np.linalg.norm(centre - predicted))
            proximity = math.exp(-distance / max(self.search_radius_px, 1.0))
            contour_mask = np.zeros_like(mask)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            pixels = hsv[contour_mask > 0]
            if pixels.size == 0:
                continue
            brightness = float(np.clip(np.mean(pixels[:, 2]) / 255.0, 0.0, 1.0))
            whiteness = float(np.clip(1.0 - np.mean(pixels[:, 1]) / 255.0, 0.0, 1.0))
            tail_proximity = 0.5
            tail_distance = None
            if tail_point is not None:
                tail_distance = float(np.linalg.norm(centre - tail_point))
                if tail_bbox_values is not None:
                    tail_expand = self.tail_bbox_expand_px
                    if (
                        tail_bbox_values[0] - tail_expand
                        <= centre[0]
                        <= tail_bbox_values[2] + tail_expand
                        and tail_bbox_values[1] - tail_expand
                        <= centre[1]
                        <= tail_bbox_values[3] + tail_expand
                    ):
                        # A white highlight physically attached to the orange
                        # marker is part of the tail, not an independent head.
                        result["rejected_components"] = int(result["rejected_components"]) + 1
                        continue
                separation_floor = self.min_tail_separation_px
                if bbox_values is not None:
                    # A white fin/highlight can be compact and larger than the
                    # true head marker.  If YOLO does not corroborate this
                    # component, require it to be farther from the orange
                    # tail than a plausible attached fin.  A corroborated
                    # head hint may use a slightly looser ratio, but it still
                    # cannot be immediately adjacent to the tail.
                    separation_ratio = (
                        self.hinted_tail_separation_ratio
                        if head_hint_match
                        else self.min_tail_separation_ratio
                    )
                    separation_floor = max(
                        separation_floor,
                        separation_ratio * math.hypot(
                            float(bbox_values[2] - bbox_values[0]),
                            float(bbox_values[3] - bbox_values[1]),
                        ),
                    )
                # White pixels attached to/inside the orange tail are not a
                # second endpoint.  Reject them independently of score.
                if tail_distance < separation_floor:
                    result["rejected_components"] = int(result["rejected_components"]) + 1
                    continue
                if tail_distance_gate is not None and tail_distance > tail_distance_gate:
                    # A malformed/tolerated candidate still needs a geometric
                    # relation to the orange marker.  This also rejects the
                    # persistent ~275 px white strip seen in the replay.
                    result["rejected_components"] = int(result["rejected_components"]) + 1
                    continue
                tail_proximity = math.exp(
                    -tail_distance / max(self.search_radius_px * 1.35, 1.0)
                )
            if (
                not bbox_inside
                and tail_point is None
            ):
                # With no orange anchor there is no safe way to *reacquire* an
                # entirely outside-box component.  Keep an existing track
                # alive by prediction instead of accepting a new glare blob;
                # when the tail returns, the geometric gate can reacquire it.
                result["rejected_components"] = int(result["rejected_components"]) + 1
                continue
            area_score = float(np.clip(
                math.log1p(area) / math.log1p(self.max_component_area_px),
                0.0,
                1.0,
            ))
            score = float(np.clip(
                0.30 * proximity + 0.21 * whiteness + 0.14 * brightness
                + 0.14 * tail_proximity + 0.04 * area_score
                + 0.17 * bbox_score,
                0.0, 1.0,
            ))
            global_x1, global_y1 = x1 + bx, y1 + by
            global_x2, global_y2 = global_x1 + bw, global_y1 + bh
            margin = self.frame_edge_margin_px
            if (
                global_x1 <= margin
                or global_y1 <= margin
                or global_x2 >= frame_width - margin
                or global_y2 >= frame_height - margin
            ):
                # A component clipped by the camera edge has no reliable
                # centroid; wait for an interior observation instead of
                # steering from a partial reflection.
                continue
            candidates.append((
                score,
                centre,
                area,
                (x1 + bx, y1 + by, x1 + bx + bw, y1 + by + bh),
                head_hint_match,
                head_hint_distance,
                tail_distance,
            ))
        if candidates:
            self._miss_count = 0
            # Preserve the largest-component rule when YOLO has no usable
            # corroboration.  If a valid component contains the YOLO head
            # hint, however, prefer that component over a larger white fin;
            # the hint is only eligible after the tail/shape/ROI gates above.
            hinted_candidates = [item for item in candidates if item[4]]
            selection_mode = "YOLO_HEAD_HINT" if hinted_candidates else "LARGEST_WHITE"
            best_candidate = max(
                hinted_candidates or candidates,
                key=lambda item: (item[2], item[0]),
            )
            (
                score,
                centre,
                area,
                candidate_bbox,
                head_hint_match,
                head_hint_distance,
                selected_tail_distance,
            ) = best_candidate
            now = float(timestamp)
            if self._position is not None and self._last_time is not None:
                elapsed = now - self._last_time
                if 1e-4 < elapsed <= 1.0:
                    velocity = (centre - self._position) / elapsed
                    self._velocity = (
                        self.velocity_alpha * velocity
                        + (1.0 - self.velocity_alpha) * self._velocity
                    )
            self._position = centre
            self._last_time = now
            result.update({
                "position": (float(centre[0]), float(centre[1])),
                "confidence": score,
                "source": "WHITE",
                "valid": True,
                "area": area,
                "score": score,
                "bbox": tuple(int(value) for value in candidate_bbox),
                "tail_distance_px": selected_tail_distance,
                "head_hint_inside": bool(head_hint_match)
                if head_hint_point is not None else None,
                "head_hint_distance_px": head_hint_distance,
                "selection_mode": selection_mode,
            })
            return result
        self._miss_count += 1
        if self._miss_count >= self.reacquire_after_misses:
            # Drop a stale point (often a previously rejected border
            # reflection) so the next frame rebuilds the ROI from the current
            # tail/bbox anchors.
            self._position = None
            self._velocity[:] = 0.0
            self._last_time = None
        if self._position is not None and self._last_time is not None:
            elapsed = max(0.0, float(timestamp) - self._last_time)
            if elapsed <= self.prediction_horizon_s:
                predicted_position = self._position + self._velocity * elapsed
                decay = max(0.0, 1.0 - elapsed / self.prediction_horizon_s)
                result.update({
                    "position": (float(predicted_position[0]), float(predicted_position[1])),
                    "confidence": 0.30 * decay,
                    "source": "PREDICTION",
                    "valid": True,
                    "score": 0.30 * decay,
                })
        return result
