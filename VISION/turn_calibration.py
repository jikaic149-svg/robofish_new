"""从视觉位置样本拟合机器鱼 HYBRID 满转半径。

该模块只负责被动采样、几何拟合和本地保存，不发送任何运动指令。
世界坐标沿用视觉主程序：x 向右、y 向下，因此圆周角递减表示左转，
圆周角递增表示右转。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import os
import time
from typing import Dict, Iterable, Optional

import numpy as np


CALIBRATION_VERSION = 1
VALID_DIRECTIONS = ("LEFT", "RIGHT")


class TurnCalibrationError(ValueError):
    """转圈数据不足或质量不合格。"""


@dataclass(frozen=True)
class TurnCircleFit:
    direction: str
    center_x_m: float
    center_y_m: float
    radius_m: float
    rms_residual_m: float
    arc_coverage_deg: float
    sample_count: int
    inlier_count: int
    started_at: float
    ended_at: float

    @property
    def diameter_m(self) -> float:
        return 2.0 * self.radius_m

    @property
    def curvature_per_m(self) -> float:
        return 1.0 / self.radius_m

    def to_dict(self) -> dict:
        payload = asdict(self)
        payload.update({
            "diameter_m": self.diameter_m,
            "curvature_per_m": self.curvature_per_m,
            "mode": "HYBRID",
            "turn_strength": 1.0,
        })
        return payload

    @classmethod
    def from_dict(cls, payload: dict) -> "TurnCircleFit":
        direction = str(payload["direction"]).upper()
        if direction not in VALID_DIRECTIONS:
            raise TurnCalibrationError(f"未知转向方向：{direction}")
        result = cls(
            direction=direction,
            center_x_m=float(payload["center_x_m"]),
            center_y_m=float(payload["center_y_m"]),
            radius_m=float(payload["radius_m"]),
            rms_residual_m=float(payload["rms_residual_m"]),
            arc_coverage_deg=float(payload["arc_coverage_deg"]),
            sample_count=int(payload["sample_count"]),
            inlier_count=int(payload["inlier_count"]),
            started_at=float(payload["started_at"]),
            ended_at=float(payload["ended_at"]),
        )
        if not math.isfinite(result.radius_m) or result.radius_m <= 0.0:
            raise TurnCalibrationError("保存的转圈半径无效")
        return result


def _algebraic_circle_fit(points: np.ndarray, weights=None):
    """以线性最小二乘求圆；坐标先归一化以改善数值条件。"""
    points = np.asarray(points, dtype=np.float64).reshape((-1, 2))
    origin = np.mean(points, axis=0)
    scale = float(np.sqrt(np.mean(np.sum((points - origin) ** 2, axis=1))))
    if not math.isfinite(scale) or scale <= 1e-6:
        raise TurnCalibrationError("位置几乎没有移动，无法拟合圆")

    normalized = (points - origin) / scale
    x = normalized[:, 0]
    y = normalized[:, 1]
    matrix = np.column_stack((2.0 * x, 2.0 * y, np.ones(len(points))))
    target = x * x + y * y
    if weights is not None:
        root_weight = np.sqrt(np.clip(
            np.asarray(weights, dtype=np.float64).reshape(-1), 1e-6, 1.0
        ))
        matrix = matrix * root_weight[:, None]
        target = target * root_weight

    solution, _, rank, _ = np.linalg.lstsq(matrix, target, rcond=None)
    if rank < 3:
        raise TurnCalibrationError("采样轨迹接近直线，无法拟合转圈半径")
    center_normalized = solution[:2]
    radius_sq = float(solution[2] + np.dot(
        center_normalized, center_normalized
    ))
    if not math.isfinite(radius_sq) or radius_sq <= 0.0:
        raise TurnCalibrationError("圆拟合结果无效")
    center = origin + center_normalized * scale
    radius = math.sqrt(radius_sq) * scale
    return center, radius


def _robust_circle_fit(points: np.ndarray):
    """Huber 迭代后用 MAD 门控剔除少量反光/色块跳点。"""
    weights = np.ones(len(points), dtype=np.float64)
    center = None
    radius = None
    for _ in range(8):
        center, radius = _algebraic_circle_fit(points, weights)
        residuals = np.linalg.norm(points - center, axis=1) - radius
        median = float(np.median(residuals))
        mad = float(np.median(np.abs(residuals - median)))
        robust_scale = max(0.001, 1.4826 * mad)
        huber_limit = 1.5 * robust_scale
        magnitude = np.abs(residuals - median)
        new_weights = np.ones(len(points), dtype=np.float64)
        outside = magnitude > huber_limit
        new_weights[outside] = huber_limit / np.maximum(
            magnitude[outside], 1e-9
        )
        if float(np.max(np.abs(new_weights - weights))) < 1e-3:
            weights = new_weights
            break
        weights = new_weights

    residuals = np.linalg.norm(points - center, axis=1) - radius
    median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - median)))
    robust_scale = max(0.001, 1.4826 * mad)
    gate_m = max(0.015, min(0.080, 3.5 * robust_scale))
    inliers = np.abs(residuals - median) <= gate_m
    if int(np.count_nonzero(inliers)) >= 3:
        center, radius = _algebraic_circle_fit(points[inliers])
        residuals = np.linalg.norm(points - center, axis=1) - radius
        inlier_residuals = residuals[inliers]
    else:
        inlier_residuals = residuals
    return center, float(radius), inliers, inlier_residuals


def fit_turn_circle(
    samples: Iterable[Iterable[float]],
    timestamps: Optional[Iterable[float]] = None,
    *,
    min_samples: int = 25,
    min_arc_coverage_deg: float = 180.0,
    min_radius_m: float = 0.04,
    max_radius_m: float = 2.0,
) -> TurnCircleFit:
    """稳健拟合转圈半径，并拒绝样本不足、圆弧不足和高残差结果。"""
    points = np.asarray(list(samples), dtype=np.float64).reshape((-1, 2))
    if len(points) < int(min_samples):
        raise TurnCalibrationError(
            f"有效样本不足：{len(points)}/{int(min_samples)}"
        )
    if not np.isfinite(points).all():
        raise TurnCalibrationError("采样中包含无效坐标")

    if timestamps is None:
        sample_times = np.arange(len(points), dtype=np.float64)
    else:
        sample_times = np.asarray(list(timestamps), dtype=np.float64).reshape(-1)
        if len(sample_times) != len(points) or not np.isfinite(sample_times).all():
            raise TurnCalibrationError("采样时间与位置不匹配")
        if np.any(np.diff(sample_times) <= 0.0):
            raise TurnCalibrationError("采样时间必须严格递增")

    center, radius, inlier_mask, inlier_residuals = _robust_circle_fit(points)
    inlier_count = int(np.count_nonzero(inlier_mask))
    if inlier_count < int(min_samples):
        raise TurnCalibrationError(
            f"剔除跳点后样本不足：{inlier_count}/{int(min_samples)}"
        )
    if inlier_count < math.ceil(0.70 * len(points)):
        raise TurnCalibrationError("反光或色块跳点过多，可靠样本不足 70%")
    if not (float(min_radius_m) <= radius <= float(max_radius_m)):
        raise TurnCalibrationError(
            f"拟合半径 {radius:.3f}m 超出允许范围 "
            f"{float(min_radius_m):.2f}~{float(max_radius_m):.2f}m"
        )

    inlier_points = points[inlier_mask]
    angles = np.unwrap(np.arctan2(
        inlier_points[:, 1] - center[1],
        inlier_points[:, 0] - center[0],
    ))
    net_angle = float(angles[-1] - angles[0])
    arc_coverage_deg = abs(math.degrees(net_angle))
    if arc_coverage_deg < float(min_arc_coverage_deg):
        raise TurnCalibrationError(
            f"圆弧覆盖不足：{arc_coverage_deg:.1f}°/"
            f"{float(min_arc_coverage_deg):.0f}°，请至少转过半圈"
        )

    signed_steps = np.sign(net_angle) * np.diff(angles)
    forward_angle = float(np.sum(np.clip(signed_steps, 0.0, None)))
    backward_angle = float(np.sum(np.clip(-signed_steps, 0.0, None)))
    if forward_angle <= 1e-6 or backward_angle > max(
        math.radians(30.0), 0.35 * forward_angle
    ):
        raise TurnCalibrationError("转圈方向变化过多，请保持同一方向再测一次")

    rms_residual_m = float(np.sqrt(np.mean(inlier_residuals ** 2)))
    max_allowed_rms_m = max(0.035, 0.20 * radius)
    if rms_residual_m > max_allowed_rms_m:
        raise TurnCalibrationError(
            f"圆拟合残差过大：{rms_residual_m:.3f}m，"
            "请检查反光、鱼尾丢失或场地标定"
        )

    # y 向下坐标：角度递减为左转，角度递增为右转。
    direction = "LEFT" if net_angle < 0.0 else "RIGHT"
    return TurnCircleFit(
        direction=direction,
        center_x_m=float(center[0]),
        center_y_m=float(center[1]),
        radius_m=radius,
        rms_residual_m=rms_residual_m,
        arc_coverage_deg=arc_coverage_deg,
        sample_count=len(points),
        inlier_count=inlier_count,
        started_at=float(sample_times[0]),
        ended_at=float(sample_times[-1]),
    )


class TurnCalibrationSession:
    """供 UI 使用的一次转圈被动采样会话。"""

    def __init__(self, min_sample_spacing_m: float = 0.003):
        self.min_sample_spacing_m = max(0.0, float(min_sample_spacing_m))
        self.active = False
        self.points = []
        self.timestamps = []
        self.skipped_samples = 0

    @property
    def sample_count(self) -> int:
        return len(self.points)

    def start(self, position=None, timestamp=None) -> None:
        self.points = []
        self.timestamps = []
        self.skipped_samples = 0
        self.active = True
        if position is not None and timestamp is not None:
            self.add(position, timestamp)

    def add(self, position, timestamp) -> bool:
        if not self.active:
            return False
        point = np.asarray(position, dtype=np.float64).reshape(2)
        now = float(timestamp)
        if not np.isfinite(point).all() or not math.isfinite(now):
            self.skipped_samples += 1
            return False
        if self.timestamps and now <= self.timestamps[-1]:
            self.skipped_samples += 1
            return False
        if self.points:
            distance = float(np.linalg.norm(point - self.points[-1]))
            if distance < self.min_sample_spacing_m:
                self.skipped_samples += 1
                return False
        self.points.append(point.copy())
        self.timestamps.append(now)
        return True

    def cancel(self) -> None:
        self.active = False
        self.points = []
        self.timestamps = []
        self.skipped_samples = 0

    def finish(self) -> TurnCircleFit:
        if not self.active:
            raise TurnCalibrationError("当前没有正在进行的转圈测量")
        self.active = False
        return fit_turn_circle(self.points, self.timestamps)


def load_turn_calibrations(path: str) -> Dict[str, TurnCircleFit]:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as file:
        payload = json.load(file)
    if int(payload.get("version", -1)) != CALIBRATION_VERSION:
        raise TurnCalibrationError("转圈标定文件版本不支持")
    results = {}
    for direction, entry in payload.get("directions", {}).items():
        result = TurnCircleFit.from_dict(entry)
        if result.direction != str(direction).upper():
            raise TurnCalibrationError("转圈标定方向字段不一致")
        results[result.direction] = result
    return results


def save_turn_calibration(path: str, result: TurnCircleFit) -> None:
    try:
        existing = load_turn_calibrations(path)
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        existing = {}
    existing[result.direction] = result
    payload = {
        "version": CALIBRATION_VERSION,
        "updated_at": time.time(),
        "note": "HYBRID 满转视觉拟合；左右分别保存，仅作为曲率前馈端点",
        "directions": {
            direction: existing[direction].to_dict()
            for direction in VALID_DIRECTIONS
            if direction in existing
        },
    }
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary_path = f"{path}.tmp"
    with open(temporary_path, "w", encoding="utf-8") as file:
        json.dump(payload, file, ensure_ascii=False, indent=2)
        file.write("\n")
    os.replace(temporary_path, path)
