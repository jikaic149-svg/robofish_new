"""Causal circular Kalman filter for YOLO fish-body yaw.

The detector supplies a semantic rigid-rear -> head vector.  This module
filters that measured body orientation only; it never derives heading from
position or velocity.  Angles are expressed in mathematical radians (positive
counter-clockwise, with image/world ``y`` converted by the caller).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


def wrap_angle_rad(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class BodyYawKalmanConfig:
    # Pose inference runs at about 5 Hz. Allow one missed result while
    # keeping the response lighter than the original filter settings.
    measurement_std_deg: float = 3.0
    yaw_acceleration_std_rps2: float = 10.0
    max_prediction_s: float = 0.40
    min_confidence: float = 0.05
    max_innovation_deg: float = 85.0
    # Physical fish turning is continuous; reject a measurement that implies
    # an instantaneous U-turn between two detector samples.
    max_turn_rate_deg_s: float = 120.0
    # After a prolonged miss, do not let one new frame select a polarity
    # against a stale prediction.  Collect a short, temporally consistent
    # burst first, then re-initialize from its axial (180-degree agnostic)
    # mean.  This keeps a single bad keypoint pair from commanding a flip.
    reacquire_frames: int = 3
    reacquire_consistency_deg: float = 35.0


class BodyYawKalman:
    """Two-state constant-angular-rate Kalman filter.

    State is ``[yaw_rad, yaw_rate_rps]``.  Invalid or missing measurements
    result in a bounded prediction, marked ``source='PREDICTION'``.  Once the
    prediction age exceeds ``max_prediction_s`` the result is unusable until a
    new valid semantic keypoint pair arrives.
    """

    def __init__(self, config: BodyYawKalmanConfig | None = None):
        self.config = config or BodyYawKalmanConfig()
        if self.config.measurement_std_deg <= 0.0:
            raise ValueError("measurement_std_deg must be positive")
        if self.config.yaw_acceleration_std_rps2 <= 0.0:
            raise ValueError("yaw_acceleration_std_rps2 must be positive")
        if self.config.max_innovation_deg <= 0.0:
            raise ValueError("max_innovation_deg must be positive")
        if self.config.max_turn_rate_deg_s <= 0.0:
            raise ValueError("max_turn_rate_deg_s must be positive")
        if int(self.config.reacquire_frames) < 1:
            raise ValueError("reacquire_frames must be at least one")
        if self.config.reacquire_consistency_deg <= 0.0:
            raise ValueError("reacquire_consistency_deg must be positive")
        self.reset()

    def reset(self, yaw_rad: Optional[float] = None, timestamp: Optional[float] = None):
        self._yaw = None if yaw_rad is None else wrap_angle_rad(yaw_rad)
        self._rate = 0.0
        self._cov_00 = math.pi * math.pi
        self._cov_01 = 0.0
        self._cov_11 = math.pi * math.pi
        self._last_t = None if timestamp is None else float(timestamp)
        self._last_measurement_t = self._last_t if self._yaw is not None else None
        self._prediction_age = None
        self._last_measurement_rejected = False
        self._last_polarity_corrected = False
        self._needs_reacquire = False
        self._reacquire_samples = []
        self._polarity_locked = False

    def set_polarity_locked(self, locked: bool = True) -> None:
        """Lock head/tail polarity after the user's initial calibration."""
        self._polarity_locked = bool(locked)

    @property
    def initialized(self) -> bool:
        return self._yaw is not None

    def step(
        self,
        timestamp: float,
        yaw_rad: Optional[float],
        *,
        confidence: float = 1.0,
        measurement_valid: bool = True,
    ) -> dict:
        now = float(timestamp)
        if not math.isfinite(now):
            return self._result("INVALID", None)
        if self._last_t is not None and now <= self._last_t:
            return self._result("STALE", self._prediction_age)

        # A stale state must never be integrated once its bounded prediction
        # window has elapsed.  The previous implementation continued
        # integrating ``_rate`` on every call, so after a detector stall the
        # state could drift far away and the first recovered measurement was
        # incorrectly resolved as ``yaw + 180°``.
        if (
            self._yaw is not None
            and self._last_measurement_t is not None
            and not self._needs_reacquire
            and now - self._last_measurement_t > self.config.max_prediction_s
        ):
            self._enter_expired()

        dt = 0.0 if self._last_t is None else min(max(now - self._last_t, 0.0), 1.0)
        if self._yaw is not None and dt > 0.0 and not self._needs_reacquire:
            self._predict(dt)
        self._last_t = now

        confidence = float(confidence)
        self._last_polarity_corrected = False
        valid = (
            measurement_valid
            and yaw_rad is not None
            and math.isfinite(float(yaw_rad))
            and math.isfinite(confidence)
            and confidence >= self.config.min_confidence
        )
        if valid:
            measured = wrap_angle_rad(float(yaw_rad))
            if self._needs_reacquire:
                return self._reacquire(now, measured, confidence)
            accepted = True
            if self._yaw is None:
                self._yaw = measured
                self._rate = 0.0
                self._cov_00 = math.radians(self.config.measurement_std_deg) ** 2
                self._cov_01 = 0.0
                self._cov_11 = math.pi * math.pi
            else:
                accepted = self._update(measured, confidence, dt)
            self._last_measurement_rejected = not accepted
            if accepted:
                self._last_measurement_t = now
                self._prediction_age = 0.0
                return self._result("MEASUREMENT", self._prediction_age)
            # An implausible innovation (typically a 180-degree keypoint
            # polarity flip) must not overwrite the last valid body yaw.
            self._prediction_age = (
                None
                if self._last_measurement_t is None
                else max(0.0, now - self._last_measurement_t)
            )
            if (
                self._prediction_age is None
                or self._prediction_age > self.config.max_prediction_s
            ):
                self._enter_expired()
                return self._result("EXPIRED", self._prediction_age)
            return self._result("PREDICTION", self._prediction_age)

        self._last_measurement_rejected = False

        if self._yaw is None or self._last_measurement_t is None:
            self._prediction_age = None
            return self._result("INVALID", None)
        self._prediction_age = max(0.0, now - self._last_measurement_t)
        if self._prediction_age > self.config.max_prediction_s:
            self._enter_expired()
            return self._result("EXPIRED", self._prediction_age)
        return self._result("PREDICTION", self._prediction_age)

    @staticmethod
    def _axial_error(a: float, b: float) -> float:
        """Shortest difference when a body axis has unknown polarity."""
        return (float(a) - float(b) + 0.5 * math.pi) % math.pi - 0.5 * math.pi

    @staticmethod
    def _axial_mean(samples) -> float:
        # Doubling the angle maps an unoriented axis to a normal circular
        # variable; halve the result to return to the body-axis domain.
        sin_sum = sum(math.sin(2.0 * value) for value in samples)
        cos_sum = sum(math.cos(2.0 * value) for value in samples)
        return wrap_angle_rad(0.5 * math.atan2(sin_sum, cos_sum))

    def _enter_expired(self) -> None:
        """Freeze the last yaw and wait for a stable burst of new frames."""
        self._needs_reacquire = True
        self._reacquire_samples = []
        self._rate = 0.0
        # Keep a finite covariance for diagnostics, but do not carry the old
        # rate uncertainty into the re-initialized state.
        self._cov_01 = 0.0
        self._cov_11 = math.pi * math.pi

    def _reacquire(self, now: float, measured: float, confidence: float) -> dict:
        """Collect consistent raw axes, then re-seed the Kalman state.

        Samples are compared modulo 180° because the detector can briefly
        exchange head/tail keypoints.  Polarity is chosen only at commit time,
        against the frozen pre-expiry yaw (never against an integrated stale
        prediction).
        """
        samples = self._reacquire_samples
        if not samples or abs(self._axial_error(measured, samples[-1])) <= math.radians(
            self.config.reacquire_consistency_deg
        ):
            samples.append(measured)
        else:
            samples[:] = [measured]

        required = int(self.config.reacquire_frames)
        if len(samples) < required:
            self._prediction_age = (
                None
                if self._last_measurement_t is None
                else max(0.0, now - self._last_measurement_t)
            )
            self._last_measurement_rejected = False
            return self._result("REACQUIRE", self._prediction_age)

        axis = self._axial_mean(samples[-required:])
        candidate = axis
        if self._yaw is not None and not self._polarity_locked:
            alternate = wrap_angle_rad(axis + math.pi)
            if abs(wrap_angle_rad(alternate - self._yaw)) < abs(
                wrap_angle_rad(candidate - self._yaw)
            ):
                candidate = alternate
        elif self._yaw is not None and self._polarity_locked:
            # A calibrated fish cannot silently reacquire on the opposite
            # polarity.  Keep waiting for a consistent same-direction burst;
            # the operator can explicitly recalibrate if the fish truly turns
            # around after a long occlusion.
            if abs(wrap_angle_rad(candidate - self._yaw)) > math.radians(90.0):
                self._reacquire_samples = []
                self._prediction_age = max(0.0, now - self._last_measurement_t)
                self._last_measurement_rejected = True
                return self._result("REACQUIRE", self._prediction_age)

        self._yaw = candidate
        self._rate = 0.0
        self._cov_00 = math.radians(self.config.measurement_std_deg) ** 2
        self._cov_01 = 0.0
        self._cov_11 = math.pi * math.pi
        self._last_measurement_t = now
        self._prediction_age = 0.0
        self._needs_reacquire = False
        self._reacquire_samples = []
        self._last_measurement_rejected = False
        self._last_polarity_corrected = False
        return self._result("REACQUIRED", self._prediction_age)

    def _predict(self, dt: float) -> None:
        self._yaw = wrap_angle_rad(self._yaw + self._rate * dt)
        q = self.config.yaw_acceleration_std_rps2 ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        self._cov_00 += 2.0 * dt * self._cov_01 + dt2 * self._cov_11 + q * dt4 / 4.0
        self._cov_01 += dt * self._cov_11 + q * dt3 / 2.0
        self._cov_11 += q * dt2

    def _update(self, measured_yaw: float, confidence: float, dt: float) -> bool:
        # Confidence scales the measurement variance but is bounded so a
        # single high-confidence frame cannot create a discontinuous command.
        variance = math.radians(self.config.measurement_std_deg) ** 2
        # Low endpoint confidence should reduce the gain, but not turn a
        # visually stable pair into an almost-ignored measurement.
        variance /= max(0.30, min(1.0, confidence))
        # Before calibration the two keypoints form an unoriented axis, so an
        # alternate polarity can be considered.  Once calibrated, never select
        # the 180-degree alternative: a single swapped frame must be rejected.
        direct_innovation = wrap_angle_rad(measured_yaw - self._yaw)
        corrected = wrap_angle_rad(measured_yaw + math.pi)
        corrected_innovation = wrap_angle_rad(corrected - self._yaw)
        polarity_corrected = False
        if (not self._polarity_locked
                and abs(corrected_innovation) < abs(direct_innovation)):
            measured_yaw = corrected
            innovation = corrected_innovation
            polarity_corrected = True
        else:
            innovation = direct_innovation
        max_step_deg = max(
            20.0,
            min(
                self.config.max_innovation_deg,
                self.config.max_turn_rate_deg_s * max(float(dt), 1e-3) + 20.0,
            ),
        )
        max_step_rad = math.radians(max_step_deg)
        innovation_abs = abs(innovation)
        # A near-180° change is still treated as a swapped head/tail pair and
        # rejected outright.  Smaller excursions are limited to the physically
        # reachable turn in this sample interval instead of causing a long
        # chain of rejected frames and an unnecessary REACQUIRE cycle.
        if innovation_abs > math.radians(90.0):
            return False
        if innovation_abs > max_step_rad:
            innovation = math.copysign(max_step_rad, innovation)
        innovation_variance = self._cov_00 + variance
        if innovation_variance <= 1e-12:
            return False
        gain_yaw = self._cov_00 / innovation_variance
        gain_rate = self._cov_01 / innovation_variance
        self._yaw = wrap_angle_rad(self._yaw + gain_yaw * innovation)
        self._rate += gain_rate * innovation
        old_00, old_01, old_11 = self._cov_00, self._cov_01, self._cov_11
        self._cov_00 = max(1e-12, (1.0 - gain_yaw) * old_00)
        self._cov_01 = (1.0 - gain_yaw) * old_01
        self._cov_11 = max(1e-12, old_11 - gain_rate * old_01)
        # Only report a polarity correction when the measurement update was
        # actually accepted. Rejected updates may still have selected the
        # alternate candidate while failing the innovation/covariance gate.
        self._last_polarity_corrected = polarity_corrected
        return True

    def _result(self, source: str, prediction_age: Optional[float]) -> dict:
        usable = self._yaw is not None and source in {
            "MEASUREMENT", "PREDICTION", "REACQUIRED"
        }
        return {
            "yaw_rad": self._yaw if usable else None,
            "yaw_deg": math.degrees(self._yaw) if usable else None,
            "yaw_rate_rps": self._rate if usable else None,
            "source": source,
            "prediction_age_s": prediction_age,
            "usable": usable,
            "measurement_rejected": bool(self._last_measurement_rejected),
            "polarity_corrected": bool(self._last_polarity_corrected),
            "reacquire_pending": bool(self._needs_reacquire),
            "reacquire_samples": len(self._reacquire_samples),
        }
