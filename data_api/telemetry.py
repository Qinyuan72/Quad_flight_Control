from __future__ import annotations

import math
import time
from typing import Any

from .krpc_bindings import KrpcQuadHardware
from .models import RollRateTelemetry


class RollRateTelemetryReader:
    """Read telemetry needed for attitude and altitude loop testing."""

    def __init__(self, hardware: KrpcQuadHardware) -> None:
        self.hardware = hardware
        self.streams: dict[str, Any] = {}
        self._last_timestamp_s: float | None = None

    def connect(self) -> None:
        if self.streams:
            return
        if not self.hardware.is_connected():
            raise RuntimeError("Telemetry reader requires an active kRPC connection.")

        assert self.hardware.conn is not None
        assert self.hardware.vessel is not None
        assert self.hardware.body is not None

        vessel = self.hardware.vessel
        body = self.hardware.body
        conn = self.hardware.conn

        vessel_rf = vessel.reference_frame
        body_rf = body.reference_frame

        body_axes_surface_rate_rf = conn.space_center.ReferenceFrame.create_hybrid(
            position=vessel_rf,
            rotation=vessel_rf,
            velocity=body_rf,
            angular_velocity=body_rf,
        )

        flight_default = vessel.flight()
        self.streams = {
            "roll_deg": conn.add_stream(getattr, flight_default, "roll"),
            "pitch_deg": conn.add_stream(getattr, flight_default, "pitch"),
            "heading_deg": conn.add_stream(getattr, flight_default, "heading"),
            "mean_altitude": conn.add_stream(getattr, flight_default, "mean_altitude"),
            "vertical_speed": conn.add_stream(getattr, flight_default, "vertical_speed"),
            "position_body_m": conn.add_stream(vessel.position, body_rf),
            "velocity_body_m_s": conn.add_stream(vessel.velocity, body_rf),
            "body_rates_rfd_rad_s": conn.add_stream(vessel.angular_velocity, body_axes_surface_rate_rf),
        }
        self._last_timestamp_s = None

    def disconnect(self) -> None:
        for stream in self.streams.values():
            try:
                stream.remove()
            except Exception:
                pass
        self.streams.clear()
        self._last_timestamp_s = None

    def read(self) -> RollRateTelemetry:
        if not self.streams:
            raise RuntimeError("Telemetry reader is not initialized.")

        timestamp_s = time.monotonic()
        body_rates_rfd = self._sanitize_vector(tuple(self.streams["body_rates_rfd_rad_s"]()))
        body_rates_frd = self._rfd_to_frd(body_rates_rfd)
        roll_deg = self._sanitize_scalar(float(self.streams["roll_deg"]()))
        pitch_deg = self._sanitize_scalar(float(self.streams["pitch_deg"]()))
        heading_deg = self._sanitize_scalar(float(self.streams["heading_deg"]()))
        alt_m = self._sanitize_scalar(float(self.streams["mean_altitude"]()))
        raw_vz_m_s = self._sanitize_scalar(float(self.streams["vertical_speed"]()))
        position_body_m = self._sanitize_vector(tuple(self.streams["position_body_m"]()))
        velocity_body_m_s = self._sanitize_vector(tuple(self.streams["velocity_body_m_s"]()))
        dt_s = self._compute_dt(timestamp_s)
        vz_m_s = self._surface_vertical_speed_from_vectors(
            position_body_m=position_body_m,
            velocity_body_m_s=velocity_body_m_s,
            fallback_vz_m_s=raw_vz_m_s,
        )
        roll_rate_deg_s, pitch_rate_deg_s, yaw_rate_deg_s = self._derive_euler_rates(
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            body_rates_frd_rad_s=body_rates_frd,
        )

        return RollRateTelemetry(
            p_meas_rad_s=body_rates_frd[0],
            q_meas_rad_s=body_rates_frd[1],
            r_meas_rad_s=body_rates_frd[2],
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            heading_deg=heading_deg,
            roll_rate_deg_s=roll_rate_deg_s,
            pitch_rate_deg_s=pitch_rate_deg_s,
            yaw_rate_deg_s=yaw_rate_deg_s,
            alt_m=alt_m,
            vz_m_s=vz_m_s,
            body_rates_rfd_rad_s=body_rates_rfd,
            body_rates_frd_rad_s=body_rates_frd,
            timestamp_s=timestamp_s,
            dt_s=dt_s,
        )

    def _compute_dt(self, timestamp_s: float) -> float:
        if self._last_timestamp_s is None:
            self._last_timestamp_s = timestamp_s
            return 0.0
        dt_s = timestamp_s - self._last_timestamp_s
        self._last_timestamp_s = timestamp_s
        if not math.isfinite(dt_s) or dt_s < 0.0:
            return 0.0
        return dt_s

    def _surface_vertical_speed_from_vectors(
        self,
        *,
        position_body_m: tuple[float, float, float],
        velocity_body_m_s: tuple[float, float, float],
        fallback_vz_m_s: float,
    ) -> float:
        up_vector = self._normalize_vector(position_body_m)
        if up_vector is None:
            return fallback_vz_m_s
        return self._dot3(velocity_body_m_s, up_vector)

    def _derive_euler_rates(
        self,
        *,
        roll_deg: float,
        pitch_deg: float,
        body_rates_frd_rad_s: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Estimate Euler angle rates from FRD body rates for display/analysis."""

        phi = math.radians(roll_deg)
        theta = math.radians(pitch_deg)
        p_rate, q_rate, r_rate = body_rates_frd_rad_s

        cos_theta = math.cos(theta)
        if abs(cos_theta) < 1e-6:
            return (0.0, 0.0, 0.0)

        roll_rate = p_rate + math.sin(phi) * math.tan(theta) * q_rate + math.cos(phi) * math.tan(theta) * r_rate
        pitch_rate = math.cos(phi) * q_rate - math.sin(phi) * r_rate
        yaw_rate = (math.sin(phi) / cos_theta) * q_rate + (math.cos(phi) / cos_theta) * r_rate
        return (
            math.degrees(roll_rate),
            math.degrees(pitch_rate),
            math.degrees(yaw_rate),
        )

    def _rfd_to_frd(self, vector: tuple[float, float, float]) -> tuple[float, float, float]:
        right, forward, down = vector
        return (forward, right, down)

    def _normalize_vector(self, vector: tuple[float, float, float]) -> tuple[float, float, float] | None:
        magnitude = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])
        if magnitude < 1e-9 or not math.isfinite(magnitude):
            return None
        return (vector[0] / magnitude, vector[1] / magnitude, vector[2] / magnitude)

    def _dot3(self, a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return self._sanitize_scalar(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])

    def _sanitize_scalar(self, value: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return value

    def _sanitize_vector(self, vector: tuple[float, float, float]) -> tuple[float, float, float]:
        return tuple(self._sanitize_scalar(component) for component in vector)
