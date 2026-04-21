from __future__ import annotations

from dataclasses import dataclass
import math

try:
    from data_api.models import (
        AltitudeControlConfig,
        AngleOuterLoopConfig,
        AngleOuterLoopOutput,
        BodyVelocityOuterLoopConfig,
        BodyVelocityOuterLoopOutput,
        ControllerResult,
        WorldToBodyVelocityProjectionOutput,
        YawOuterLoopConfig,
        YawOuterLoopOutput,
    )
except ImportError:
    from ..data_api.models import (
        AltitudeControlConfig,
        AngleOuterLoopConfig,
        AngleOuterLoopOutput,
        BodyVelocityOuterLoopConfig,
        BodyVelocityOuterLoopOutput,
        ControllerResult,
        WorldToBodyVelocityProjectionOutput,
        YawOuterLoopConfig,
        YawOuterLoopOutput,
    )


def wrap_deg_180(angle_deg: float) -> float:
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


@dataclass
class BodyRatePIDController:
    """PID-capable body-rate inner-loop controller."""

    integrator_p: float = 0.0
    integrator_q: float = 0.0
    integrator_r: float = 0.0
    prev_p_meas: float | None = None
    prev_q_meas: float | None = None
    prev_r_meas: float | None = None

    def reset(self) -> None:
        self.integrator_p = 0.0
        self.integrator_q = 0.0
        self.integrator_r = 0.0
        self.prev_p_meas = None
        self.prev_q_meas = None
        self.prev_r_meas = None

    def compute(
        self,
        *,
        p_cmd_rad_s: float,
        q_cmd_rad_s: float,
        r_cmd_rad_s: float,
        p_meas_rad_s: float,
        q_meas_rad_s: float,
        r_meas_rad_s: float,
        kp_p: float,
        kp_q: float,
        kp_r: float,
        ki_p: float,
        ki_q: float,
        ki_r: float,
        kd_p: float,
        kd_q: float,
        kd_r: float,
        integrator_limit_p: float,
        integrator_limit_q: float,
        integrator_limit_r: float,
        dt_s: float,
        output_limit: float,
    ) -> ControllerResult:
        error_p = float(p_cmd_rad_s) - float(p_meas_rad_s)
        error_q = float(q_cmd_rad_s) - float(q_meas_rad_s)
        error_r = float(r_cmd_rad_s) - float(r_meas_rad_s)
        limit = abs(float(output_limit))

        roll_terms = self._compute_axis(
            error=error_p,
            meas=float(p_meas_rad_s),
            kp=float(kp_p),
            ki=float(ki_p),
            kd=float(kd_p),
            integrator=self.integrator_p,
            prev_meas=self.prev_p_meas,
            integrator_limit=float(integrator_limit_p),
            dt_s=float(dt_s),
            output_limit=limit,
        )
        pitch_terms = self._compute_axis(
            error=error_q,
            meas=float(q_meas_rad_s),
            kp=float(kp_q),
            ki=float(ki_q),
            kd=float(kd_q),
            integrator=self.integrator_q,
            prev_meas=self.prev_q_meas,
            integrator_limit=float(integrator_limit_q),
            dt_s=float(dt_s),
            output_limit=limit,
        )
        yaw_terms = self._compute_axis(
            error=error_r,
            meas=float(r_meas_rad_s),
            kp=float(kp_r),
            ki=float(ki_r),
            kd=float(kd_r),
            integrator=self.integrator_r,
            prev_meas=self.prev_r_meas,
            integrator_limit=float(integrator_limit_r),
            dt_s=float(dt_s),
            output_limit=limit,
        )

        self.integrator_p = float(roll_terms["integrator"])
        self.integrator_q = float(pitch_terms["integrator"])
        self.integrator_r = float(yaw_terms["integrator"])
        self.prev_p_meas = float(roll_terms["prev_meas"])
        self.prev_q_meas = float(pitch_terms["prev_meas"])
        self.prev_r_meas = float(yaw_terms["prev_meas"])

        return ControllerResult(
            error_p_rad_s=error_p,
            error_q_rad_s=error_q,
            error_r_rad_s=error_r,
            p_term_roll=float(roll_terms["p_term"]),
            i_term_roll=float(roll_terms["i_term"]),
            d_term_roll=float(roll_terms["d_term"]),
            p_term_pitch=float(pitch_terms["p_term"]),
            i_term_pitch=float(pitch_terms["i_term"]),
            d_term_pitch=float(pitch_terms["d_term"]),
            p_term_yaw=float(yaw_terms["p_term"]),
            i_term_yaw=float(yaw_terms["i_term"]),
            d_term_yaw=float(yaw_terms["d_term"]),
            u_roll=float(roll_terms["output"]),
            u_pitch=float(pitch_terms["output"]),
            u_yaw=float(yaw_terms["output"]),
        )

    def _compute_axis(
        self,
        *,
        error: float,
        meas: float,
        kp: float,
        ki: float,
        kd: float,
        integrator: float,
        prev_meas: float | None,
        integrator_limit: float,
        dt_s: float,
        output_limit: float,
    ) -> dict[str, float]:
        p_term = kp * error
        valid_dt = dt_s > 0.0
        d_term = 0.0
        if valid_dt and prev_meas is not None:
            d_term = -kd * ((meas - prev_meas) / dt_s)

        next_integrator = integrator
        if valid_dt and ki != 0.0:
            candidate_integrator = integrator + error * dt_s
            resolved_limit = self._resolve_integrator_limit(
                ki=ki,
                output_limit=output_limit,
                integrator_limit=integrator_limit,
            )
            candidate_integrator = self._clamp(candidate_integrator, -resolved_limit, resolved_limit)
            candidate_i_term = ki * candidate_integrator
            raw_output = p_term + candidate_i_term + d_term
            clipped_output = self._clip_output(raw_output, output_limit)
            driving_further_into_saturation = (
                raw_output != clipped_output
                and ((clipped_output >= output_limit and error > 0.0) or (clipped_output <= -output_limit and error < 0.0))
            )
            if not driving_further_into_saturation:
                next_integrator = candidate_integrator

        i_term = ki * next_integrator
        output = self._clip_output(p_term + i_term + d_term, output_limit)
        return {
            "p_term": p_term,
            "i_term": i_term,
            "d_term": d_term,
            "output": output,
            "integrator": next_integrator,
            "prev_meas": meas,
        }

    def _resolve_integrator_limit(self, *, ki: float, output_limit: float, integrator_limit: float) -> float:
        if integrator_limit > 0.0:
            return integrator_limit
        if ki != 0.0 and output_limit > 0.0:
            return output_limit / abs(ki)
        return 0.0

    def _clip_output(self, value: float, output_limit: float) -> float:
        if output_limit <= 0.0:
            return 0.0
        return self._clamp(value, -output_limit, output_limit)

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


@dataclass
class AngleOuterLoopController:
    config: AngleOuterLoopConfig
    integrator_roll: float = 0.0
    integrator_pitch: float = 0.0

    def reset(self) -> None:
        self.integrator_roll = 0.0
        self.integrator_pitch = 0.0

    def compute(
        self,
        *,
        roll_cmd_deg: float,
        pitch_cmd_deg: float,
        roll_meas_deg: float,
        pitch_meas_deg: float,
        roll_rate_rad_s: float = 0.0,
        pitch_rate_rad_s: float = 0.0,
        dt_s: float = 0.0,
    ) -> AngleOuterLoopOutput:
        roll_cmd_rad = math.radians(float(roll_cmd_deg))
        pitch_cmd_rad = math.radians(float(pitch_cmd_deg))
        roll_meas_rad = math.radians(float(roll_meas_deg))
        pitch_meas_rad = math.radians(float(pitch_meas_deg))
        error_roll_rad = roll_cmd_rad - roll_meas_rad
        error_pitch_rad = pitch_cmd_rad - pitch_meas_rad
        limit = abs(float(self.config.rate_limit_rad_s))
        p_cmd, next_integrator_roll = self._compute_axis(
            error_rad=error_roll_rad,
            measured_rate_rad_s=float(roll_rate_rad_s),
            kp=self._convert_gain_per_deg(float(self.config.kp_roll_angle)),
            ki=self._convert_gain_per_deg(float(self.config.ki_roll_angle)),
            kd=float(self.config.kd_roll_angle),
            integrator=self.integrator_roll,
            integrator_limit=float(self.config.integrator_limit_roll),
            dt_s=float(dt_s),
            output_limit=limit,
        )
        q_cmd, next_integrator_pitch = self._compute_axis(
            error_rad=error_pitch_rad,
            measured_rate_rad_s=float(pitch_rate_rad_s),
            kp=self._convert_gain_per_deg(float(self.config.kp_pitch_angle)),
            ki=self._convert_gain_per_deg(float(self.config.ki_pitch_angle)),
            kd=float(self.config.kd_pitch_angle),
            integrator=self.integrator_pitch,
            integrator_limit=float(self.config.integrator_limit_pitch),
            dt_s=float(dt_s),
            output_limit=limit,
        )
        self.integrator_roll = next_integrator_roll
        self.integrator_pitch = next_integrator_pitch

        return AngleOuterLoopOutput(
            roll_error_deg=math.degrees(error_roll_rad),
            pitch_error_deg=math.degrees(error_pitch_rad),
            p_cmd_rad_s=p_cmd,
            q_cmd_rad_s=q_cmd,
        )

    def _compute_axis(
        self,
        *,
        error_rad: float,
        measured_rate_rad_s: float,
        kp: float,
        ki: float,
        kd: float,
        integrator: float,
        integrator_limit: float,
        dt_s: float,
        output_limit: float,
    ) -> tuple[float, float]:
        p_term = kp * error_rad
        d_term = -kd * measured_rate_rad_s
        valid_dt = dt_s > 0.0
        next_integrator = integrator
        if valid_dt and ki != 0.0:
            candidate_integrator = integrator + error_rad * dt_s
            resolved_limit = self._resolve_integrator_limit(
                ki=ki,
                output_limit=output_limit,
                integrator_limit=integrator_limit,
            )
            candidate_integrator = self._clamp(candidate_integrator, -resolved_limit, resolved_limit)
            candidate_i_term = ki * candidate_integrator
            raw_output = p_term + candidate_i_term + d_term
            clipped_output = self._clip_output(raw_output, output_limit)
            driving_further_into_saturation = (
                raw_output != clipped_output
                and ((clipped_output >= output_limit and error_rad > 0.0) or (clipped_output <= -output_limit and error_rad < 0.0))
            )
            if not driving_further_into_saturation:
                next_integrator = candidate_integrator

        i_term = ki * next_integrator
        output = self._clip_output(p_term + i_term + d_term, output_limit)
        return (output, next_integrator)

    def _convert_gain_per_deg(self, gain_per_deg: float) -> float:
        return gain_per_deg * (180.0 / math.pi)

    def _resolve_integrator_limit(self, *, ki: float, output_limit: float, integrator_limit: float) -> float:
        if integrator_limit > 0.0:
            return integrator_limit
        if ki != 0.0 and output_limit > 0.0:
            return output_limit / abs(ki)
        return 0.0

    def _clip_output(self, value: float, output_limit: float) -> float:
        if output_limit <= 0.0:
            return 0.0
        return self._clamp(value, -output_limit, output_limit)

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


@dataclass
class YawOuterLoopController:
    config: YawOuterLoopConfig

    def compute(self, *, yaw_cmd_deg: float, heading_deg: float) -> YawOuterLoopOutput:
        yaw_error_deg = wrap_deg_180(float(yaw_cmd_deg) - float(heading_deg))
        raw_r_cmd = float(self.config.kp_yaw) * math.radians(yaw_error_deg)
        limit = abs(float(self.config.yaw_rate_limit_rad_s))
        if limit <= 0.0:
            r_cmd_rad_s = 0.0
        else:
            r_cmd_rad_s = max(-limit, min(limit, raw_r_cmd))
        return YawOuterLoopOutput(
            yaw_error_deg=yaw_error_deg,
            r_cmd_rad_s=r_cmd_rad_s,
            yaw_cmd_deg=float(yaw_cmd_deg),
            heading_deg=float(heading_deg),
        )


@dataclass
class BodyVelocityOuterLoopController:
    config: BodyVelocityOuterLoopConfig

    def reset(self) -> None:
        return None

    def compute(
        self,
        *,
        v_forward_cmd_m_s: float,
        v_right_cmd_m_s: float,
        vx_body_m_s: float,
        vy_body_m_s: float,
    ) -> BodyVelocityOuterLoopOutput:
        v_forward_error_m_s = float(v_forward_cmd_m_s) - float(vx_body_m_s)
        v_right_error_m_s = float(v_right_cmd_m_s) - float(vy_body_m_s)
        limit_deg = abs(float(self.config.velocity_angle_limit_deg))

        # TODO: Confirm the final flight-test sign against the craft convention.
        raw_pitch_cmd_deg = float(self.config.kp_v_forward) * v_forward_error_m_s
        raw_roll_cmd_deg = float(self.config.kp_v_right) * v_right_error_m_s

        if limit_deg <= 0.0:
            pitch_cmd_deg = 0.0
            roll_cmd_deg = 0.0
        else:
            pitch_cmd_deg = max(-limit_deg, min(limit_deg, raw_pitch_cmd_deg))
            roll_cmd_deg = max(-limit_deg, min(limit_deg, raw_roll_cmd_deg))

        return BodyVelocityOuterLoopOutput(
            v_forward_cmd_m_s=float(v_forward_cmd_m_s),
            v_right_cmd_m_s=float(v_right_cmd_m_s),
            v_forward_error_m_s=v_forward_error_m_s,
            v_right_error_m_s=v_right_error_m_s,
            pitch_cmd_from_velocity_deg=pitch_cmd_deg,
            roll_cmd_from_velocity_deg=roll_cmd_deg,
        )


@dataclass
class WorldToBodyVelocityProjection:
    """
    Debug-only horizontal world-to-body velocity projection.

    This helper is kept for the existing command-side projection chain. It is
    not the raw UNE feedback source and it is not the full velocity outer loop.
    """

    def compute(
        self,
        *,
        v_north_cmd_m_s: float,
        v_east_cmd_m_s: float,
        heading_deg: float,
    ) -> WorldToBodyVelocityProjectionOutput:
        heading_rad = math.radians(float(heading_deg))
        north_cmd = float(v_north_cmd_m_s)
        east_cmd = float(v_east_cmd_m_s)
        # TODO: Confirm the final flight-test sign against the craft convention.
        # Current assumption: heading 0 deg points body-forward north, positive
        # body-right points east when heading is 0 deg.
        v_forward_cmd = north_cmd * math.cos(heading_rad) + east_cmd * math.sin(heading_rad)
        v_right_cmd = -north_cmd * math.sin(heading_rad) + east_cmd * math.cos(heading_rad)
        return WorldToBodyVelocityProjectionOutput(
            v_forward_cmd_from_world_m_s=v_forward_cmd,
            v_right_cmd_from_world_m_s=v_right_cmd,
        )


@dataclass
class AltitudeOuterLoopController:
    config: AltitudeControlConfig

    def compute(self, *, alt_cmd_m: float, alt_m: float) -> tuple[float, float]:
        alt_error_m = float(alt_cmd_m) - float(alt_m)
        vz_max = abs(float(self.config.vz_max))
        raw_vz_cmd = float(self.config.kp_alt) * alt_error_m
        if vz_max <= 0.0:
            vz_cmd_m_s = 0.0
        else:
            vz_cmd_m_s = max(-vz_max, min(vz_max, raw_vz_cmd))
        return (alt_error_m, vz_cmd_m_s)


@dataclass
class VerticalSpeedPIDController:
    config: AltitudeControlConfig
    integrator_vz: float = 0.0
    prev_vz_meas: float | None = None

    def reset(self) -> None:
        self.integrator_vz = 0.0
        self.prev_vz_meas = None

    def compute(
        self,
        *,
        vz_cmd_m_s: float,
        vz_m_s: float,
        dt_s: float,
        hover_throttle: float,
    ) -> tuple[float, float, float, float, float, float]:
        vz_error_m_s = float(vz_cmd_m_s) - float(vz_m_s)
        valid_dt = float(dt_s) > 0.0
        p_term = float(self.config.kp_vz) * vz_error_m_s
        d_term = 0.0
        if valid_dt and self.prev_vz_meas is not None:
            d_term = -float(self.config.kd_vz) * ((float(vz_m_s) - self.prev_vz_meas) / float(dt_s))

        if valid_dt and float(self.config.ki_vz) != 0.0:
            candidate_integrator = self.integrator_vz + vz_error_m_s * float(dt_s)
            limit = self._resolve_integrator_limit()
            candidate_integrator = self._clamp(candidate_integrator, -limit, limit)
            candidate_i_term = float(self.config.ki_vz) * candidate_integrator
            raw_correction = p_term + candidate_i_term + d_term
            clipped_correction = self._clamp_to_throttle_bounds(raw_correction, hover_throttle)
            driving_further_into_saturation = (
                raw_correction != clipped_correction
                and (
                    (float(hover_throttle) + clipped_correction >= float(self.config.throttle_max) and vz_error_m_s > 0.0)
                    or (float(hover_throttle) + clipped_correction <= float(self.config.throttle_min) and vz_error_m_s < 0.0)
                )
            )
            if not driving_further_into_saturation:
                self.integrator_vz = candidate_integrator

        i_term = float(self.config.ki_vz) * self.integrator_vz
        throttle_correction = p_term + i_term + d_term
        throttle_cmd = max(
            float(self.config.throttle_min),
            min(float(self.config.throttle_max), float(hover_throttle) + throttle_correction),
        )
        self.prev_vz_meas = float(vz_m_s)
        return (vz_error_m_s, p_term, i_term, d_term, throttle_correction, throttle_cmd)

    def _resolve_integrator_limit(self) -> float:
        if float(self.config.vz_integrator_limit) > 0.0:
            return float(self.config.vz_integrator_limit)
        if float(self.config.ki_vz) != 0.0:
            return (float(self.config.throttle_max) - float(self.config.throttle_min)) / abs(float(self.config.ki_vz))
        return 0.0

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def _clamp_to_throttle_bounds(self, correction: float, hover_throttle: float) -> float:
        return self._clamp(
            float(correction),
            float(self.config.throttle_min) - float(hover_throttle),
            float(self.config.throttle_max) - float(hover_throttle),
        )
