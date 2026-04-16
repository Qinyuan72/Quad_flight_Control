from __future__ import annotations

from dataclasses import dataclass

try:
    from data_api.models import (
        AltitudeCommand,
        AltitudeControlConfig,
        AltitudeLoopOutput,
        AngleCommand,
        AngleOuterLoopConfig,
        AngleOuterLoopOutput,
        ControllerResult,
        RollRateTelemetry,
        RollRateTestCommand,
    )
except ImportError:
    from ..data_api.models import (
        AltitudeCommand,
        AltitudeControlConfig,
        AltitudeLoopOutput,
        AngleCommand,
        AngleOuterLoopConfig,
        AngleOuterLoopOutput,
        ControllerResult,
        RollRateTelemetry,
        RollRateTestCommand,
    )


@dataclass
class RollPitchRatePController:
    """P-only roll/pitch inner-loop controller for sign and gain testing."""

    def compute(
        self,
        *,
        p_cmd_rad_s: float,
        q_cmd_rad_s: float,
        p_meas_rad_s: float,
        q_meas_rad_s: float,
        kp_p: float,
        kp_q: float,
        output_limit: float,
    ) -> ControllerResult:
        error_p = float(p_cmd_rad_s) - float(p_meas_rad_s)
        error_q = float(q_cmd_rad_s) - float(q_meas_rad_s)
        limit = abs(float(output_limit))

        raw_u_roll = float(kp_p) * error_p
        raw_u_pitch = float(kp_q) * error_q

        if limit <= 0.0:
            u_roll = 0.0
            u_pitch = 0.0
        else:
            u_roll = max(-limit, min(limit, raw_u_roll))
            u_pitch = max(-limit, min(limit, raw_u_pitch))

        return ControllerResult(
            error_p_rad_s=error_p,
            error_q_rad_s=error_q,
            u_roll=u_roll,
            u_pitch=u_pitch,
        )


@dataclass
class AngleOuterLoopController:
    config: AngleOuterLoopConfig

    def compute(
        self,
        *,
        roll_cmd_deg: float,
        pitch_cmd_deg: float,
        roll_meas_deg: float,
        pitch_meas_deg: float,
    ) -> AngleOuterLoopOutput:
        error_roll_deg = float(roll_cmd_deg) - float(roll_meas_deg)
        error_pitch_deg = float(pitch_cmd_deg) - float(pitch_meas_deg)
        limit = abs(float(self.config.rate_limit_rad_s))

        raw_p_cmd = float(self.config.kp_roll_angle) * error_roll_deg
        raw_q_cmd = float(self.config.kp_pitch_angle) * error_pitch_deg

        if limit <= 0.0:
            p_cmd = 0.0
            q_cmd = 0.0
        else:
            p_cmd = max(-limit, min(limit, raw_p_cmd))
            q_cmd = max(-limit, min(limit, raw_q_cmd))

        return AngleOuterLoopOutput(
            roll_error_deg=error_roll_deg,
            pitch_error_deg=error_pitch_deg,
            p_cmd_rad_s=p_cmd,
            q_cmd_rad_s=q_cmd,
        )


@dataclass
class AltitudeOuterLoopController:
    """Retained for future altitude outer-loop work."""

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
class VerticalSpeedPController:
    """P-only vertical-speed controller that outputs a collective correction."""

    config: AltitudeControlConfig

    def reset(self) -> None:
        return None

    def compute(
        self,
        *,
        vz_cmd_m_s: float,
        vz_m_s: float,
        hover_throttle: float,
    ) -> tuple[float, float, float]:
        vz_error_m_s = float(vz_cmd_m_s) - float(vz_m_s)
        throttle_correction = float(self.config.kp_vz) * vz_error_m_s
        throttle_cmd = max(
            float(self.config.throttle_min),
            min(float(self.config.throttle_max), float(hover_throttle) + throttle_correction),
        )
        return (vz_error_m_s, throttle_correction, throttle_cmd)


@dataclass
class CascadedRollPitchController:
    rate_controller: RollPitchRatePController
    angle_controller: AngleOuterLoopController

    def set_outer_loop_config(self, config: AngleOuterLoopConfig) -> None:
        self.angle_controller.config = config

    def compute(
        self,
        *,
        telemetry: RollRateTelemetry,
        angle_command: AngleCommand,
        rate_command: RollRateTestCommand,
        outer_config: AngleOuterLoopConfig | None = None,
    ) -> tuple[AngleOuterLoopOutput, ControllerResult]:
        outer_loop = self._compute_outer_loop(
            telemetry=telemetry,
            angle_command=angle_command,
            outer_config=outer_config,
        )
        controller_result = self.rate_controller.compute(
            p_cmd_rad_s=outer_loop.p_cmd_rad_s,
            q_cmd_rad_s=outer_loop.q_cmd_rad_s,
            p_meas_rad_s=telemetry.p_meas_rad_s,
            q_meas_rad_s=telemetry.q_meas_rad_s,
            kp_p=rate_command.kp_p,
            kp_q=rate_command.kp_q,
            output_limit=rate_command.output_limit,
        )
        return (outer_loop, controller_result)

    def _compute_outer_loop(
        self,
        *,
        telemetry: RollRateTelemetry,
        angle_command: AngleCommand,
        outer_config: AngleOuterLoopConfig | None = None,
    ) -> AngleOuterLoopOutput:
        controller = self.angle_controller if outer_config is None else AngleOuterLoopController(config=outer_config)
        return controller.compute(
            roll_cmd_deg=angle_command.roll_cmd_deg,
            pitch_cmd_deg=angle_command.pitch_cmd_deg,
            roll_meas_deg=telemetry.roll_deg,
            pitch_meas_deg=telemetry.pitch_deg,
        )


@dataclass
class CascadedAltitudeController:
    """Current stage: direct vertical-speed command plus P collective control."""

    altitude_outer_controller: AltitudeOuterLoopController
    vertical_speed_controller: VerticalSpeedPController

    def set_config(self, config: AltitudeControlConfig) -> None:
        self.altitude_outer_controller.config = config
        self.vertical_speed_controller.config = config

    def reset(self) -> None:
        self.vertical_speed_controller.reset()

    def compute(
        self,
        *,
        telemetry: RollRateTelemetry,
        altitude_command: AltitudeCommand,
        config: AltitudeControlConfig | None = None,
    ) -> AltitudeLoopOutput:
        inner_controller = VerticalSpeedPController(config=config) if config is not None else self.vertical_speed_controller
        vz_cmd_m_s = float(altitude_command.alt_cmd_m)
        vz_error_m_s, throttle_correction, throttle_cmd = inner_controller.compute(
            vz_cmd_m_s=vz_cmd_m_s,
            vz_m_s=telemetry.vz_m_s,
            hover_throttle=altitude_command.hover_throttle,
        )

        return AltitudeLoopOutput(
            alt_cmd_m=vz_cmd_m_s,
            alt_m=telemetry.alt_m,
            vz_m_s=telemetry.vz_m_s,
            alt_error_m=0.0,
            vz_cmd_m_s=vz_cmd_m_s,
            vz_error_m_s=vz_error_m_s,
            throttle_correction=throttle_correction,
            throttle_cmd=throttle_cmd,
            integrator_state=0.0,
        )
