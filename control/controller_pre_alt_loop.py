from __future__ import annotations

from dataclasses import dataclass

try:
    from data_api.models import (
        AngleCommand,
        AngleOuterLoopConfig,
        AngleOuterLoopOutput,
        ControllerResult,
        RollRateTelemetry,
        RollRateTestCommand,
    )
except ImportError:
    from ..data_api.models import (
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
    """
    P-only roll/pitch angle outer-loop controller.

    Note:
        This implementation assumes:
        - roll_cmd_deg / pitch_cmd_deg / roll_meas_deg / pitch_meas_deg are in degrees
        - kp_roll_angle / kp_pitch_angle convert degree error directly to rad/s command
          (i.e. unit is approximately rad/s per deg)
    """

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
class CascadedRollPitchController:
    """Own the roll/pitch outer+inner control law used by the runtime."""

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
