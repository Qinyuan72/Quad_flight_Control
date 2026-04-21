from __future__ import annotations

from dataclasses import dataclass

try:
    from control.controller import (
        AltitudeOuterLoopController,
        AngleOuterLoopController,
        BodyVelocityOuterLoopController,
        BodyRatePIDController,
        VerticalSpeedPIDController,
        YawOuterLoopController,
    )
    from data_api.models import (
        AltitudeCommand,
        AltitudeControlConfig,
        AltitudeLoopOutput,
        AngleCommand,
        AngleOuterLoopConfig,
        AngleOuterLoopOutput,
        BodyVelocityCommand,
        BodyVelocityOuterLoopConfig,
        BodyVelocityOuterLoopOutput,
        BodyRateCommand,
        BodyRatePIDConfig,
        ControllerResult,
        RollRateTelemetry,
        VerticalSpeedCommand,
        VerticalSpeedPIDConfig,
        YawOuterLoopConfig,
        YawOuterLoopOutput,
    )
except ImportError:
    from .controller import (
        AltitudeOuterLoopController,
        AngleOuterLoopController,
        BodyVelocityOuterLoopController,
        BodyRatePIDController,
        VerticalSpeedPIDController,
        YawOuterLoopController,
    )
    from ..data_api.models import (
        AltitudeCommand,
        AltitudeControlConfig,
        AltitudeLoopOutput,
        AngleCommand,
        AngleOuterLoopConfig,
        AngleOuterLoopOutput,
        BodyVelocityCommand,
        BodyVelocityOuterLoopConfig,
        BodyVelocityOuterLoopOutput,
        BodyRateCommand,
        BodyRatePIDConfig,
        ControllerResult,
        RollRateTelemetry,
        VerticalSpeedCommand,
        VerticalSpeedPIDConfig,
        YawOuterLoopConfig,
        YawOuterLoopOutput,
    )


@dataclass
class AttitudeControlPipeline:
    """Compose the roll/pitch outer loop with the body-rate inner loop."""

    body_velocity_controller: BodyVelocityOuterLoopController
    angle_controller: AngleOuterLoopController
    yaw_controller: YawOuterLoopController
    rate_controller: BodyRatePIDController

    def reset(self) -> None:
        self.body_velocity_controller.reset()
        self.angle_controller.reset()
        self.rate_controller.reset()

    def set_outer_loop_config(self, config: AngleOuterLoopConfig) -> None:
        self.angle_controller.config = config

    def set_body_velocity_outer_loop_config(self, config: BodyVelocityOuterLoopConfig) -> None:
        self.body_velocity_controller.config = config

    def set_yaw_outer_loop_config(self, config: YawOuterLoopConfig) -> None:
        self.yaw_controller.config = config

    def compute(
        self,
        *,
        telemetry: RollRateTelemetry,
        angle_command: AngleCommand,
        body_velocity_command: BodyVelocityCommand,
        rate_command: BodyRateCommand,
        rate_config: BodyRatePIDConfig,
        body_velocity_outer_config: BodyVelocityOuterLoopConfig | None = None,
        outer_config: AngleOuterLoopConfig | None = None,
        yaw_outer_config: YawOuterLoopConfig | None = None,
        body_velocity_outer_loop_enabled: bool,
        outer_loop_enabled: bool,
    ) -> tuple[BodyVelocityOuterLoopOutput, AngleOuterLoopOutput, YawOuterLoopOutput, ControllerResult]:
        if body_velocity_outer_loop_enabled:
            body_velocity_outer_loop = self._compute_body_velocity_outer_loop(
                telemetry=telemetry,
                body_velocity_command=body_velocity_command,
                body_velocity_outer_config=body_velocity_outer_config,
            )
            active_angle_command = AngleCommand(
                roll_cmd_deg=body_velocity_outer_loop.roll_cmd_from_velocity_deg,
                pitch_cmd_deg=body_velocity_outer_loop.pitch_cmd_from_velocity_deg,
                yaw_cmd_deg=angle_command.yaw_cmd_deg,
            )
        else:
            body_velocity_outer_loop = BodyVelocityOuterLoopOutput()
            active_angle_command = angle_command

        if outer_loop_enabled:
            outer_loop = self._compute_outer_loop(
                telemetry=telemetry,
                angle_command=active_angle_command,
                outer_config=outer_config,
            )
            yaw_outer_loop = self._compute_yaw_outer_loop(
                telemetry=telemetry,
                angle_command=active_angle_command,
                yaw_outer_config=yaw_outer_config,
            )
            active_rate_command = BodyRateCommand(
                p_cmd_rad_s=outer_loop.p_cmd_rad_s,
                q_cmd_rad_s=outer_loop.q_cmd_rad_s,
                r_cmd_rad_s=yaw_outer_loop.r_cmd_rad_s,
            )
        else:
            outer_loop = AngleOuterLoopOutput()
            yaw_outer_loop = YawOuterLoopOutput()
            active_rate_command = rate_command

        controller_result = self.rate_controller.compute(
            p_cmd_rad_s=active_rate_command.p_cmd_rad_s,
            q_cmd_rad_s=active_rate_command.q_cmd_rad_s,
            r_cmd_rad_s=active_rate_command.r_cmd_rad_s,
            p_meas_rad_s=telemetry.p_meas_rad_s,
            q_meas_rad_s=telemetry.q_meas_rad_s,
            r_meas_rad_s=telemetry.r_meas_rad_s,
            kp_p=rate_config.kp_p,
            kp_q=rate_config.kp_q,
            kp_r=rate_config.kp_r,
            ki_p=rate_config.ki_p,
            ki_q=rate_config.ki_q,
            ki_r=rate_config.ki_r,
            kd_p=rate_config.kd_p,
            kd_q=rate_config.kd_q,
            kd_r=rate_config.kd_r,
            integrator_limit_p=rate_config.integrator_limit_p,
            integrator_limit_q=rate_config.integrator_limit_q,
            integrator_limit_r=rate_config.integrator_limit_r,
            dt_s=telemetry.dt_s,
            output_limit=rate_config.output_limit,
        )
        return (body_velocity_outer_loop, outer_loop, yaw_outer_loop, controller_result)

    def _compute_body_velocity_outer_loop(
        self,
        *,
        telemetry: RollRateTelemetry,
        body_velocity_command: BodyVelocityCommand,
        body_velocity_outer_config: BodyVelocityOuterLoopConfig | None = None,
    ) -> BodyVelocityOuterLoopOutput:
        controller = self.body_velocity_controller if body_velocity_outer_config is None else BodyVelocityOuterLoopController(config=body_velocity_outer_config)
        return controller.compute(
            v_forward_cmd_m_s=body_velocity_command.v_forward_cmd_m_s,
            v_right_cmd_m_s=body_velocity_command.v_right_cmd_m_s,
            vx_body_m_s=telemetry.vx_body_m_s,
            vy_body_m_s=telemetry.vy_body_m_s,
        )

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
            roll_rate_rad_s=telemetry.p_meas_rad_s,
            pitch_rate_rad_s=telemetry.q_meas_rad_s,
            dt_s=telemetry.dt_s,
        )

    def _compute_yaw_outer_loop(
        self,
        *,
        telemetry: RollRateTelemetry,
        angle_command: AngleCommand,
        yaw_outer_config: YawOuterLoopConfig | None = None,
    ) -> YawOuterLoopOutput:
        controller = self.yaw_controller if yaw_outer_config is None else YawOuterLoopController(config=yaw_outer_config)
        return controller.compute(
            yaw_cmd_deg=angle_command.yaw_cmd_deg,
            heading_deg=telemetry.heading_deg,
        )


@dataclass
class AltitudeControlPipeline:
    """Compose the altitude outer loop with the vertical-speed inner loop."""

    altitude_outer_controller: AltitudeOuterLoopController
    vertical_speed_controller: VerticalSpeedPIDController

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
        active_config = config if config is not None else self.altitude_outer_controller.config
        outer_controller = self.altitude_outer_controller if config is None else AltitudeOuterLoopController(config=active_config)
        inner_controller = self.vertical_speed_controller if config is None else VerticalSpeedPIDController(config=active_config)

        alt_cmd_m = float(altitude_command.alt_cmd_m)
        alt_error_m, vz_cmd_m_s = outer_controller.compute(
            alt_cmd_m=alt_cmd_m,
            alt_m=telemetry.alt_m,
        )
        vz_command = VerticalSpeedCommand(
            vz_cmd_m_s=vz_cmd_m_s,
            hover_throttle=altitude_command.hover_throttle,
        )
        vz_config = VerticalSpeedPIDConfig(
            kp_vz=active_config.kp_vz,
            ki_vz=active_config.ki_vz,
            kd_vz=active_config.kd_vz,
            vz_integrator_limit=active_config.vz_integrator_limit,
            throttle_min=active_config.throttle_min,
            throttle_max=active_config.throttle_max,
        )
        inner_controller.config = AltitudeControlConfig(
            kp_alt=active_config.kp_alt,
            vz_max=active_config.vz_max,
            kp_vz=vz_config.kp_vz,
            ki_vz=vz_config.ki_vz,
            kd_vz=vz_config.kd_vz,
            vz_integrator_limit=vz_config.vz_integrator_limit,
            throttle_min=vz_config.throttle_min,
            throttle_max=vz_config.throttle_max,
        )
        vz_error_m_s, p_term, i_term, d_term, throttle_correction, throttle_cmd = inner_controller.compute(
            vz_cmd_m_s=vz_command.vz_cmd_m_s,
            vz_m_s=telemetry.vz_m_s,
            dt_s=telemetry.dt_s,
            hover_throttle=vz_command.hover_throttle,
        )

        return AltitudeLoopOutput(
            alt_cmd_m=alt_cmd_m,
            alt_m=telemetry.alt_m,
            vz_m_s=telemetry.vz_m_s,
            alt_error_m=alt_error_m,
            vz_cmd_m_s=vz_cmd_m_s,
            vz_error_m_s=vz_error_m_s,
            p_term=p_term,
            i_term=i_term,
            d_term=d_term,
            throttle_correction=throttle_correction,
            throttle_cmd=throttle_cmd,
            integrator_state=inner_controller.integrator_vz,
        )
