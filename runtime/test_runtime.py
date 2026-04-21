from __future__ import annotations

from dataclasses import replace

try:
    from control.controller import (
        AltitudeOuterLoopController,
        BodyRatePIDController,
        AngleOuterLoopController,
        BodyVelocityOuterLoopController,
        WorldToBodyVelocityProjection,
        VerticalSpeedPIDController,
        YawOuterLoopController,
    )
    from control.horizontal_frame_mapper import UneToControlPlaneMapper
    from control.mixer import MatrixMixer, check_yaw_sign_consistency
    from control.mixer_presets import get_candidate, list_candidates
    from control.pipelines import AltitudeControlPipeline, AttitudeControlPipeline
    from data_api.krpc_bindings import KrpcQuadHardware
    from data_api.models import (
        ActuatorDemand,
        AltitudeCommand,
        AltitudeControlConfig,
        AltitudeLoopOutput,
        AngleCommand,
        AngleOuterLoopConfig,
        AngleOuterLoopOutput,
        BodyVelocityCommand,
        BodyVelocityOuterLoopConfig,
        BodyVelocityOuterLoopOutput,
        ControlPlaneVelocityDebug,
        BodyRateCommand,
        BodyRatePIDConfig,
        ControllerResult,
        MotorCommand,
        RollRateTelemetry,
        RollRateTestCommand,
        RollRateTestState,
        UneToControlPlaneConfig,
        WorldCommandPreprocessConfig,
        WorldToBodyVelocityProjectionOutput,
        WorldVelocityCommand,
        YawOuterLoopConfig,
        YawOuterLoopOutput,
    )
    from data_api.telemetry import RollRateTelemetryReader
except ImportError:
    from ..control.controller import (
        AltitudeOuterLoopController,
        BodyRatePIDController,
        AngleOuterLoopController,
        BodyVelocityOuterLoopController,
        WorldToBodyVelocityProjection,
        VerticalSpeedPIDController,
        YawOuterLoopController,
    )
    from ..control.horizontal_frame_mapper import UneToControlPlaneMapper
    from ..control.mixer import MatrixMixer, check_yaw_sign_consistency
    from ..control.mixer_presets import get_candidate, list_candidates
    from ..control.pipelines import AltitudeControlPipeline, AttitudeControlPipeline
    from ..data_api.krpc_bindings import KrpcQuadHardware
    from ..data_api.models import (
        ActuatorDemand,
        AltitudeCommand,
        AltitudeControlConfig,
        AltitudeLoopOutput,
        AngleCommand,
        AngleOuterLoopConfig,
        AngleOuterLoopOutput,
        BodyVelocityCommand,
        BodyVelocityOuterLoopConfig,
        BodyVelocityOuterLoopOutput,
        ControlPlaneVelocityDebug,
        BodyRateCommand,
        BodyRatePIDConfig,
        ControllerResult,
        MotorCommand,
        RollRateTelemetry,
        RollRateTestCommand,
        RollRateTestState,
        UneToControlPlaneConfig,
        WorldCommandPreprocessConfig,
        WorldToBodyVelocityProjectionOutput,
        WorldVelocityCommand,
        YawOuterLoopConfig,
        YawOuterLoopOutput,
    )
    from ..data_api.telemetry import RollRateTelemetryReader


class RollRateInnerLoopRuntime:
    """Orchestrate one narrow roll/pitch inner-loop test step."""

    def __init__(self) -> None:
        self.hardware = KrpcQuadHardware()
        self.telemetry = RollRateTelemetryReader(self.hardware)
        self.world_velocity_projection = WorldToBodyVelocityProjection()
        self.une_to_control_plane_mapper = UneToControlPlaneMapper()
        self.attitude_pipeline = AttitudeControlPipeline(
            body_velocity_controller=BodyVelocityOuterLoopController(
                config=BodyVelocityOuterLoopConfig(
                    kp_v_forward=0.0,
                    kp_v_right=0.0,
                    velocity_angle_limit_deg=10.0,
                )
            ),
            angle_controller=AngleOuterLoopController(
                config=AngleOuterLoopConfig(
                    kp_roll_angle=0.0,
                    kp_pitch_angle=0.0,
                    ki_roll_angle=0.0,
                    ki_pitch_angle=0.0,
                    kd_roll_angle=0.0,
                    kd_pitch_angle=0.0,
                    integrator_limit_roll=0.0,
                    integrator_limit_pitch=0.0,
                    rate_limit_rad_s=1.0,
                )
            ),
            yaw_controller=YawOuterLoopController(
                config=YawOuterLoopConfig(
                    kp_yaw=0.0,
                    yaw_rate_limit_rad_s=1.0,
                )
            ),
            rate_controller=BodyRatePIDController(),
        )
        self.altitude_pipeline = AltitudeControlPipeline(
            altitude_outer_controller=AltitudeOuterLoopController(
                config=AltitudeControlConfig(
                    kp_alt=0.0,
                    vz_max=0.0,
                    kp_vz=0.0,
                    ki_vz=0.0,
                    kd_vz=0.0,
                    vz_integrator_limit=0.0,
                    throttle_min=0.0,
                    throttle_max=460.0,
                )
            ),
            vertical_speed_controller=VerticalSpeedPIDController(
                config=AltitudeControlConfig(
                    kp_alt=0.0,
                    vz_max=0.0,
                    kp_vz=0.0,
                    ki_vz=0.0,
                    kd_vz=0.0,
                    vz_integrator_limit=0.0,
                    throttle_min=0.0,
                    throttle_max=460.0,
                )
            ),
        )
        self.mixer = MatrixMixer()
        self.command = RollRateTestCommand(
            base_rpm=0.0,
            p_cmd_rad_s=0.0,
            q_cmd_rad_s=0.0,
            r_cmd_rad_s=0.0,
            kp_p=0.0,
            kp_q=0.0,
            kp_r=0.0,
            ki_p=0.0,
            ki_q=0.0,
            ki_r=0.0,
            kd_p=0.0,
            kd_q=0.0,
            kd_r=0.0,
            integrator_limit_p=0.0,
            integrator_limit_q=0.0,
            integrator_limit_r=0.0,
            output_limit=0.0,
        )
        self.body_rate_command, self.body_rate_config = self._split_legacy_rate_command(self.command)
        self.angle_command = AngleCommand()
        self.body_velocity_command = BodyVelocityCommand()
        self.world_velocity_command = WorldVelocityCommand()
        self.world_command_preprocess_config = WorldCommandPreprocessConfig()
        self.une_to_control_plane_config = UneToControlPlaneConfig()
        self.une_to_control_plane_mapper.config = self.une_to_control_plane_config
        self.world_velocity_feedback_mode = "raw_body"
        self.body_velocity_outer_loop_config = BodyVelocityOuterLoopConfig(
            kp_v_forward=0.0,
            kp_v_right=0.0,
            velocity_angle_limit_deg=10.0,
        )
        self.yaw_outer_loop_config = YawOuterLoopConfig(
            kp_yaw=0.0,
            yaw_rate_limit_rad_s=1.0,
        )
        self.altitude_command = AltitudeCommand()
        self.altitude_config = AltitudeControlConfig(
            kp_alt=0.0,
            vz_max=0.0,
            kp_vz=0.0,
            ki_vz=0.0,
            kd_vz=0.0,
            vz_integrator_limit=0.0,
            throttle_min=0.0,
            throttle_max=460.0,
        )
        self.current_mixer = get_candidate(list_candidates()[0])
        self.test_running = False
        self.outer_loop_running = False
        self.body_velocity_outer_loop_running = False
        self.body_velocity_outer_loop_source = "body"
        self.altitude_loop_running = False
        self.last_snapshot = RollRateTestState(
            command=self.command,
            angle_command=self.angle_command,
            body_velocity_command=self.body_velocity_command,
            world_velocity_command=self.world_velocity_command,
            world_command_preprocess_config=self.world_command_preprocess_config,
            une_to_control_plane_config=self.une_to_control_plane_config,
            world_velocity_feedback_mode=self.world_velocity_feedback_mode,
            outer_loop_config=self.attitude_pipeline.angle_controller.config,
            yaw_outer_loop_config=self.yaw_outer_loop_config,
            body_velocity_outer_loop_config=self.body_velocity_outer_loop_config,
            body_velocity_outer_loop_enabled=self.body_velocity_outer_loop_running,
            altitude_command=self.altitude_command,
            altitude_control_config=self.altitude_config,
            mixer_name=self.current_mixer.name,
            mixer_matrix_text=self.mixer.format_matrix_text(self.current_mixer),
        )

    def connect(self) -> None:
        self.hardware.connect()
        self.telemetry.connect()
        self._refresh_live_snapshot(status="Connected")

    def disconnect(self) -> None:
        self.test_running = False
        self.outer_loop_running = False
        self.body_velocity_outer_loop_running = False
        self.altitude_loop_running = False
        self.telemetry.disconnect()
        self.hardware.disconnect()
        self.attitude_pipeline.reset()
        self.altitude_pipeline.reset()
        self._refresh_snapshot(status="Disconnected")

    def bind(self) -> None:
        self.hardware.bind()
        self._refresh_snapshot(status="Bound exact controller and rotor tags")

    def initialize(self) -> None:
        self.hardware.initialize()
        self._refresh_snapshot(status="Initialized rotors and KAL controllers")

    def start_test(self) -> None:
        if not self.hardware.is_bound():
            raise RuntimeError("Bind tags before starting the test loop.")
        if not self.hardware.initialized:
            raise RuntimeError("Initialize hardware before starting the test loop.")
        self.attitude_pipeline.reset()
        if self.altitude_loop_running:
            self.altitude_pipeline.reset()
        self.test_running = True
        self._refresh_snapshot(status="Test loop started")

    def stop_test(self) -> None:
        self.test_running = False
        self.attitude_pipeline.reset()
        self.altitude_pipeline.reset()
        if self.hardware.is_bound():
            self.hardware.zero_outputs()
        self._refresh_snapshot(status="Test loop stopped")

    def emergency_stop(self) -> None:
        self.test_running = False
        self.outer_loop_running = False
        self.body_velocity_outer_loop_running = False
        self.altitude_loop_running = False
        self.attitude_pipeline.reset()
        self.altitude_pipeline.reset()
        self.hardware.emergency_stop()
        self._refresh_snapshot(status="Emergency stop issued")

    def set_command(self, command: RollRateTestCommand) -> None:
        self.command = command
        self.body_rate_command, self.body_rate_config = self._split_legacy_rate_command(command)
        self._refresh_live_snapshot(status="Inner-loop parameters updated")

    def set_angle_command(self, angle_command: AngleCommand) -> None:
        self.angle_command = angle_command
        self._refresh_live_snapshot(status="Outer-loop targets updated")

    def set_outer_loop_config(self, config: AngleOuterLoopConfig) -> None:
        self.attitude_pipeline.set_outer_loop_config(config)
        self._refresh_live_snapshot(status="Outer-loop gains updated")

    def set_body_velocity_command(self, command: BodyVelocityCommand) -> None:
        self.body_velocity_command = command
        self._refresh_live_snapshot(status="Body-velocity targets updated")

    def set_world_velocity_command(self, command: WorldVelocityCommand) -> None:
        self.world_velocity_command = command
        self._refresh_live_snapshot(status="World-velocity debug targets updated")

    def set_world_command_preprocess_config(self, config: WorldCommandPreprocessConfig) -> None:
        self.world_command_preprocess_config = config
        self._refresh_live_snapshot(status="World-command sign preprocessing updated")

    def set_une_to_control_plane_config(self, config: UneToControlPlaneConfig) -> None:
        self.une_to_control_plane_config = config
        self.une_to_control_plane_mapper.config = config
        self._refresh_live_snapshot(status="UNE-to-control-plane mapping updated")

    def set_world_velocity_feedback_mode(self, mode: str) -> None:
        normalized_mode = str(mode).strip().lower()
        allowed_modes = {"projected_une", "raw_body"}
        if normalized_mode not in allowed_modes:
            raise ValueError(f"Unsupported world velocity feedback mode: {mode}")
        self.world_velocity_feedback_mode = normalized_mode
        self._refresh_live_snapshot(status="World-velocity feedback mode updated")

    def set_body_velocity_outer_loop_config(self, config: BodyVelocityOuterLoopConfig) -> None:
        self.body_velocity_outer_loop_config = config
        self.attitude_pipeline.set_body_velocity_outer_loop_config(config)
        self._refresh_live_snapshot(status="Body-velocity outer-loop gains updated")

    def set_yaw_outer_loop_config(self, config: YawOuterLoopConfig) -> None:
        self.yaw_outer_loop_config = config
        self.attitude_pipeline.set_yaw_outer_loop_config(config)
        self._refresh_live_snapshot(status="Yaw outer-loop gains updated")

    def start_outer_loop(self) -> None:
        self.attitude_pipeline.angle_controller.reset()
        self.outer_loop_running = True
        self._refresh_live_snapshot(status="Outer loop enabled")

    def stop_outer_loop(self) -> None:
        self.outer_loop_running = False
        self.attitude_pipeline.angle_controller.reset()
        self._refresh_live_snapshot(status="Outer loop disabled")

    def start_body_velocity_outer_loop(self) -> None:
        self.attitude_pipeline.body_velocity_controller.reset()
        self.outer_loop_running = True
        self.body_velocity_outer_loop_running = True
        self.body_velocity_outer_loop_source = "body"
        self._refresh_live_snapshot(status="Body-velocity outer loop enabled")

    def stop_body_velocity_outer_loop(self) -> None:
        self.body_velocity_outer_loop_running = False
        self.attitude_pipeline.body_velocity_controller.reset()
        self._refresh_live_snapshot(status="Body-velocity outer loop disabled")

    def start_world_velocity_control(self) -> None:
        self.attitude_pipeline.body_velocity_controller.reset()
        self.outer_loop_running = True
        self.body_velocity_outer_loop_running = True
        self.body_velocity_outer_loop_source = "world"
        self._refresh_live_snapshot(status="World-velocity control enabled")

    def stop_world_velocity_control(self) -> None:
        self.stop_body_velocity_outer_loop()

    def set_altitude_command(self, altitude_command: AltitudeCommand) -> None:
        self.altitude_command = altitude_command
        self._refresh_live_snapshot(status="Altitude target updated")

    def set_altitude_config(self, config: AltitudeControlConfig) -> None:
        self.altitude_config = config
        self.altitude_pipeline.set_config(config)
        self._refresh_live_snapshot(status="Altitude-loop gains updated")

    def start_altitude_loop(self) -> None:
        self.altitude_pipeline.reset()
        self.altitude_loop_running = True
        self._refresh_live_snapshot(status="Altitude loop enabled")

    def stop_altitude_loop(self) -> None:
        self.altitude_loop_running = False
        self.altitude_pipeline.reset()
        self._refresh_live_snapshot(status="Altitude loop disabled")

    def set_mixer_candidate(self, name: str) -> None:
        self.current_mixer = get_candidate(name)
        self._refresh_snapshot(status=f"Mixer candidate set to {name}")

    def get_mixer_names(self) -> list[str]:
        return list_candidates()

    def step_once(self) -> RollRateTestState:
        return self._run_cycle(write_outputs=True, status="Step executed")

    def preview_once(self) -> RollRateTestState:
        return self._run_cycle(write_outputs=False, status="Feedback preview updated")

    def step(self) -> RollRateTestState:
        return self.step_once()

    def preview(self) -> RollRateTestState:
        return self.preview_once()

    def record_background_error(self, message: str) -> RollRateTestState:
        return self._refresh_snapshot(status=f"Loop error: {message}", last_error=message)

    def _refresh_live_snapshot(self, *, status: str) -> RollRateTestState:
        if self.hardware.is_connected() and self.telemetry.streams:
            return self._run_cycle(write_outputs=False, status=status)
        return self._refresh_snapshot(status=status)

    def _run_cycle(self, *, write_outputs: bool, status: str) -> RollRateTestState:
        telemetry = self.telemetry.read()
        world_velocity_projection = self._compute_world_velocity_projection(telemetry)
        translated_velocity_command, translated_velocity_feedback, control_plane_velocity_debug = self._translate_world_velocity_for_control_plane(telemetry)
        active_velocity_command, active_velocity_feedback = self._select_active_velocity_outer_loop_inputs(
            telemetry=telemetry,
            translated_velocity_command=translated_velocity_command,
            translated_velocity_feedback=translated_velocity_feedback,
        )
        altitude_loop, collective_cmd = self._compute_altitude_channel(telemetry)
        body_velocity_outer_loop, outer_loop, yaw_outer_loop, controller_result = self._compute_attitude_channel(
            telemetry=active_velocity_feedback,
            active_body_velocity_command=active_velocity_command,
        )
        actuator_demand = ActuatorDemand(
            base_rpm=collective_cmd,
            u_roll=controller_result.u_roll,
            u_pitch=controller_result.u_pitch,
            u_yaw=controller_result.u_yaw,
        )
        motor_command = self.mixer.mix(
            candidate=self.current_mixer,
            base_rpm=actuator_demand.base_rpm,
            u_roll=actuator_demand.u_roll,
            u_pitch=actuator_demand.u_pitch,
            u_yaw=actuator_demand.u_yaw,
        )
        if write_outputs:
            self.hardware.write_motor_command(motor_command)
        return self._refresh_snapshot(
            telemetry=telemetry,
            world_velocity_projection=world_velocity_projection,
            control_plane_velocity_debug=control_plane_velocity_debug,
            body_velocity_outer_loop=body_velocity_outer_loop,
            outer_loop=outer_loop,
            yaw_outer_loop=yaw_outer_loop,
            altitude_loop=altitude_loop,
            controller=controller_result,
            motor_command=motor_command,
            status=status,
        )

    def _compute_attitude_channel(
        self,
        *,
        telemetry: RollRateTelemetry,
        active_body_velocity_command: BodyVelocityCommand,
    ) -> tuple[BodyVelocityOuterLoopOutput, AngleOuterLoopOutput, YawOuterLoopOutput, ControllerResult]:
        return self.attitude_pipeline.compute(
            telemetry=telemetry,
            angle_command=self.angle_command,
            body_velocity_command=active_body_velocity_command,
            rate_command=self.body_rate_command,
            rate_config=self.body_rate_config,
            body_velocity_outer_config=self.body_velocity_outer_loop_config,
            yaw_outer_config=self.yaw_outer_loop_config,
            body_velocity_outer_loop_enabled=self.body_velocity_outer_loop_running,
            outer_loop_enabled=self.outer_loop_running,
        )

    def _compute_altitude_channel(self, telemetry: RollRateTelemetry) -> tuple[AltitudeLoopOutput, float]:
        altitude_loop = self.altitude_pipeline.compute(
            telemetry=telemetry,
            altitude_command=self.altitude_command,
        )
        collective_cmd = altitude_loop.throttle_cmd if self.altitude_loop_running else self.command.base_rpm
        return (altitude_loop, collective_cmd)

    def _compute_world_velocity_projection(self, telemetry: RollRateTelemetry) -> WorldToBodyVelocityProjectionOutput:
        preprocessed_north_m_s, preprocessed_east_m_s = self._preprocess_world_velocity_command()
        return self.world_velocity_projection.compute(
            v_north_cmd_m_s=preprocessed_north_m_s,
            v_east_cmd_m_s=preprocessed_east_m_s,
            heading_deg=telemetry.heading_deg,
        )

    def _preprocess_world_velocity_command(self) -> tuple[float, float]:
        north_cmd_m_s = self.world_velocity_command.v_north_cmd_m_s
        east_cmd_m_s = self.world_velocity_command.v_east_cmd_m_s
        if self.world_command_preprocess_config.invert_world_north:
            north_cmd_m_s = -north_cmd_m_s
        if self.world_command_preprocess_config.invert_world_east:
            east_cmd_m_s = -east_cmd_m_s
        return (north_cmd_m_s, east_cmd_m_s)

    def _translate_world_velocity_command_for_control_plane(
        self,
        telemetry: RollRateTelemetry,
    ) -> tuple[BodyVelocityCommand, float, float, WorldToBodyVelocityProjectionOutput]:
        preprocessed_north_m_s, preprocessed_east_m_s = self._preprocess_world_velocity_command()
        projected_command = self.world_velocity_projection.compute(
            v_north_cmd_m_s=preprocessed_north_m_s,
            v_east_cmd_m_s=preprocessed_east_m_s,
            heading_deg=telemetry.heading_deg,
        )
        ctrl_x_cmd_m_s, ctrl_y_cmd_m_s = self.une_to_control_plane_mapper.map_horizontal(
            projected_command.v_forward_cmd_from_world_m_s,
            projected_command.v_right_cmd_from_world_m_s,
        )
        return (
            BodyVelocityCommand(
                v_forward_cmd_m_s=ctrl_x_cmd_m_s,
                v_right_cmd_m_s=ctrl_y_cmd_m_s,
            ),
            preprocessed_north_m_s,
            preprocessed_east_m_s,
            projected_command,
        )

    def _translate_projected_une_world_feedback(
        self,
        telemetry: RollRateTelemetry,
    ) -> tuple[float, float]:
        projected_feedback = self.world_velocity_projection.compute(
            v_north_cmd_m_s=telemetry.v_north_une_m_s,
            v_east_cmd_m_s=telemetry.v_east_une_m_s,
            heading_deg=telemetry.heading_deg,
        )
        return self.une_to_control_plane_mapper.map_horizontal(
            projected_feedback.v_forward_cmd_from_world_m_s,
            projected_feedback.v_right_cmd_from_world_m_s,
        )

    def _translate_world_velocity_for_control_plane(
        self,
        telemetry: RollRateTelemetry,
    ) -> tuple[BodyVelocityCommand, RollRateTelemetry, ControlPlaneVelocityDebug]:
        (
            translated_command,
            preprocessed_north_m_s,
            preprocessed_east_m_s,
            projected_command,
        ) = self._translate_world_velocity_command_for_control_plane(telemetry)
        projected_une_meas_x_m_s, projected_une_meas_y_m_s = self._translate_projected_une_world_feedback(telemetry)
        raw_body_meas_x_m_s = telemetry.vx_body_m_s
        raw_body_meas_y_m_s = telemetry.vy_body_m_s
        if self.world_velocity_feedback_mode == "raw_body":
            active_meas_x_m_s = raw_body_meas_x_m_s
            active_meas_y_m_s = raw_body_meas_y_m_s
        else:
            active_meas_x_m_s = projected_une_meas_x_m_s
            active_meas_y_m_s = projected_une_meas_y_m_s
        translated_feedback = replace(
            telemetry,
            # Compatibility shim: the legacy body-velocity outer loop still reads
            # vx_body/vy_body fields, but runtime now loads them with translated
            # control-plane measurements instead of raw body-frame velocity.
            vx_body_m_s=active_meas_x_m_s,
            vy_body_m_s=active_meas_y_m_s,
        )
        debug = ControlPlaneVelocityDebug(
            v_north_cmd_raw_m_s=self.world_velocity_command.v_north_cmd_m_s,
            v_east_cmd_raw_m_s=self.world_velocity_command.v_east_cmd_m_s,
            v_north_cmd_preprocessed_m_s=preprocessed_north_m_s,
            v_east_cmd_preprocessed_m_s=preprocessed_east_m_s,
            v_forward_cmd_projected_m_s=projected_command.v_forward_cmd_from_world_m_s,
            v_right_cmd_projected_m_s=projected_command.v_right_cmd_from_world_m_s,
            vx_control_plane_cmd_m_s=translated_command.v_forward_cmd_m_s,
            vy_control_plane_cmd_m_s=translated_command.v_right_cmd_m_s,
            vx_control_plane_meas_m_s=active_meas_x_m_s,
            vy_control_plane_meas_m_s=active_meas_y_m_s,
            vx_projected_une_meas_m_s=projected_une_meas_x_m_s,
            vy_projected_une_meas_m_s=projected_une_meas_y_m_s,
            vx_raw_body_meas_m_s=raw_body_meas_x_m_s,
            vy_raw_body_meas_m_s=raw_body_meas_y_m_s,
            world_feedback_mode=self.world_velocity_feedback_mode,
        )
        return (translated_command, translated_feedback, debug)

    def _select_active_velocity_outer_loop_inputs(
        self,
        *,
        telemetry: RollRateTelemetry,
        translated_velocity_command: BodyVelocityCommand,
        translated_velocity_feedback: RollRateTelemetry,
    ) -> tuple[BodyVelocityCommand, RollRateTelemetry]:
        if self.body_velocity_outer_loop_source == "world":
            return (translated_velocity_command, translated_velocity_feedback)
        return (self.body_velocity_command, telemetry)

    def _split_legacy_rate_command(self, command: RollRateTestCommand) -> tuple[BodyRateCommand, BodyRatePIDConfig]:
        return (
            BodyRateCommand(
                p_cmd_rad_s=command.p_cmd_rad_s,
                q_cmd_rad_s=command.q_cmd_rad_s,
                r_cmd_rad_s=command.r_cmd_rad_s,
            ),
            BodyRatePIDConfig(
                kp_p=command.kp_p,
                kp_q=command.kp_q,
                kp_r=command.kp_r,
                ki_p=command.ki_p,
                ki_q=command.ki_q,
                ki_r=command.ki_r,
                kd_p=command.kd_p,
                kd_q=command.kd_q,
                kd_r=command.kd_r,
                integrator_limit_p=command.integrator_limit_p,
                integrator_limit_q=command.integrator_limit_q,
                integrator_limit_r=command.integrator_limit_r,
                output_limit=command.output_limit,
            ),
        )

    def run_outer_loop_self_check(self) -> dict[str, object]:
        test_command = replace(
            self.command,
            base_rpm=max(0.0, self.command.base_rpm),
            kp_p=1.0 if self.command.kp_p == 0.0 else self.command.kp_p,
            kp_q=1.0 if self.command.kp_q == 0.0 else self.command.kp_q,
            kp_r=1.0 if self.command.kp_r == 0.0 else self.command.kp_r,
            ki_p=0.0,
            ki_q=0.0,
            ki_r=0.0,
            kd_p=0.0,
            kd_q=0.0,
            kd_r=0.0,
            integrator_limit_p=0.0,
            integrator_limit_q=0.0,
            integrator_limit_r=0.0,
            output_limit=10.0 if self.command.output_limit <= 0.0 else self.command.output_limit,
        )
        test_config = AngleOuterLoopConfig(
            kp_roll_angle=0.10,
            kp_pitch_angle=0.10,
            ki_roll_angle=0.0,
            ki_pitch_angle=0.0,
            kd_roll_angle=0.0,
            kd_pitch_angle=0.0,
            integrator_limit_roll=0.0,
            integrator_limit_pitch=0.0,
            rate_limit_rad_s=2.0,
        )
        test_yaw_config = YawOuterLoopConfig(
            kp_yaw=1.0,
            yaw_rate_limit_rad_s=2.0,
        )
        test_rate_command, test_rate_config = self._split_legacy_rate_command(test_command)

        case_roll = self.attitude_pipeline.compute(
            telemetry=RollRateTelemetry(roll_deg=0.0, pitch_deg=0.0, p_meas_rad_s=0.0, q_meas_rad_s=0.0),
            angle_command=AngleCommand(roll_cmd_deg=5.0, pitch_cmd_deg=0.0, yaw_cmd_deg=0.0),
            body_velocity_command=BodyVelocityCommand(),
            rate_command=test_rate_command,
            rate_config=test_rate_config,
            body_velocity_outer_loop_enabled=False,
            outer_config=test_config,
            yaw_outer_config=test_yaw_config,
            outer_loop_enabled=True,
        )[1]
        case_roll_near = self.attitude_pipeline.compute(
            telemetry=RollRateTelemetry(roll_deg=4.0, pitch_deg=0.0, p_meas_rad_s=0.0, q_meas_rad_s=0.0),
            angle_command=AngleCommand(roll_cmd_deg=5.0, pitch_cmd_deg=0.0, yaw_cmd_deg=0.0),
            body_velocity_command=BodyVelocityCommand(),
            rate_command=test_rate_command,
            rate_config=test_rate_config,
            body_velocity_outer_loop_enabled=False,
            outer_config=test_config,
            yaw_outer_config=test_yaw_config,
            outer_loop_enabled=True,
        )[1]
        case_pitch = self.attitude_pipeline.compute(
            telemetry=RollRateTelemetry(roll_deg=0.0, pitch_deg=0.0, p_meas_rad_s=0.0, q_meas_rad_s=0.0),
            angle_command=AngleCommand(roll_cmd_deg=0.0, pitch_cmd_deg=5.0, yaw_cmd_deg=0.0),
            body_velocity_command=BodyVelocityCommand(),
            rate_command=test_rate_command,
            rate_config=test_rate_config,
            body_velocity_outer_loop_enabled=False,
            outer_config=test_config,
            yaw_outer_config=test_yaw_config,
            outer_loop_enabled=True,
        )[1]
        damped_config = AngleOuterLoopConfig(
            kp_roll_angle=0.10,
            kp_pitch_angle=0.10,
            ki_roll_angle=0.0,
            ki_pitch_angle=0.0,
            kd_roll_angle=0.5,
            kd_pitch_angle=0.0,
            integrator_limit_roll=0.0,
            integrator_limit_pitch=0.0,
            rate_limit_rad_s=2.0,
        )
        damped_case = AngleOuterLoopController(config=damped_config).compute(
            roll_cmd_deg=5.0,
            pitch_cmd_deg=0.0,
            roll_meas_deg=0.0,
            pitch_meas_deg=0.0,
            roll_rate_rad_s=0.2,
            pitch_rate_rad_s=0.0,
            dt_s=0.1,
        )
        integral_config = AngleOuterLoopConfig(
            kp_roll_angle=0.10,
            kp_pitch_angle=0.10,
            ki_roll_angle=0.20,
            ki_pitch_angle=0.0,
            kd_roll_angle=0.0,
            kd_pitch_angle=0.0,
            integrator_limit_roll=1.0,
            integrator_limit_pitch=0.0,
            rate_limit_rad_s=2.0,
        )
        integral_controller = AngleOuterLoopController(config=integral_config)
        integral_first = integral_controller.compute(
            roll_cmd_deg=5.0,
            pitch_cmd_deg=0.0,
            roll_meas_deg=0.0,
            pitch_meas_deg=0.0,
            roll_rate_rad_s=0.0,
            pitch_rate_rad_s=0.0,
            dt_s=0.1,
        )
        integral_second = integral_controller.compute(
            roll_cmd_deg=5.0,
            pitch_cmd_deg=0.0,
            roll_meas_deg=0.0,
            pitch_meas_deg=0.0,
            roll_rate_rad_s=0.0,
            pitch_rate_rad_s=0.0,
            dt_s=0.1,
        )
        integral_controller.reset()
        integral_after_reset = integral_controller.compute(
            roll_cmd_deg=5.0,
            pitch_cmd_deg=0.0,
            roll_meas_deg=0.0,
            pitch_meas_deg=0.0,
            roll_rate_rad_s=0.0,
            pitch_rate_rad_s=0.0,
            dt_s=0.1,
        )

        return {
            "case_roll": {
                "roll_error_deg": case_roll.roll_error_deg,
                "p_cmd_from_angle": case_roll.p_cmd_rad_s,
                "q_cmd_from_angle": case_roll.q_cmd_rad_s,
                "pass": case_roll.roll_error_deg > 0.0 and case_roll.p_cmd_rad_s > 0.0 and abs(case_roll.q_cmd_rad_s) < 1e-9,
            },
            "case_roll_near_target": {
                "roll_error_deg": case_roll_near.roll_error_deg,
                "p_cmd_from_angle": case_roll_near.p_cmd_rad_s,
                "pass": case_roll_near.roll_error_deg < case_roll.roll_error_deg and case_roll_near.p_cmd_rad_s < case_roll.p_cmd_rad_s,
            },
            "case_pitch": {
                "pitch_error_deg": case_pitch.pitch_error_deg,
                "p_cmd_from_angle": case_pitch.p_cmd_rad_s,
                "q_cmd_from_angle": case_pitch.q_cmd_rad_s,
                "pass": case_pitch.q_cmd_rad_s > 0.0 and abs(case_pitch.p_cmd_rad_s) < 1e-9,
            },
            "case_roll_damped": {
                "p_cmd_from_angle": damped_case.p_cmd_rad_s,
                "pass": damped_case.p_cmd_rad_s < case_roll.p_cmd_rad_s,
            },
            "case_roll_integral_reset": {
                "first_cmd": integral_first.p_cmd_rad_s,
                "second_cmd": integral_second.p_cmd_rad_s,
                "after_reset_cmd": integral_after_reset.p_cmd_rad_s,
                "pass": integral_second.p_cmd_rad_s > integral_first.p_cmd_rad_s and abs(integral_after_reset.p_cmd_rad_s - integral_first.p_cmd_rad_s) < 1e-9,
            },
        }

    def run_body_velocity_outer_loop_self_check(self) -> dict[str, object]:
        test_config = BodyVelocityOuterLoopConfig(
            kp_v_forward=2.0,
            kp_v_right=3.0,
            velocity_angle_limit_deg=8.0,
        )
        test_command = BodyVelocityCommand(
            v_forward_cmd_m_s=1.0,
            v_right_cmd_m_s=0.5,
        )
        controller = BodyVelocityOuterLoopController(config=test_config)
        case_positive = controller.compute(
            v_forward_cmd_m_s=test_command.v_forward_cmd_m_s,
            v_right_cmd_m_s=test_command.v_right_cmd_m_s,
            vx_body_m_s=0.0,
            vy_body_m_s=0.0,
        )
        case_limited = controller.compute(
            v_forward_cmd_m_s=10.0,
            v_right_cmd_m_s=10.0,
            vx_body_m_s=0.0,
            vy_body_m_s=0.0,
        )
        case_zero_error = controller.compute(
            v_forward_cmd_m_s=1.0,
            v_right_cmd_m_s=-2.0,
            vx_body_m_s=1.0,
            vy_body_m_s=-2.0,
        )
        runtime_case = self.attitude_pipeline.compute(
            telemetry=RollRateTelemetry(vx_body_m_s=0.0, vy_body_m_s=0.0, p_meas_rad_s=0.0, q_meas_rad_s=0.0),
            angle_command=AngleCommand(roll_cmd_deg=0.0, pitch_cmd_deg=0.0, yaw_cmd_deg=0.0),
            body_velocity_command=test_command,
            rate_command=BodyRateCommand(),
            rate_config=BodyRatePIDConfig(output_limit=10.0),
            body_velocity_outer_config=test_config,
            outer_config=AngleOuterLoopConfig(
                kp_roll_angle=0.10,
                kp_pitch_angle=0.10,
                rate_limit_rad_s=2.0,
            ),
            yaw_outer_config=YawOuterLoopConfig(),
            body_velocity_outer_loop_enabled=True,
            outer_loop_enabled=True,
        )
        velocity_output = runtime_case[0]
        attitude_output = runtime_case[1]
        return {
            "case_positive": {
                "pitch_cmd_from_velocity_deg": case_positive.pitch_cmd_from_velocity_deg,
                "roll_cmd_from_velocity_deg": case_positive.roll_cmd_from_velocity_deg,
                "pass": case_positive.pitch_cmd_from_velocity_deg > 0.0 and case_positive.roll_cmd_from_velocity_deg > 0.0,
            },
            "case_limited": {
                "pitch_cmd_from_velocity_deg": case_limited.pitch_cmd_from_velocity_deg,
                "roll_cmd_from_velocity_deg": case_limited.roll_cmd_from_velocity_deg,
                "pass": abs(case_limited.pitch_cmd_from_velocity_deg) <= test_config.velocity_angle_limit_deg + 1e-9 and abs(case_limited.roll_cmd_from_velocity_deg) <= test_config.velocity_angle_limit_deg + 1e-9,
            },
            "case_zero_error": {
                "pitch_cmd_from_velocity_deg": case_zero_error.pitch_cmd_from_velocity_deg,
                "roll_cmd_from_velocity_deg": case_zero_error.roll_cmd_from_velocity_deg,
                "pass": abs(case_zero_error.pitch_cmd_from_velocity_deg) < 1e-9 and abs(case_zero_error.roll_cmd_from_velocity_deg) < 1e-9,
            },
            "runtime_wiring": {
                "velocity_pitch_cmd_deg": velocity_output.pitch_cmd_from_velocity_deg,
                "velocity_roll_cmd_deg": velocity_output.roll_cmd_from_velocity_deg,
                "p_cmd_rad_s": attitude_output.p_cmd_rad_s,
                "q_cmd_rad_s": attitude_output.q_cmd_rad_s,
                "pass": abs(velocity_output.pitch_cmd_from_velocity_deg) > 0.0 and abs(velocity_output.roll_cmd_from_velocity_deg) > 0.0 and abs(attitude_output.p_cmd_rad_s) > 0.0 and abs(attitude_output.q_cmd_rad_s) > 0.0,
            },
        }

    def run_une_to_control_plane_mapper_self_check(self) -> dict[str, object]:
        default_mapper = UneToControlPlaneMapper(config=UneToControlPlaneConfig())
        flipped_x_mapper = UneToControlPlaneMapper(config=UneToControlPlaneConfig(sign_x=-1.0))
        flipped_y_mapper = UneToControlPlaneMapper(config=UneToControlPlaneConfig(sign_y=-1.0))
        swapped_mapper = UneToControlPlaneMapper(config=UneToControlPlaneConfig(swap_ne=True))
        combined_mapper = UneToControlPlaneMapper(
            config=UneToControlPlaneConfig(swap_ne=True, sign_x=-1.0, sign_y=1.0)
        )
        default_case = default_mapper.map_horizontal(2.0, 3.0)
        flipped_x_case = flipped_x_mapper.map_horizontal(2.0, 3.0)
        flipped_y_case = flipped_y_mapper.map_horizontal(2.0, 3.0)
        swapped_case = swapped_mapper.map_horizontal(2.0, 3.0)
        combined_case = combined_mapper.map_horizontal(2.0, 3.0)
        return {
            "default": {"x": default_case[0], "y": default_case[1], "pass": default_case == (2.0, 3.0)},
            "sign_x": {"x": flipped_x_case[0], "y": flipped_x_case[1], "pass": flipped_x_case == (-2.0, 3.0)},
            "sign_y": {"x": flipped_y_case[0], "y": flipped_y_case[1], "pass": flipped_y_case == (2.0, -3.0)},
            "swap_ne": {"x": swapped_case[0], "y": swapped_case[1], "pass": swapped_case == (3.0, 2.0)},
            "combined": {"x": combined_case[0], "y": combined_case[1], "pass": combined_case == (-3.0, 2.0)},
        }

    def run_world_velocity_translation_chain_self_check(self) -> dict[str, object]:
        original_config = self.une_to_control_plane_config
        original_preprocess_config = self.world_command_preprocess_config
        original_mode = self.world_velocity_feedback_mode
        try:
            self.set_une_to_control_plane_config(
                UneToControlPlaneConfig(swap_ne=True, sign_x=-1.0, sign_y=1.0)
            )
            self.set_world_command_preprocess_config(WorldCommandPreprocessConfig())
            self.set_world_velocity_feedback_mode("projected_une")
            self.set_world_velocity_command(WorldVelocityCommand(v_north_cmd_m_s=2.0, v_east_cmd_m_s=3.0))
            telemetry_heading_0 = RollRateTelemetry(
                v_north_une_m_s=4.0,
                v_east_une_m_s=5.0,
                heading_deg=0.0,
                p_meas_rad_s=0.0,
                q_meas_rad_s=0.0,
            )
            command_heading_0, feedback_heading_0, debug_heading_0 = self._translate_world_velocity_for_control_plane(
                telemetry_heading_0
            )
            telemetry_heading_90 = RollRateTelemetry(
                v_north_une_m_s=4.0,
                v_east_une_m_s=5.0,
                heading_deg=90.0,
                p_meas_rad_s=0.0,
                q_meas_rad_s=0.0,
            )
            translated_command, translated_feedback, debug = self._translate_world_velocity_for_control_plane(telemetry_heading_90)
            runtime_case = self.attitude_pipeline.compute(
                telemetry=translated_feedback,
                angle_command=AngleCommand(),
                body_velocity_command=translated_command,
                rate_command=BodyRateCommand(),
                rate_config=BodyRatePIDConfig(output_limit=10.0),
                body_velocity_outer_config=BodyVelocityOuterLoopConfig(
                    kp_v_forward=1.0,
                    kp_v_right=1.0,
                    velocity_angle_limit_deg=20.0,
                ),
                outer_config=AngleOuterLoopConfig(
                    kp_roll_angle=0.10,
                    kp_pitch_angle=0.10,
                    rate_limit_rad_s=2.0,
                ),
                yaw_outer_config=YawOuterLoopConfig(),
                body_velocity_outer_loop_enabled=True,
                outer_loop_enabled=True,
            )
            snapshot = self._refresh_snapshot(
                telemetry=telemetry_heading_90,
                control_plane_velocity_debug=debug,
                body_velocity_outer_loop=runtime_case[0],
                outer_loop=runtime_case[1],
                yaw_outer_loop=runtime_case[2],
                controller=runtime_case[3],
                status="Translation chain self-check",
            )
            return {
                "heading_0": {
                    "cmd_x": command_heading_0.v_forward_cmd_m_s,
                    "cmd_y": command_heading_0.v_right_cmd_m_s,
                    "meas_x": feedback_heading_0.vx_body_m_s,
                    "meas_y": feedback_heading_0.vy_body_m_s,
                    "pass": (
                        abs(command_heading_0.v_forward_cmd_m_s - (-3.0)) < 1e-9
                        and abs(command_heading_0.v_right_cmd_m_s - 2.0) < 1e-9
                        and abs(feedback_heading_0.vx_body_m_s - (-5.0)) < 1e-9
                        and abs(feedback_heading_0.vy_body_m_s - 4.0) < 1e-9
                        and abs(debug_heading_0.vx_control_plane_cmd_m_s - (-3.0)) < 1e-9
                        and abs(debug_heading_0.vy_control_plane_cmd_m_s - 2.0) < 1e-9
                    ),
                },
                "heading_90": {
                    "cmd_x": translated_command.v_forward_cmd_m_s,
                    "cmd_y": translated_command.v_right_cmd_m_s,
                    "meas_x": translated_feedback.vx_body_m_s,
                    "meas_y": translated_feedback.vy_body_m_s,
                    "pass": (
                        abs(translated_command.v_forward_cmd_m_s - 2.0) < 1e-9
                        and abs(translated_command.v_right_cmd_m_s - 3.0) < 1e-9
                        and abs(translated_feedback.vx_body_m_s - 4.0) < 1e-9
                        and abs(translated_feedback.vy_body_m_s - 5.0) < 1e-9
                    ),
                },
                "translated_command": {
                    "x": translated_command.v_forward_cmd_m_s,
                    "y": translated_command.v_right_cmd_m_s,
                    "pass": abs(translated_command.v_forward_cmd_m_s - 2.0) < 1e-9 and abs(translated_command.v_right_cmd_m_s - 3.0) < 1e-9,
                },
                "translated_feedback": {
                    "x": translated_feedback.vx_body_m_s,
                    "y": translated_feedback.vy_body_m_s,
                    "pass": abs(translated_feedback.vx_body_m_s - 4.0) < 1e-9 and abs(translated_feedback.vy_body_m_s - 5.0) < 1e-9,
                },
                "snapshot_debug": {
                    "x_cmd": snapshot.control_plane_velocity_debug.vx_control_plane_cmd_m_s,
                    "y_cmd": snapshot.control_plane_velocity_debug.vy_control_plane_cmd_m_s,
                    "x_meas": snapshot.control_plane_velocity_debug.vx_control_plane_meas_m_s,
                    "y_meas": snapshot.control_plane_velocity_debug.vy_control_plane_meas_m_s,
                    "pass": (
                        abs(snapshot.control_plane_velocity_debug.vx_control_plane_cmd_m_s - 2.0) < 1e-9
                        and abs(snapshot.control_plane_velocity_debug.vy_control_plane_cmd_m_s - 3.0) < 1e-9
                        and abs(snapshot.control_plane_velocity_debug.vx_control_plane_meas_m_s - 4.0) < 1e-9
                        and abs(snapshot.control_plane_velocity_debug.vy_control_plane_meas_m_s - 5.0) < 1e-9
                        and snapshot.control_plane_velocity_debug.world_feedback_mode == "projected_une"
                    ),
                },
                "outer_loop_output": {
                    "pitch_cmd_from_velocity_deg": runtime_case[0].pitch_cmd_from_velocity_deg,
                    "roll_cmd_from_velocity_deg": runtime_case[0].roll_cmd_from_velocity_deg,
                    "pass": runtime_case[0].pitch_cmd_from_velocity_deg < 0.0 and runtime_case[0].roll_cmd_from_velocity_deg < 0.0,
                },
            }
        finally:
            self.set_une_to_control_plane_config(original_config)
            self.set_world_command_preprocess_config(original_preprocess_config)
            self.set_world_velocity_feedback_mode(original_mode)

    def run_world_velocity_feedback_mode_self_check(self) -> dict[str, object]:
        original_config = self.une_to_control_plane_config
        original_preprocess_config = self.world_command_preprocess_config
        original_mode = self.world_velocity_feedback_mode
        try:
            self.set_une_to_control_plane_config(UneToControlPlaneConfig(swap_ne=False, sign_x=1.0, sign_y=1.0))
            self.set_world_command_preprocess_config(WorldCommandPreprocessConfig())
            self.set_world_velocity_command(WorldVelocityCommand(v_north_cmd_m_s=1.0, v_east_cmd_m_s=2.0))
            telemetry = RollRateTelemetry(
                v_north_une_m_s=3.0,
                v_east_une_m_s=4.0,
                vx_body_m_s=7.0,
                vy_body_m_s=8.0,
                heading_deg=0.0,
            )

            self.set_world_velocity_feedback_mode("projected_une")
            projected_command, projected_feedback, projected_debug = self._translate_world_velocity_for_control_plane(telemetry)

            self.set_world_velocity_feedback_mode("raw_body")
            raw_command, raw_feedback, raw_debug = self._translate_world_velocity_for_control_plane(telemetry)

            return {
                "projected_une": {
                    "cmd_x": projected_command.v_forward_cmd_m_s,
                    "cmd_y": projected_command.v_right_cmd_m_s,
                    "meas_x": projected_feedback.vx_body_m_s,
                    "meas_y": projected_feedback.vy_body_m_s,
                    "debug_active_mode": projected_debug.world_feedback_mode,
                    "pass": (
                        abs(projected_command.v_forward_cmd_m_s - 1.0) < 1e-9
                        and abs(projected_command.v_right_cmd_m_s - 2.0) < 1e-9
                        and abs(projected_feedback.vx_body_m_s - 3.0) < 1e-9
                        and abs(projected_feedback.vy_body_m_s - 4.0) < 1e-9
                        and abs(projected_debug.vx_projected_une_meas_m_s - 3.0) < 1e-9
                        and abs(projected_debug.vy_projected_une_meas_m_s - 4.0) < 1e-9
                        and abs(projected_debug.vx_raw_body_meas_m_s - 7.0) < 1e-9
                        and abs(projected_debug.vy_raw_body_meas_m_s - 8.0) < 1e-9
                        and projected_debug.world_feedback_mode == "projected_une"
                    ),
                },
                "raw_body": {
                    "cmd_x": raw_command.v_forward_cmd_m_s,
                    "cmd_y": raw_command.v_right_cmd_m_s,
                    "meas_x": raw_feedback.vx_body_m_s,
                    "meas_y": raw_feedback.vy_body_m_s,
                    "debug_active_mode": raw_debug.world_feedback_mode,
                    "pass": (
                        abs(raw_command.v_forward_cmd_m_s - 1.0) < 1e-9
                        and abs(raw_command.v_right_cmd_m_s - 2.0) < 1e-9
                        and abs(raw_feedback.vx_body_m_s - 7.0) < 1e-9
                        and abs(raw_feedback.vy_body_m_s - 8.0) < 1e-9
                        and abs(raw_debug.vx_projected_une_meas_m_s - 3.0) < 1e-9
                        and abs(raw_debug.vy_projected_une_meas_m_s - 4.0) < 1e-9
                        and abs(raw_debug.vx_raw_body_meas_m_s - 7.0) < 1e-9
                        and abs(raw_debug.vy_raw_body_meas_m_s - 8.0) < 1e-9
                        and raw_debug.world_feedback_mode == "raw_body"
                    ),
                },
            }
        finally:
            self.set_une_to_control_plane_config(original_config)
            self.set_world_command_preprocess_config(original_preprocess_config)
            self.set_world_velocity_feedback_mode(original_mode)

    def run_world_command_preprojection_sign_self_check(self) -> dict[str, object]:
        original_config = self.une_to_control_plane_config
        original_preprocess_config = self.world_command_preprocess_config
        try:
            self.set_une_to_control_plane_config(UneToControlPlaneConfig(swap_ne=False, sign_x=1.0, sign_y=-1.0))
            self.set_world_velocity_command(WorldVelocityCommand(v_north_cmd_m_s=1.0, v_east_cmd_m_s=1.0))

            self.set_world_command_preprocess_config(
                WorldCommandPreprocessConfig(invert_world_north=False, invert_world_east=True)
            )
            pre_heading_0_command, _, pre_heading_0_debug = self._translate_world_velocity_for_control_plane(
                RollRateTelemetry(heading_deg=0.0)
            )
            pre_heading_45_command, _, pre_heading_45_debug = self._translate_world_velocity_for_control_plane(
                RollRateTelemetry(heading_deg=45.0)
            )

            self.set_world_command_preprocess_config(WorldCommandPreprocessConfig())
            projected_heading_0 = self.world_velocity_projection.compute(
                v_north_cmd_m_s=1.0,
                v_east_cmd_m_s=1.0,
                heading_deg=0.0,
            )
            projected_heading_45 = self.world_velocity_projection.compute(
                v_north_cmd_m_s=1.0,
                v_east_cmd_m_s=1.0,
                heading_deg=45.0,
            )
            post_heading_0 = self.une_to_control_plane_mapper.map_horizontal(
                projected_heading_0.v_forward_cmd_from_world_m_s,
                projected_heading_0.v_right_cmd_from_world_m_s,
            )
            post_heading_45 = self.une_to_control_plane_mapper.map_horizontal(
                projected_heading_45.v_forward_cmd_from_world_m_s,
                projected_heading_45.v_right_cmd_from_world_m_s,
            )

            return {
                "heading_0": {
                    "pre_projection_fix": {
                        "cmd_x": pre_heading_0_command.v_forward_cmd_m_s,
                        "cmd_y": pre_heading_0_command.v_right_cmd_m_s,
                    },
                    "post_projection_invert_y": {
                        "cmd_x": post_heading_0[0],
                        "cmd_y": post_heading_0[1],
                    },
                    "pass": (
                        abs(pre_heading_0_command.v_forward_cmd_m_s - 1.0) < 1e-9
                        and abs(pre_heading_0_command.v_right_cmd_m_s - 1.0) < 1e-9
                        and abs(post_heading_0[0] - 1.0) < 1e-9
                        and abs(post_heading_0[1] + 1.0) < 1e-9
                    ),
                },
                "heading_45": {
                    "pre_projection_fix": {
                        "north_pre": pre_heading_45_debug.v_north_cmd_preprocessed_m_s,
                        "east_pre": pre_heading_45_debug.v_east_cmd_preprocessed_m_s,
                        "proj_forward": pre_heading_45_debug.v_forward_cmd_projected_m_s,
                        "proj_right": pre_heading_45_debug.v_right_cmd_projected_m_s,
                        "cmd_x": pre_heading_45_command.v_forward_cmd_m_s,
                        "cmd_y": pre_heading_45_command.v_right_cmd_m_s,
                    },
                    "post_projection_invert_y": {
                        "proj_forward": projected_heading_45.v_forward_cmd_from_world_m_s,
                        "proj_right": projected_heading_45.v_right_cmd_from_world_m_s,
                        "cmd_x": post_heading_45[0],
                        "cmd_y": post_heading_45[1],
                    },
                    "pass": (
                        abs(pre_heading_45_command.v_forward_cmd_m_s) < 1e-9
                        and abs(pre_heading_45_command.v_right_cmd_m_s - 1.414213562373095) < 1e-9
                        and abs(post_heading_45[0] - 1.414213562373095) < 1e-9
                        and abs(post_heading_45[1]) < 1e-9
                    ),
                },
            }
        finally:
            self.set_une_to_control_plane_config(original_config)
            self.set_world_command_preprocess_config(original_preprocess_config)

    def run_velocity_outer_loop_source_selection_self_check(self) -> dict[str, object]:
        original_config = self.une_to_control_plane_config
        original_preprocess_config = self.world_command_preprocess_config
        original_mode = self.world_velocity_feedback_mode
        telemetry = RollRateTelemetry(
            v_north_une_m_s=10.0,
            v_east_une_m_s=20.0,
            vx_body_m_s=1.0,
            vy_body_m_s=2.0,
            p_meas_rad_s=0.0,
            q_meas_rad_s=0.0,
        )
        try:
            self.body_velocity_command = BodyVelocityCommand(v_forward_cmd_m_s=3.0, v_right_cmd_m_s=4.0)
            self.world_velocity_command = WorldVelocityCommand(v_north_cmd_m_s=5.0, v_east_cmd_m_s=6.0)
            self.set_world_command_preprocess_config(WorldCommandPreprocessConfig())
            self.set_une_to_control_plane_config(UneToControlPlaneConfig(swap_ne=False, sign_x=1.0, sign_y=1.0))
            self.set_world_velocity_feedback_mode("projected_une")
            translated_command, translated_feedback, _ = self._translate_world_velocity_for_control_plane(telemetry)

            self.body_velocity_outer_loop_source = "body"
            body_command, body_feedback = self._select_active_velocity_outer_loop_inputs(
                telemetry=telemetry,
                translated_velocity_command=translated_command,
                translated_velocity_feedback=translated_feedback,
            )
            self.body_velocity_outer_loop_source = "world"
            world_command, world_feedback = self._select_active_velocity_outer_loop_inputs(
                telemetry=telemetry,
                translated_velocity_command=translated_command,
                translated_velocity_feedback=translated_feedback,
            )

            return {
                "body_source": {
                    "cmd_x": body_command.v_forward_cmd_m_s,
                    "cmd_y": body_command.v_right_cmd_m_s,
                    "meas_x": body_feedback.vx_body_m_s,
                    "meas_y": body_feedback.vy_body_m_s,
                    "pass": (
                        body_command.v_forward_cmd_m_s == 3.0
                        and body_command.v_right_cmd_m_s == 4.0
                        and body_feedback.vx_body_m_s == 1.0
                        and body_feedback.vy_body_m_s == 2.0
                    ),
                },
                "world_source": {
                    "cmd_x": world_command.v_forward_cmd_m_s,
                    "cmd_y": world_command.v_right_cmd_m_s,
                    "meas_x": world_feedback.vx_body_m_s,
                    "meas_y": world_feedback.vy_body_m_s,
                    "pass": (
                        world_command.v_forward_cmd_m_s == 5.0
                        and world_command.v_right_cmd_m_s == 6.0
                        and world_feedback.vx_body_m_s == 10.0
                        and world_feedback.vy_body_m_s == 20.0
                    ),
                },
            }
        finally:
            self.set_une_to_control_plane_config(original_config)
            self.set_world_command_preprocess_config(original_preprocess_config)
            self.set_world_velocity_feedback_mode(original_mode)

    def run_world_velocity_projection_self_check(self) -> dict[str, object]:
        case_north_aligned = self.world_velocity_projection.compute(
            v_north_cmd_m_s=1.0,
            v_east_cmd_m_s=0.0,
            heading_deg=0.0,
        )
        case_east_heading = self.world_velocity_projection.compute(
            v_north_cmd_m_s=1.0,
            v_east_cmd_m_s=0.0,
            heading_deg=90.0,
        )
        case_east_command = self.world_velocity_projection.compute(
            v_north_cmd_m_s=0.0,
            v_east_cmd_m_s=1.0,
            heading_deg=0.0,
        )
        return {
            "case_north_aligned": {
                "v_forward_cmd_from_world_m_s": case_north_aligned.v_forward_cmd_from_world_m_s,
                "v_right_cmd_from_world_m_s": case_north_aligned.v_right_cmd_from_world_m_s,
                "pass": case_north_aligned.v_forward_cmd_from_world_m_s > 0.0 and abs(case_north_aligned.v_right_cmd_from_world_m_s) < 1e-9,
            },
            "case_east_heading": {
                "v_forward_cmd_from_world_m_s": case_east_heading.v_forward_cmd_from_world_m_s,
                "v_right_cmd_from_world_m_s": case_east_heading.v_right_cmd_from_world_m_s,
                "pass": abs(case_east_heading.v_forward_cmd_from_world_m_s) < 1e-9 and case_east_heading.v_right_cmd_from_world_m_s < 0.0,
            },
            "case_east_command": {
                "v_forward_cmd_from_world_m_s": case_east_command.v_forward_cmd_from_world_m_s,
                "v_right_cmd_from_world_m_s": case_east_command.v_right_cmd_from_world_m_s,
                "pass": abs(case_east_command.v_forward_cmd_from_world_m_s) < 1e-9 and case_east_command.v_right_cmd_from_world_m_s > 0.0,
            },
        }

    def run_yaw_rate_self_check(self) -> dict[str, object]:
        test_command = replace(
            self.command,
            base_rpm=max(50.0, self.command.base_rpm),
            p_cmd_rad_s=0.0,
            q_cmd_rad_s=0.0,
            r_cmd_rad_s=0.2,
            kp_p=0.0,
            kp_q=0.0,
            kp_r=40.0 if self.command.kp_r == 0.0 else self.command.kp_r,
            output_limit=60.0 if self.command.output_limit <= 0.0 else self.command.output_limit,
        )
        test_rate_command, test_rate_config = self._split_legacy_rate_command(test_command)
        case_positive = self.attitude_pipeline.rate_controller.compute(
            p_cmd_rad_s=0.0,
            q_cmd_rad_s=0.0,
            r_cmd_rad_s=test_rate_command.r_cmd_rad_s,
            p_meas_rad_s=0.0,
            q_meas_rad_s=0.0,
            r_meas_rad_s=0.0,
            kp_p=0.0,
            kp_q=0.0,
            kp_r=test_rate_config.kp_r,
            ki_p=0.0,
            ki_q=0.0,
            ki_r=0.0,
            kd_p=0.0,
            kd_q=0.0,
            kd_r=0.0,
            integrator_limit_p=0.0,
            integrator_limit_q=0.0,
            integrator_limit_r=0.0,
            dt_s=0.1,
            output_limit=test_rate_config.output_limit,
        )
        case_near = self.attitude_pipeline.rate_controller.compute(
            p_cmd_rad_s=0.0,
            q_cmd_rad_s=0.0,
            r_cmd_rad_s=test_rate_command.r_cmd_rad_s,
            p_meas_rad_s=0.0,
            q_meas_rad_s=0.0,
            r_meas_rad_s=0.15,
            kp_p=0.0,
            kp_q=0.0,
            kp_r=test_rate_config.kp_r,
            ki_p=0.0,
            ki_q=0.0,
            ki_r=0.0,
            kd_p=0.0,
            kd_q=0.0,
            kd_r=0.0,
            integrator_limit_p=0.0,
            integrator_limit_q=0.0,
            integrator_limit_r=0.0,
            dt_s=0.1,
            output_limit=test_rate_config.output_limit,
        )
        case_negative = self.attitude_pipeline.rate_controller.compute(
            p_cmd_rad_s=0.0,
            q_cmd_rad_s=0.0,
            r_cmd_rad_s=-0.2,
            p_meas_rad_s=0.0,
            q_meas_rad_s=0.0,
            r_meas_rad_s=0.0,
            kp_p=0.0,
            kp_q=0.0,
            kp_r=test_rate_config.kp_r,
            ki_p=0.0,
            ki_q=0.0,
            ki_r=0.0,
            kd_p=0.0,
            kd_q=0.0,
            kd_r=0.0,
            integrator_limit_p=0.0,
            integrator_limit_q=0.0,
            integrator_limit_r=0.0,
            dt_s=0.1,
            output_limit=test_rate_config.output_limit,
        )
        positive_motor = self.mixer.mix(
            candidate=self.current_mixer,
            base_rpm=test_command.base_rpm,
            u_roll=0.0,
            u_pitch=0.0,
            u_yaw=case_positive.u_yaw,
        )
        negative_motor = self.mixer.mix(
            candidate=self.current_mixer,
            base_rpm=test_command.base_rpm,
            u_roll=0.0,
            u_pitch=0.0,
            u_yaw=case_negative.u_yaw,
        )
        positive_sign = check_yaw_sign_consistency(
            r_cmd_rad_s=test_rate_command.r_cmd_rad_s,
            r_meas_rad_s=0.0,
            u_yaw=case_positive.u_yaw,
            candidate=self.current_mixer,
            motor_outputs=positive_motor,
        )
        negative_sign = check_yaw_sign_consistency(
            r_cmd_rad_s=-0.2,
            r_meas_rad_s=0.0,
            u_yaw=case_negative.u_yaw,
            candidate=self.current_mixer,
            motor_outputs=negative_motor,
        )
        return {
            "case_positive": {
                "error_r_rad_s": case_positive.error_r_rad_s,
                "u_yaw": case_positive.u_yaw,
                "pass": case_positive.error_r_rad_s > 0.0 and case_positive.u_yaw > 0.0,
                "sign_check": positive_sign,
            },
            "case_near_target": {
                "error_r_rad_s": case_near.error_r_rad_s,
                "u_yaw": case_near.u_yaw,
                "pass": abs(case_near.error_r_rad_s) < abs(case_positive.error_r_rad_s) and abs(case_near.u_yaw) < abs(case_positive.u_yaw),
            },
            "case_negative": {
                "error_r_rad_s": case_negative.error_r_rad_s,
                "u_yaw": case_negative.u_yaw,
                "pass": case_negative.error_r_rad_s < 0.0 and case_negative.u_yaw < 0.0,
                "sign_check": negative_sign,
            },
            "sign_flip": {
                "pass": case_positive.u_yaw > 0.0 and case_negative.u_yaw < 0.0 and positive_sign["pass"] and negative_sign["pass"],
                "positive_motors": positive_motor.as_mapping(),
                "negative_motors": negative_motor.as_mapping(),
            },
        }

    def run_altitude_loop_self_check(self) -> dict[str, object]:
        test_config = AltitudeControlConfig(
            kp_alt=1.0,
            vz_max=2.0,
            kp_vz=10.0,
            ki_vz=0.0,
            kd_vz=0.0,
            vz_integrator_limit=0.0,
            throttle_min=0.0,
            throttle_max=460.0,
        )
        case_climb = self.altitude_pipeline.compute(
            telemetry=RollRateTelemetry(alt_m=0.0, vz_m_s=0.0, dt_s=0.1),
            altitude_command=AltitudeCommand(alt_cmd_m=2.0, hover_throttle=220.0),
            config=test_config,
        )
        case_near = self.altitude_pipeline.compute(
            telemetry=RollRateTelemetry(alt_m=1.5, vz_m_s=0.5, dt_s=0.1),
            altitude_command=AltitudeCommand(alt_cmd_m=2.0, hover_throttle=220.0),
            config=test_config,
        )
        case_descend = self.altitude_pipeline.compute(
            telemetry=RollRateTelemetry(alt_m=2.0, vz_m_s=0.0, dt_s=0.1),
            altitude_command=AltitudeCommand(alt_cmd_m=0.0, hover_throttle=220.0),
            config=test_config,
        )
        return {
            "case_climb": {
                "alt_cmd_m": case_climb.alt_cmd_m,
                "alt_error_m": case_climb.alt_error_m,
                "vz_cmd_m_s": case_climb.vz_cmd_m_s,
                "vz_error_m_s": case_climb.vz_error_m_s,
                "throttle_cmd": case_climb.throttle_cmd,
                "hover_throttle": 220.0,
                "pass": case_climb.alt_error_m > 0.0 and case_climb.vz_cmd_m_s > 0.0 and case_climb.vz_error_m_s > 0.0 and case_climb.throttle_cmd > 220.0,
            },
            "case_near_target": {
                "alt_error_m": case_near.alt_error_m,
                "vz_cmd_m_s": case_near.vz_cmd_m_s,
                "vz_error_m_s": case_near.vz_error_m_s,
                "throttle_cmd": case_near.throttle_cmd,
                "pass": abs(case_near.alt_error_m) < abs(case_climb.alt_error_m) and abs(case_near.vz_cmd_m_s) < abs(case_climb.vz_cmd_m_s) and abs(case_near.throttle_cmd - 220.0) < abs(case_climb.throttle_cmd - 220.0),
            },
            "case_descend": {
                "alt_cmd_m": case_descend.alt_cmd_m,
                "alt_error_m": case_descend.alt_error_m,
                "vz_cmd_m_s": case_descend.vz_cmd_m_s,
                "vz_error_m_s": case_descend.vz_error_m_s,
                "throttle_cmd": case_descend.throttle_cmd,
                "hover_throttle": 220.0,
                "pass": case_descend.alt_error_m < 0.0 and case_descend.vz_cmd_m_s < 0.0 and case_descend.vz_error_m_s < 0.0 and case_descend.throttle_cmd < 220.0,
            },
        }

    def run_yaw_outer_loop_self_check(self) -> dict[str, object]:
        test_config = YawOuterLoopConfig(
            kp_yaw=1.0,
            yaw_rate_limit_rad_s=1.0,
        )
        test_controller = YawOuterLoopController(config=test_config)
        case_wrap_positive = test_controller.compute(
            yaw_cmd_deg=10.0,
            heading_deg=350.0,
        )
        case_wrap_negative = test_controller.compute(
            yaw_cmd_deg=350.0,
            heading_deg=10.0,
        )
        limited = test_controller.compute(
            yaw_cmd_deg=180.0,
            heading_deg=0.0,
        )
        return {
            "case_wrap_positive": {
                "yaw_error_deg": case_wrap_positive.yaw_error_deg,
                "r_cmd_rad_s": case_wrap_positive.r_cmd_rad_s,
                "pass": abs(case_wrap_positive.yaw_error_deg - 20.0) < 1e-6 and case_wrap_positive.r_cmd_rad_s > 0.0,
            },
            "case_wrap_negative": {
                "yaw_error_deg": case_wrap_negative.yaw_error_deg,
                "r_cmd_rad_s": case_wrap_negative.r_cmd_rad_s,
                "pass": abs(case_wrap_negative.yaw_error_deg + 20.0) < 1e-6 and case_wrap_negative.r_cmd_rad_s < 0.0,
            },
            "case_limit": {
                "yaw_error_deg": limited.yaw_error_deg,
                "r_cmd_rad_s": limited.r_cmd_rad_s,
                "pass": abs(limited.r_cmd_rad_s) <= test_config.yaw_rate_limit_rad_s + 1e-9,
            },
        }

    def get_snapshot(self) -> RollRateTestState:
        return self.last_snapshot

    def _refresh_snapshot(
        self,
        *,
        telemetry=None,
        controller=None,
        world_velocity_projection: WorldToBodyVelocityProjectionOutput | None = None,
        control_plane_velocity_debug: ControlPlaneVelocityDebug | None = None,
        body_velocity_outer_loop: BodyVelocityOuterLoopOutput | None = None,
        outer_loop: AngleOuterLoopOutput | None = None,
        yaw_outer_loop: YawOuterLoopOutput | None = None,
        altitude_loop: AltitudeLoopOutput | None = None,
        motor_command: MotorCommand | None = None,
        status: str = "Idle",
        last_error: str = "",
    ) -> RollRateTestState:
        telemetry_value = telemetry if telemetry is not None else self.last_snapshot.telemetry
        controller_value = controller if controller is not None else self.last_snapshot.controller
        if world_velocity_projection is not None:
            world_velocity_projection_value = world_velocity_projection
        else:
            world_velocity_projection_value = self._compute_world_velocity_projection(telemetry_value)
        control_plane_velocity_debug_value = (
            control_plane_velocity_debug
            if control_plane_velocity_debug is not None
            else self._translate_world_velocity_for_control_plane(telemetry_value)[2]
        )
        body_velocity_outer_loop_value = body_velocity_outer_loop if body_velocity_outer_loop is not None else self.last_snapshot.body_velocity_outer_loop
        outer_loop_value = outer_loop if outer_loop is not None else self.last_snapshot.outer_loop
        yaw_outer_loop_value = yaw_outer_loop if yaw_outer_loop is not None else self.last_snapshot.yaw_outer_loop
        altitude_loop_value = altitude_loop if altitude_loop is not None else self.last_snapshot.altitude_loop
        motor_value = motor_command if motor_command is not None else self.last_snapshot.motor_command
        binding = self.hardware.get_binding_status(test_running=self.test_running)
        self.last_snapshot = RollRateTestState(
            binding=binding,
            command=self.command,
            telemetry=telemetry_value,
            controller=controller_value,
            angle_command=self.angle_command,
            body_velocity_command=self.body_velocity_command,
            world_velocity_command=self.world_velocity_command,
            world_command_preprocess_config=self.world_command_preprocess_config,
            une_to_control_plane_config=self.une_to_control_plane_config,
            world_velocity_feedback_mode=self.world_velocity_feedback_mode,
            outer_loop_config=self.attitude_pipeline.angle_controller.config,
            outer_loop=outer_loop_value,
            yaw_outer_loop_config=self.yaw_outer_loop_config,
            yaw_outer_loop=yaw_outer_loop_value,
            body_velocity_outer_loop_config=self.body_velocity_outer_loop_config,
            body_velocity_outer_loop=body_velocity_outer_loop_value,
            control_plane_velocity_debug=control_plane_velocity_debug_value,
            world_velocity_projection=world_velocity_projection_value,
            body_velocity_outer_loop_enabled=self.body_velocity_outer_loop_running,
            outer_loop_running=self.outer_loop_running,
            altitude_command=self.altitude_command,
            altitude_control_config=self.altitude_config,
            altitude_loop=altitude_loop_value,
            altitude_loop_running=self.altitude_loop_running,
            motor_command=motor_value,
            mixer_name=self.current_mixer.name,
            mixer_matrix_text=self.mixer.format_matrix_text(self.current_mixer),
            controller_tags_bound=dict(binding.controller_tags),
            rotor_tags_bound=dict(binding.rotor_tags),
            last_error=last_error,
            last_status=status,
        )
        return self.last_snapshot
