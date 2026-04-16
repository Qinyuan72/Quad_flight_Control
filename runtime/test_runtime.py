from __future__ import annotations

from dataclasses import replace

try:
    from control.controller import (
        AltitudeOuterLoopController,
        CascadedAltitudeController,
        CascadedRollPitchController,
        AngleOuterLoopController,
        RollPitchRatePController,
        VerticalSpeedPController,
    )
    from control.mixer import MatrixMixer
    from control.mixer_presets import get_candidate, list_candidates
    from data_api.krpc_bindings import KrpcQuadHardware
    from data_api.models import (
        AltitudeCommand,
        AltitudeControlConfig,
        AltitudeLoopOutput,
        AngleCommand,
        AngleOuterLoopConfig,
        AngleOuterLoopOutput,
        MotorCommand,
        RollRateTelemetry,
        RollRateTestCommand,
        RollRateTestState,
    )
    from data_api.telemetry import RollRateTelemetryReader
except ImportError:
    from ..control.controller import (
        AltitudeOuterLoopController,
        CascadedAltitudeController,
        CascadedRollPitchController,
        AngleOuterLoopController,
        RollPitchRatePController,
        VerticalSpeedPController,
    )
    from ..control.mixer import MatrixMixer
    from ..control.mixer_presets import get_candidate, list_candidates
    from ..data_api.krpc_bindings import KrpcQuadHardware
    from ..data_api.models import (
        AltitudeCommand,
        AltitudeControlConfig,
        AltitudeLoopOutput,
        AngleCommand,
        AngleOuterLoopConfig,
        AngleOuterLoopOutput,
        MotorCommand,
        RollRateTelemetry,
        RollRateTestCommand,
        RollRateTestState,
    )
    from ..data_api.telemetry import RollRateTelemetryReader


class RollRateInnerLoopRuntime:
    """Orchestrate one narrow roll/pitch inner-loop test step."""

    def __init__(self) -> None:
        self.hardware = KrpcQuadHardware()
        self.telemetry = RollRateTelemetryReader(self.hardware)
        self.controller = CascadedRollPitchController(
            rate_controller=RollPitchRatePController(),
            angle_controller=AngleOuterLoopController(
                config=AngleOuterLoopConfig(
                    kp_roll_angle=0.0,
                    kp_pitch_angle=0.0,
                    rate_limit_rad_s=1.0,
                )
            ),
        )
        self.altitude_controller = CascadedAltitudeController(
            altitude_outer_controller=AltitudeOuterLoopController(
                config=AltitudeControlConfig(
                    kp_alt=0.0,
                    vz_max=0.0,
                    kp_vz=0.0,
                    ki_vz=0.0,
                    throttle_min=0.0,
                    throttle_max=460.0,
                )
            ),
            vertical_speed_controller=VerticalSpeedPController(
                config=AltitudeControlConfig(
                    kp_alt=0.0,
                    vz_max=0.0,
                    kp_vz=0.0,
                    ki_vz=0.0,
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
            kp_p=0.0,
            kp_q=0.0,
            output_limit=0.0,
        )
        self.angle_command = AngleCommand()
        self.altitude_command = AltitudeCommand()
        self.altitude_config = AltitudeControlConfig(
            kp_alt=0.0,
            vz_max=0.0,
            kp_vz=0.0,
            ki_vz=0.0,
            throttle_min=0.0,
            throttle_max=460.0,
        )
        self.current_mixer = get_candidate(list_candidates()[0])
        self.test_running = False
        self.outer_loop_running = False
        self.altitude_loop_running = False
        self.last_snapshot = RollRateTestState(
            command=self.command,
            angle_command=self.angle_command,
            outer_loop_config=self.controller.angle_controller.config,
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
        self.altitude_loop_running = False
        self.telemetry.disconnect()
        self.hardware.disconnect()
        self.altitude_controller.reset()
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
        if self.altitude_loop_running:
            self.altitude_controller.reset()
        self.test_running = True
        self._refresh_snapshot(status="Test loop started")

    def stop_test(self) -> None:
        self.test_running = False
        self.altitude_controller.reset()
        if self.hardware.is_bound():
            self.hardware.zero_outputs()
        self._refresh_snapshot(status="Test loop stopped")

    def emergency_stop(self) -> None:
        self.test_running = False
        self.outer_loop_running = False
        self.altitude_loop_running = False
        self.altitude_controller.reset()
        self.hardware.emergency_stop()
        self._refresh_snapshot(status="Emergency stop issued")

    def set_command(self, command: RollRateTestCommand) -> None:
        self.command = command
        self._refresh_live_snapshot(status="Inner-loop parameters updated")

    def set_angle_command(self, angle_command: AngleCommand) -> None:
        self.angle_command = angle_command
        self._refresh_live_snapshot(status="Outer-loop targets updated")

    def set_outer_loop_config(self, config: AngleOuterLoopConfig) -> None:
        self.controller.set_outer_loop_config(config)
        self._refresh_live_snapshot(status="Outer-loop gains updated")

    def start_outer_loop(self) -> None:
        self.outer_loop_running = True
        self._refresh_live_snapshot(status="Outer loop enabled")

    def stop_outer_loop(self) -> None:
        self.outer_loop_running = False
        self._refresh_live_snapshot(status="Outer loop disabled")

    def set_altitude_command(self, altitude_command: AltitudeCommand) -> None:
        self.altitude_command = altitude_command
        self._refresh_live_snapshot(status="Vertical-speed target updated")

    def set_altitude_config(self, config: AltitudeControlConfig) -> None:
        self.altitude_config = config
        self.altitude_controller.set_config(config)
        self._refresh_live_snapshot(status="Vertical-speed gains updated")

    def start_altitude_loop(self) -> None:
        self.altitude_controller.reset()
        self.altitude_loop_running = True
        self._refresh_live_snapshot(status="Vertical-speed loop enabled")

    def stop_altitude_loop(self) -> None:
        self.altitude_loop_running = False
        self.altitude_controller.reset()
        self._refresh_live_snapshot(status="Vertical-speed loop disabled")

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
        altitude_loop, collective_cmd = self._compute_altitude_channel(telemetry)
        outer_loop, controller_result = self._compute_attitude_channel(telemetry)
        motor_command = self.mixer.mix(
            candidate=self.current_mixer,
            base_rpm=collective_cmd,
            u_roll=controller_result.u_roll,
            u_pitch=controller_result.u_pitch,
        )
        if write_outputs:
            self.hardware.write_motor_command(motor_command)
        return self._refresh_snapshot(
            telemetry=telemetry,
            outer_loop=outer_loop,
            altitude_loop=altitude_loop,
            controller=controller_result,
            motor_command=motor_command,
            status=status,
        )

    def _compute_attitude_channel(self, telemetry: RollRateTelemetry) -> tuple[AngleOuterLoopOutput, object]:
        if self.outer_loop_running:
            return self.controller.compute(
                telemetry=telemetry,
                angle_command=self.angle_command,
                rate_command=self.command,
            )

        outer_loop = AngleOuterLoopOutput()
        controller_result = self.controller.rate_controller.compute(
            p_cmd_rad_s=self.command.p_cmd_rad_s,
            q_cmd_rad_s=self.command.q_cmd_rad_s,
            p_meas_rad_s=telemetry.p_meas_rad_s,
            q_meas_rad_s=telemetry.q_meas_rad_s,
            kp_p=self.command.kp_p,
            kp_q=self.command.kp_q,
            output_limit=self.command.output_limit,
        )
        return (outer_loop, controller_result)

    def _compute_altitude_channel(self, telemetry: RollRateTelemetry) -> tuple[AltitudeLoopOutput, float]:
        altitude_loop = self.altitude_controller.compute(
            telemetry=telemetry,
            altitude_command=self.altitude_command,
        )
        collective_cmd = altitude_loop.throttle_cmd if self.altitude_loop_running else self.command.base_rpm
        return (altitude_loop, collective_cmd)

    def run_outer_loop_self_check(self) -> dict[str, object]:
        test_command = replace(
            self.command,
            base_rpm=max(0.0, self.command.base_rpm),
            kp_p=1.0 if self.command.kp_p == 0.0 else self.command.kp_p,
            kp_q=1.0 if self.command.kp_q == 0.0 else self.command.kp_q,
            output_limit=10.0 if self.command.output_limit <= 0.0 else self.command.output_limit,
        )
        test_config = AngleOuterLoopConfig(
            kp_roll_angle=0.10,
            kp_pitch_angle=0.10,
            rate_limit_rad_s=2.0,
        )

        case_roll = self.controller.compute(
            telemetry=RollRateTelemetry(roll_deg=0.0, pitch_deg=0.0, p_meas_rad_s=0.0, q_meas_rad_s=0.0),
            angle_command=AngleCommand(roll_cmd_deg=5.0, pitch_cmd_deg=0.0),
            rate_command=test_command,
            outer_config=test_config,
        )[0]
        case_roll_near = self.controller.compute(
            telemetry=RollRateTelemetry(roll_deg=4.0, pitch_deg=0.0, p_meas_rad_s=0.0, q_meas_rad_s=0.0),
            angle_command=AngleCommand(roll_cmd_deg=5.0, pitch_cmd_deg=0.0),
            rate_command=test_command,
            outer_config=test_config,
        )[0]
        case_pitch = self.controller.compute(
            telemetry=RollRateTelemetry(roll_deg=0.0, pitch_deg=0.0, p_meas_rad_s=0.0, q_meas_rad_s=0.0),
            angle_command=AngleCommand(roll_cmd_deg=0.0, pitch_cmd_deg=5.0),
            rate_command=test_command,
            outer_config=test_config,
        )[0]

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
        }

    def run_altitude_loop_self_check(self) -> dict[str, object]:
        test_config = AltitudeControlConfig(
            kp_alt=0.0,
            vz_max=0.0,
            kp_vz=10.0,
            ki_vz=0.0,
            throttle_min=0.0,
            throttle_max=460.0,
        )
        case_climb = self.altitude_controller.compute(
            telemetry=RollRateTelemetry(vz_m_s=0.0, dt_s=0.1),
            altitude_command=AltitudeCommand(alt_cmd_m=2.0, hover_throttle=220.0),
            config=test_config,
        )
        case_near = self.altitude_controller.compute(
            telemetry=RollRateTelemetry(vz_m_s=1.5, dt_s=0.1),
            altitude_command=AltitudeCommand(alt_cmd_m=2.0, hover_throttle=220.0),
            config=test_config,
        )
        case_descend = self.altitude_controller.compute(
            telemetry=RollRateTelemetry(vz_m_s=0.0, dt_s=0.1),
            altitude_command=AltitudeCommand(alt_cmd_m=-2.0, hover_throttle=220.0),
            config=test_config,
        )
        return {
            "case_climb": {
                "vz_cmd_m_s": case_climb.vz_cmd_m_s,
                "vz_error_m_s": case_climb.vz_error_m_s,
                "throttle_cmd": case_climb.throttle_cmd,
                "hover_throttle": 220.0,
                "pass": case_climb.vz_cmd_m_s > 0.0 and case_climb.vz_error_m_s > 0.0 and case_climb.throttle_cmd > 220.0,
            },
            "case_near_target": {
                "vz_error_m_s": case_near.vz_error_m_s,
                "throttle_cmd": case_near.throttle_cmd,
                "pass": abs(case_near.vz_error_m_s) < abs(case_climb.vz_error_m_s) and abs(case_near.throttle_cmd - 220.0) < abs(case_climb.throttle_cmd - 220.0),
            },
            "case_descend": {
                "vz_cmd_m_s": case_descend.vz_cmd_m_s,
                "vz_error_m_s": case_descend.vz_error_m_s,
                "throttle_cmd": case_descend.throttle_cmd,
                "hover_throttle": 220.0,
                "pass": case_descend.vz_cmd_m_s < 0.0 and case_descend.vz_error_m_s < 0.0 and case_descend.throttle_cmd < 220.0,
            },
        }

    def get_snapshot(self) -> RollRateTestState:
        return self.last_snapshot

    def _refresh_snapshot(
        self,
        *,
        telemetry=None,
        controller=None,
        outer_loop: AngleOuterLoopOutput | None = None,
        altitude_loop: AltitudeLoopOutput | None = None,
        motor_command: MotorCommand | None = None,
        status: str = "Idle",
        last_error: str = "",
    ) -> RollRateTestState:
        telemetry_value = telemetry if telemetry is not None else self.last_snapshot.telemetry
        controller_value = controller if controller is not None else self.last_snapshot.controller
        outer_loop_value = outer_loop if outer_loop is not None else self.last_snapshot.outer_loop
        altitude_loop_value = altitude_loop if altitude_loop is not None else self.last_snapshot.altitude_loop
        motor_value = motor_command if motor_command is not None else self.last_snapshot.motor_command
        binding = self.hardware.get_binding_status(test_running=self.test_running)
        self.last_snapshot = RollRateTestState(
            binding=binding,
            command=self.command,
            telemetry=telemetry_value,
            controller=controller_value,
            angle_command=self.angle_command,
            outer_loop_config=self.controller.angle_controller.config,
            outer_loop=outer_loop_value,
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





