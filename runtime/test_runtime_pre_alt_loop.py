from __future__ import annotations

from dataclasses import replace

from Control_loop_test_v1.control.controller import CascadedRollPitchController, AngleOuterLoopController, RollPitchRatePController
from Control_loop_test_v1.control.mixer import MatrixMixer
from Control_loop_test_v1.control.mixer_presets import get_candidate, list_candidates
from Control_loop_test_v1.data_api.krpc_bindings import KrpcQuadHardware
from Control_loop_test_v1.data_api.models import (
    AngleCommand,
    AngleOuterLoopConfig,
    AngleOuterLoopOutput,
    MotorCommand,
    RollRateTelemetry,
    RollRateTestCommand,
    RollRateTestState,
)
from Control_loop_test_v1.data_api.telemetry import RollRateTelemetryReader


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
        self.current_mixer = get_candidate(list_candidates()[0])
        self.test_running = False
        self.last_snapshot = RollRateTestState(
            command=self.command,
            angle_command=self.angle_command,
            mixer_name=self.current_mixer.name,
            mixer_matrix_text=self.mixer.format_matrix_text(self.current_mixer),
        )

    def connect(self) -> None:
        self.hardware.connect()
        self.telemetry.connect()
        self._refresh_snapshot(status="Connected")

    def disconnect(self) -> None:
        self.test_running = False
        self.telemetry.disconnect()
        self.hardware.disconnect()
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
        self.test_running = True
        self._refresh_snapshot(status="Test loop started")

    def stop_test(self) -> None:
        self.test_running = False
        if self.hardware.is_bound():
            self.hardware.zero_outputs()
        self._refresh_snapshot(status="Test loop stopped")

    def emergency_stop(self) -> None:
        self.test_running = False
        self.hardware.emergency_stop()
        self._refresh_snapshot(status="Emergency stop issued")

    def set_command(self, command: RollRateTestCommand) -> None:
        self.command = command
        self._refresh_snapshot(status="Parameters updated")

    def set_angle_command(self, angle_command: AngleCommand) -> None:
        self.angle_command = angle_command
        self._refresh_snapshot(status="Outer-loop targets updated")

    def set_outer_loop_config(self, config: AngleOuterLoopConfig) -> None:
        self.controller.set_outer_loop_config(config)
        self._refresh_snapshot(status="Outer-loop gains updated")

    def set_mixer_candidate(self, name: str) -> None:
        self.current_mixer = get_candidate(name)
        self._refresh_snapshot(status=f"Mixer candidate set to {name}")

    def get_mixer_names(self) -> list[str]:
        return list_candidates()

    def step(self) -> RollRateTestState:
        telemetry = self.telemetry.read()
        outer_loop, controller_result = self.controller.compute(
            telemetry=telemetry,
            angle_command=self.angle_command,
            rate_command=self.command,
        )
        motor_command = self.mixer.mix(
            candidate=self.current_mixer,
            base_rpm=self.command.base_rpm,
            u_roll=controller_result.u_roll,
            u_pitch=controller_result.u_pitch,
        )
        self.hardware.write_motor_command(motor_command)
        return self._refresh_snapshot(
            telemetry=telemetry,
            outer_loop=outer_loop,
            controller=controller_result,
            motor_command=motor_command,
            status="Step executed",
        )

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
                "pass": case_roll.roll_error_deg > 0.0
                and case_roll.p_cmd_rad_s > 0.0
                and abs(case_roll.q_cmd_rad_s) < 1e-9,
            },
            "case_roll_near_target": {
                "roll_error_deg": case_roll_near.roll_error_deg,
                "p_cmd_from_angle": case_roll_near.p_cmd_rad_s,
                "pass": case_roll_near.roll_error_deg < case_roll.roll_error_deg
                and case_roll_near.p_cmd_rad_s < case_roll.p_cmd_rad_s,
            },
            "case_pitch": {
                "pitch_error_deg": case_pitch.pitch_error_deg,
                "p_cmd_from_angle": case_pitch.p_cmd_rad_s,
                "q_cmd_from_angle": case_pitch.q_cmd_rad_s,
                "pass": case_pitch.q_cmd_rad_s > 0.0 and abs(case_pitch.p_cmd_rad_s) < 1e-9,
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
        motor_command: MotorCommand | None = None,
        status: str = "Idle",
        last_error: str = "",
    ) -> RollRateTestState:
        telemetry_value = telemetry if telemetry is not None else self.last_snapshot.telemetry
        controller_value = controller if controller is not None else self.last_snapshot.controller
        outer_loop_value = outer_loop if outer_loop is not None else self.last_snapshot.outer_loop
        motor_value = motor_command if motor_command is not None else self.last_snapshot.motor_command
        binding = self.hardware.get_binding_status(test_running=self.test_running)
        self.last_snapshot = RollRateTestState(
            binding=binding,
            command=self.command,
            telemetry=telemetry_value,
            controller=controller_value,
            angle_command=self.angle_command,
            outer_loop=outer_loop_value,
            motor_command=motor_value,
            mixer_name=self.current_mixer.name,
            mixer_matrix_text=self.mixer.format_matrix_text(self.current_mixer),
            controller_tags_bound=dict(binding.controller_tags),
            rotor_tags_bound=dict(binding.rotor_tags),
            last_error=last_error,
            last_status=status,
        )
        return self.last_snapshot
