from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping


# ============================================================================
# Common type aliases and constants
# ============================================================================

Axes4 = tuple[float, float, float, float]
Matrix4x4 = tuple[Axes4, Axes4, Axes4, Axes4]

MOTOR_KEYS: tuple[str, str, str, str] = ("FL", "FR", "RL", "RR")

RPM_MIN = 0.0
RPM_MAX = 460.0

DEFAULT_CONTROLLER_TAGS: Mapping[str, str] = {
    "FL": "ControllerFL",
    "FR": "ControllerFR",
    "RL": "ControllerRL",
    "RR": "ControllerRR",
}

DEFAULT_ROTOR_TAGS: Mapping[str, str] = {
    "FL": "rotorFL",
    "FR": "rotorFR",
    "RL": "rotorRL",
    "RR": "rotorRR",
}


# ============================================================================
# Telemetry / command / controller data models
# ============================================================================

@dataclass(frozen=True)
class RollRateTelemetry:
    """Telemetry required for roll/pitch inner-loop, outer-loop, and altitude testing."""

    p_meas_rad_s: float = 0.0
    q_meas_rad_s: float = 0.0
    r_meas_rad_s: float = 0.0

    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    heading_deg: float = 0.0

    roll_rate_deg_s: float = 0.0
    pitch_rate_deg_s: float = 0.0
    yaw_rate_deg_s: float = 0.0

    alt_m: float = 0.0
    vz_m_s: float = 0.0
    vx_body_m_s: float = 0.0
    vy_body_m_s: float = 0.0
    vz_body_m_s: float = 0.0
    ground_speed_m_s: float = 0.0

    body_rates_rfd_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_rates_frd_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)

    timestamp_s: float = 0.0
    dt_s: float = 0.0


@dataclass(frozen=True)
class RollRateTestCommand:
    """User-editable command for direct body-rate inner-loop testing."""

    base_rpm: float = 0.0
    p_cmd_rad_s: float = 0.0
    q_cmd_rad_s: float = 0.0
    r_cmd_rad_s: float = 0.0
    kp_p: float = 0.0
    kp_q: float = 0.0
    kp_r: float = 0.0
    ki_p: float = 0.0
    ki_q: float = 0.0
    ki_r: float = 0.0
    kd_p: float = 0.0
    kd_q: float = 0.0
    kd_r: float = 0.0
    integrator_limit_p: float = 0.0
    integrator_limit_q: float = 0.0
    integrator_limit_r: float = 0.0
    output_limit: float = 0.0


@dataclass(frozen=True)
class BodyRateCommand:
    """Direct body-rate targets for the attitude inner loop."""

    p_cmd_rad_s: float = 0.0
    q_cmd_rad_s: float = 0.0
    r_cmd_rad_s: float = 0.0


@dataclass(frozen=True)
class BodyRatePIDConfig:
    """PID gains and limits for the body-rate inner loop."""

    kp_p: float = 0.0
    kp_q: float = 0.0
    kp_r: float = 0.0
    ki_p: float = 0.0
    ki_q: float = 0.0
    ki_r: float = 0.0
    kd_p: float = 0.0
    kd_q: float = 0.0
    kd_r: float = 0.0
    integrator_limit_p: float = 0.0
    integrator_limit_q: float = 0.0
    integrator_limit_r: float = 0.0
    output_limit: float = 0.0


@dataclass(frozen=True)
class ControllerResult:
    """Output of the PID-capable body-rate inner-loop controller."""

    error_p_rad_s: float = 0.0
    error_q_rad_s: float = 0.0
    error_r_rad_s: float = 0.0
    p_term_roll: float = 0.0
    i_term_roll: float = 0.0
    d_term_roll: float = 0.0
    p_term_pitch: float = 0.0
    i_term_pitch: float = 0.0
    d_term_pitch: float = 0.0
    p_term_yaw: float = 0.0
    i_term_yaw: float = 0.0
    d_term_yaw: float = 0.0
    u_roll: float = 0.0
    u_pitch: float = 0.0
    u_yaw: float = 0.0


# ============================================================================
# Outer-loop angle control models
# ============================================================================

@dataclass(frozen=True)
class AngleCommand:
    """User-commanded roll/pitch angle targets."""

    roll_cmd_deg: float = 0.0
    pitch_cmd_deg: float = 0.0
    yaw_cmd_deg: float = 0.0


@dataclass(frozen=True)
class BodyVelocityCommand:
    """User-commanded body-frame forward/right velocity targets."""

    v_forward_cmd_m_s: float = 0.0
    v_right_cmd_m_s: float = 0.0


@dataclass(frozen=True)
class AngleOuterLoopConfig:
    """
    PID-capable roll/pitch angle outer-loop configuration.

    The controller converts the human-edited gains once at entry and then
    performs all internal math in rad / rad/s / s.
    """

    kp_roll_angle: float = 0.08
    kp_pitch_angle: float = 0.08
    ki_roll_angle: float = 0.0
    ki_pitch_angle: float = 0.0
    kd_roll_angle: float = 0.0
    kd_pitch_angle: float = 0.0
    integrator_limit_roll: float = 0.0
    integrator_limit_pitch: float = 0.0
    rate_limit_rad_s: float = 1.0


@dataclass(frozen=True)
class AngleOuterLoopOutput:
    """Outer-loop output: angle error and generated body-rate commands."""

    roll_error_deg: float = 0.0
    pitch_error_deg: float = 0.0
    p_cmd_rad_s: float = 0.0
    q_cmd_rad_s: float = 0.0


@dataclass(frozen=True)
class OuterLoopState:
    """Latest roll/pitch outer-loop state for GUI/debug display."""

    roll_error_deg: float = 0.0
    pitch_error_deg: float = 0.0
    p_cmd_from_angle_rad_s: float = 0.0
    q_cmd_from_angle_rad_s: float = 0.0


@dataclass(frozen=True)
class YawOuterLoopConfig:
    """P-only yaw outer-loop configuration from heading error to yaw-rate command."""

    kp_yaw: float = 0.0
    yaw_rate_limit_rad_s: float = 1.0


@dataclass(frozen=True)
class YawOuterLoopOutput:
    """Yaw outer-loop output: wrapped heading error and generated yaw-rate command."""

    yaw_error_deg: float = 0.0
    r_cmd_rad_s: float = 0.0
    yaw_cmd_deg: float = 0.0
    heading_deg: float = 0.0


@dataclass(frozen=True)
class BodyVelocityOuterLoopConfig:
    """P-only body-frame XY velocity outer-loop configuration."""

    kp_v_forward: float = 0.0
    kp_v_right: float = 0.0
    velocity_angle_limit_deg: float = 10.0


@dataclass(frozen=True)
class BodyVelocityOuterLoopOutput:
    """Body-frame XY velocity outer-loop output for the attitude loop."""

    v_forward_cmd_m_s: float = 0.0
    v_right_cmd_m_s: float = 0.0
    v_forward_error_m_s: float = 0.0
    v_right_error_m_s: float = 0.0
    pitch_cmd_from_velocity_deg: float = 0.0
    roll_cmd_from_velocity_deg: float = 0.0


# ============================================================================
# Altitude-loop control models
# ============================================================================

@dataclass(frozen=True)
class AltitudeCommand:
    """User-commanded altitude target and hover collective."""

    alt_cmd_m: float = 0.0
    hover_throttle: float = 0.0


@dataclass(frozen=True)
class AltitudeControlConfig:
    """Altitude outer-loop and vertical-speed inner-loop configuration."""

    kp_alt: float = 0.0
    vz_max: float = 0.0
    kp_vz: float = 0.0
    ki_vz: float = 0.0
    kd_vz: float = 0.0
    vz_integrator_limit: float = 0.0
    throttle_min: float = 0.0
    throttle_max: float = RPM_MAX


@dataclass(frozen=True)
class AltitudeLoopOutput:
    """Latest altitude-loop state for GUI/debug display."""

    alt_cmd_m: float = 0.0
    alt_m: float = 0.0
    vz_m_s: float = 0.0
    alt_error_m: float = 0.0
    vz_cmd_m_s: float = 0.0
    vz_error_m_s: float = 0.0
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0
    throttle_correction: float = 0.0
    throttle_cmd: float = 0.0
    integrator_state: float = 0.0


@dataclass(frozen=True)
class VerticalSpeedCommand:
    """Vertical-speed inner-loop target and hover collective reference."""

    vz_cmd_m_s: float = 0.0
    hover_throttle: float = 0.0


@dataclass(frozen=True)
class VerticalSpeedPIDConfig:
    """PID gains and limits for the vertical-speed inner loop."""

    kp_vz: float = 0.0
    ki_vz: float = 0.0
    kd_vz: float = 0.0
    vz_integrator_limit: float = 0.0
    throttle_min: float = 0.0
    throttle_max: float = RPM_MAX


# ============================================================================
# Mixer / motor / binding state
# ============================================================================

@dataclass(frozen=True)
class MotorCommand:
    """Per-motor RPM commands written through the KAL controllers."""

    fl_rpm: float = 0.0
    fr_rpm: float = 0.0
    rl_rpm: float = 0.0
    rr_rpm: float = 0.0

    def as_mapping(self) -> dict[str, float]:
        return {
            "FL": self.fl_rpm,
            "FR": self.fr_rpm,
            "RL": self.rl_rpm,
            "RR": self.rr_rpm,
        }


@dataclass(frozen=True)
class ActuatorDemand:
    """Intermediate mixer demand before per-motor distribution."""

    base_rpm: float = 0.0
    u_roll: float = 0.0
    u_pitch: float = 0.0
    u_yaw: float = 0.0


@dataclass(frozen=True)
class MixerCandidate:
    """Matrix-based mixer candidate for base/roll/pitch/yaw to four motors."""

    name: str
    description: str
    matrix: Matrix4x4


@dataclass(frozen=True)
class BindingStatus:
    """High-level hardware/binding state for the GUI."""

    connected: bool = False
    bound: bool = False
    initialized: bool = False
    test_running: bool = False

    vessel_name: str = "-"

    controller_tags: Mapping[str, str] = field(
        default_factory=lambda: dict(DEFAULT_CONTROLLER_TAGS)
    )
    rotor_tags: Mapping[str, str] = field(
        default_factory=lambda: dict(DEFAULT_ROTOR_TAGS)
    )


# ============================================================================
# Top-level GUI/app state
# ============================================================================

@dataclass(frozen=True)
class RollRateTestState:
    """Single GUI snapshot: hardware, telemetry, control, and outputs."""

    binding: BindingStatus = field(default_factory=BindingStatus)

    command: RollRateTestCommand = field(default_factory=RollRateTestCommand)
    telemetry: RollRateTelemetry = field(default_factory=RollRateTelemetry)
    controller: ControllerResult = field(default_factory=ControllerResult)

    angle_command: AngleCommand = field(default_factory=AngleCommand)
    body_velocity_command: BodyVelocityCommand = field(default_factory=BodyVelocityCommand)
    outer_loop_config: AngleOuterLoopConfig = field(default_factory=AngleOuterLoopConfig)
    outer_loop: AngleOuterLoopOutput = field(default_factory=AngleOuterLoopOutput)
    yaw_outer_loop_config: YawOuterLoopConfig = field(default_factory=YawOuterLoopConfig)
    yaw_outer_loop: YawOuterLoopOutput = field(default_factory=YawOuterLoopOutput)
    body_velocity_outer_loop_config: BodyVelocityOuterLoopConfig = field(default_factory=BodyVelocityOuterLoopConfig)
    body_velocity_outer_loop: BodyVelocityOuterLoopOutput = field(default_factory=BodyVelocityOuterLoopOutput)
    body_velocity_outer_loop_enabled: bool = False
    outer_loop_running: bool = False

    altitude_command: AltitudeCommand = field(default_factory=AltitudeCommand)
    altitude_control_config: AltitudeControlConfig = field(default_factory=AltitudeControlConfig)
    altitude_loop: AltitudeLoopOutput = field(default_factory=AltitudeLoopOutput)
    altitude_loop_running: bool = False

    motor_command: MotorCommand = field(default_factory=MotorCommand)

    mixer_name: str = "quad_x_roll_pitch_a"
    mixer_matrix_text: str = ""

    controller_tags_bound: Mapping[str, str] = field(default_factory=dict)
    rotor_tags_bound: Mapping[str, str] = field(default_factory=dict)

    last_error: str = ""
    last_status: str = "Idle"
