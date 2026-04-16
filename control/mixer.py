from __future__ import annotations

try:
    from data_api.models import Axes4, MOTOR_KEYS, MixerCandidate, MotorCommand, RPM_MAX, RPM_MIN
except ImportError:
    from ..data_api.models import Axes4, MOTOR_KEYS, MixerCandidate, MotorCommand, RPM_MAX, RPM_MIN


class MatrixMixer:
    """Apply a configurable 4x4 mixer matrix to base/roll/pitch/yaw inputs."""

    def mix(
        self,
        *,
        candidate: MixerCandidate,
        base_rpm: float,
        u_roll: float,
        u_pitch: float,
        u_yaw: float,
    ) -> MotorCommand:
        control_vector = (float(base_rpm), float(u_roll), float(u_pitch), float(u_yaw))
        outputs = [self._dot(row, control_vector) for row in candidate.matrix]
        clipped = [self._clip_rpm(value) for value in outputs]
        return MotorCommand(
            fl_rpm=clipped[0],
            fr_rpm=clipped[1],
            rl_rpm=clipped[2],
            rr_rpm=clipped[3],
        )

    def format_matrix_text(self, candidate: MixerCandidate) -> str:
        header = "rows=motor outputs [FL, FR, RL, RR]\ncols=input vector [base, roll, pitch, yaw]\n"
        lines = [header, f"name: {candidate.name}", f"desc: {candidate.description}", "matrix:"]
        row_names = ("FL", "FR", "RL", "RR")
        for row_name, row in zip(row_names, candidate.matrix):
            lines.append(
                f"  {row_name}: [{row[0]: .2f}, {row[1]: .2f}, {row[2]: .2f}, {row[3]: .2f}]"
            )
        return "\n".join(lines)

    def _dot(self, row: Axes4, vector: tuple[float, float, float, float]) -> float:
        return row[0] * vector[0] + row[1] * vector[1] + row[2] * vector[2] + row[3] * vector[3]

    def _clip_rpm(self, value: float) -> float:
        return max(RPM_MIN, min(RPM_MAX, float(value)))


def check_yaw_sign_consistency(
    *,
    r_cmd_rad_s: float,
    r_meas_rad_s: float,
    u_yaw: float,
    candidate: MixerCandidate,
    motor_outputs: MotorCommand,
) -> dict[str, object]:
    yaw_error = float(r_cmd_rad_s) - float(r_meas_rad_s)
    yaw_error_sign = _sign(yaw_error)
    u_yaw_sign = _sign(u_yaw)
    yaw_column = {key: row[3] for key, row in zip(MOTOR_KEYS, candidate.matrix)}
    motor_mapping = motor_outputs.as_mapping()
    positive_group_sum = sum(motor_mapping[key] for key, coeff in yaw_column.items() if coeff > 0.0)
    negative_group_sum = sum(motor_mapping[key] for key, coeff in yaw_column.items() if coeff < 0.0)
    yaw_differential = positive_group_sum - negative_group_sum
    yaw_differential_sign = _sign(yaw_differential)

    command_sign_matches = yaw_error_sign == 0 or u_yaw_sign == yaw_error_sign
    motor_sign_matches = u_yaw_sign == 0 or yaw_differential_sign == u_yaw_sign

    return {
        "pass": command_sign_matches and motor_sign_matches,
        "yaw_error": yaw_error,
        "u_yaw": u_yaw,
        "yaw_differential": yaw_differential,
        "positive_group_sum": positive_group_sum,
        "negative_group_sum": negative_group_sum,
        "yaw_column": yaw_column,
        "command_sign_matches": command_sign_matches,
        "motor_sign_matches": motor_sign_matches,
    }


def _sign(value: float, *, tol: float = 1e-9) -> int:
    if value > tol:
        return 1
    if value < -tol:
        return -1
    return 0
