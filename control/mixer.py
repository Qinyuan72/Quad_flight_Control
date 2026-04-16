from __future__ import annotations

from Control_loop_test_v1.data_api.models import Axes4, MixerCandidate, MotorCommand, RPM_MAX, RPM_MIN


class MatrixMixer:
    """Apply a configurable 4x4 mixer matrix to base/roll/pitch/yaw inputs."""

    def mix(
        self,
        *,
        candidate: MixerCandidate,
        base_rpm: float,
        u_roll: float,
        u_pitch: float,
    ) -> MotorCommand:
        control_vector = (float(base_rpm), float(u_roll), float(u_pitch), 0.0)
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
