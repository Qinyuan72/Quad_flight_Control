from __future__ import annotations

try:
    from data_api.models import Matrix4x4, MixerCandidate
except ImportError:
    from ..data_api.models import Matrix4x4, MixerCandidate


def _candidate(name: str, description: str, rows: Matrix4x4) -> MixerCandidate:
    return MixerCandidate(name=name, description=description, matrix=rows)


_PRESETS: dict[str, MixerCandidate] = {
    "quad_x_roll_pitch_a": _candidate(
        "quad_x_roll_pitch_a",
        "Roll acts left-vs-right and pitch acts front-vs-rear with candidate A sign pattern.",
        (
            (1.0, -1.0, 1.0, 1.0),
            (1.0, 1.0, 1.0, -1.0),
            (1.0, -1.0, -1.0, -1.0),
            (1.0, 1.0, -1.0, 1.0),
        ),
    ),
    "quad_x_roll_pitch_b": _candidate(
        "quad_x_roll_pitch_b",
        "Same roll column as candidate A, inverted pitch column for sign debugging.",
        (
            (
                (1.0, -1.0, -1.0, -1.0),
                (1.0,  1.0, -1.0,  1.0),
                (1.0, -1.0,  1.0,  1.0),
                (1.0,  1.0,  1.0, -1.0),
            )
        ),
    ),
    "quad_x_roll_neg_pitch_a": _candidate(
        "quad_x_roll_neg_pitch_a",
        "Inverted roll column with candidate A pitch signs for combined sign debugging.",
        (
            (1.0, 1.0, 1.0, 1.0),
            (1.0, -1.0, 1.0, -1.0),
            (1.0, 1.0, -1.0, -1.0),
            (1.0, -1.0, -1.0, 1.0),
        ),
    ),
    "quad_x_roll_only": _candidate(
        "quad_x_roll_only",
        "Roll-only debugging mixer with pitch contribution removed.",
        (
            (1.0, -1.0, 0.0, 1.0),
            (1.0, 1.0, 0.0, -1.0),
            (1.0, -1.0, 0.0, -1.0),
            (1.0, 1.0, 0.0, 1.0),
        ),
    ),
}


def list_candidates() -> list[str]:
    """Return available mixer candidate names."""

    return list(_PRESETS.keys())


def get_candidate(name: str) -> MixerCandidate:
    """Return a mixer candidate by name."""

    try:
        return _PRESETS[name]
    except KeyError as exc:
        raise KeyError(f"Unknown mixer candidate: {name}") from exc
