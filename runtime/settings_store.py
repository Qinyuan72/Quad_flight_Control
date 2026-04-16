from __future__ import annotations

import copy
import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any


SETTINGS_PATH = Path(__file__).resolve().parent.parent / "runtime" / "startup_state.json"

DEFAULT_SETTINGS: dict[str, Any] = {
    "command": {
        "base_rpm": 288.0,
        "p_cmd_rad_s": 0.0,
        "q_cmd_rad_s": 0.0,
        "kp_p": 40.0,
        "kp_q": 40.0,
        "output_limit": 50.0,
    },
    "angle_command": {
        "roll_cmd_deg": 0.0,
        "pitch_cmd_deg": 0.0,
    },
    "outer_config": {
        "kp_roll_angle": -0.05,
        "kp_pitch_angle": -0.05,
        "rate_limit_rad_s": 1.0,
    },
    "altitude_command": {
        "alt_cmd_m": 0.0,
        "hover_throttle": 277.0,
    },
    "altitude_config": {
        "kp_alt": 0.0,
        "vz_max": 0.0,
        "kp_vz": 30.0,
        "ki_vz": 0.0,
        "throttle_min": 150.0,
        "throttle_max": 400.0,
    },
    "mixer_name": "quad_x_roll_pitch_b",
}


def load_settings() -> dict[str, Any]:
    settings = copy.deepcopy(DEFAULT_SETTINGS)
    if not SETTINGS_PATH.exists():
        return settings

    try:
        loaded = json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
    except Exception:
        return settings

    if not isinstance(loaded, dict):
        return settings

    return _merge_settings(settings, loaded)


def save_settings(settings: Mapping[str, Any]) -> None:
    payload = _merge_settings(copy.deepcopy(DEFAULT_SETTINGS), settings)
    SETTINGS_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _merge_settings(base: dict[str, Any], override: Mapping[str, Any]) -> dict[str, Any]:
    for key, value in override.items():
        if key not in base:
            continue
        if isinstance(base[key], dict) and isinstance(value, Mapping):
            nested = dict(base[key])
            for nested_key, nested_value in value.items():
                if nested_key in nested:
                    nested[nested_key] = nested_value
            base[key] = nested
        else:
            base[key] = value
    return base
