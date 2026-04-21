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
        "r_cmd_rad_s": 0.0,
        "kp_p": 40.0,
        "kp_q": 40.0,
        "kp_r": 40.0,
        "ki_p": 0.0,
        "ki_q": 0.0,
        "ki_r": 50.0,
        "kd_p": 0.0,
        "kd_q": 0.0,
        "kd_r": 1.0,
        "integrator_limit_p": 0.0,
        "integrator_limit_q": 0.0,
        "integrator_limit_r": 0.0,
        "output_limit": 50.0,
    },
    "angle_command": {
        "roll_cmd_deg": 0.0,
        "pitch_cmd_deg": 0.0,
        "yaw_cmd_deg": 0.0,
    },
    "body_velocity_command": {
        "v_forward_cmd_m_s": 0.0,
        "v_right_cmd_m_s": 0.0,
    },
    "world_velocity_command": {
        "v_north_cmd_m_s": 0.0,
        "v_east_cmd_m_s": 0.0,
    },
    "world_command_preprocess_config": {
        "invert_world_north": False,
        "invert_world_east": False,
    },
    "une_to_control_plane_config": {
        "swap_ne": False,
        "sign_x": 1.0,
        "sign_y": 1.0,
    },
    "world_velocity_feedback_mode": "raw_body",
    "outer_config": {
        "kp_roll_angle": -0.05,
        "kp_pitch_angle": -0.05,
        "ki_roll_angle": 0.0,
        "ki_pitch_angle": 0.0,
        "kd_roll_angle": 0.0,
        "kd_pitch_angle": 0.0,
        "integrator_limit_roll": 0.0,
        "integrator_limit_pitch": 0.0,
        "rate_limit_rad_s": 1.0,
    },
    "body_velocity_outer_config": {
        "kp_v_forward": 0.0,
        "kp_v_right": 0.0,
        "velocity_angle_limit_deg": 10.0,
    },
    "yaw_outer_config": {
        "kp_yaw": 0.0,
        "yaw_rate_limit_rad_s": 1.0,
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
        "kd_vz": 0.0,
        "vz_integrator_limit": 0.0,
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
