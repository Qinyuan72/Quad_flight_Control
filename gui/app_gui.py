from __future__ import annotations

import json
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, scrolledtext, ttk

from Control_loop_test_v1.data_api.models import (
    AltitudeCommand,
    AltitudeControlConfig,
    AngleCommand,
    AngleOuterLoopConfig,
    RPM_MAX,
    RollRateTestCommand,
)
from Control_loop_test_v1.gui import gui_config
from Control_loop_test_v1.runtime.test_runtime import RollRateInnerLoopRuntime


DEFAULT_STARTUP_COMMAND = RollRateTestCommand(
    base_rpm=288.0,
    p_cmd_rad_s=0.0,
    q_cmd_rad_s=0.0,
    kp_p=40.0,
    kp_q=40.0,
    output_limit=50.0,
)
DEFAULT_STARTUP_OUTER_COMMAND = AngleCommand(
    roll_cmd_deg=0.0,
    pitch_cmd_deg=0.0,
)
DEFAULT_STARTUP_OUTER_CONFIG = AngleOuterLoopConfig(
    kp_roll_angle=-0.05,
    kp_pitch_angle=-0.05,
    rate_limit_rad_s=1.0,
)
DEFAULT_STARTUP_ALTITUDE_COMMAND = AltitudeCommand(
    alt_cmd_m=0.0,
    hover_throttle=277.0,
)
DEFAULT_STARTUP_ALTITUDE_CONFIG = AltitudeControlConfig(
    kp_alt=0.0,
    vz_max=0.0,
    kp_vz=30.0,
    ki_vz=0.0,
    throttle_min=150.0,
    throttle_max=400.0,
)
DEFAULT_STARTUP_MIXER_NAME = "quad_x_roll_pitch_b"
STARTUP_STATE_PATH = Path(__file__).with_name("startup_state.json")


class RollRateTestApp:
    """Tkinter GUI for the focused roll / pitch inner-loop test harness."""

    def __init__(self, root: tk.Tk, runtime: RollRateInnerLoopRuntime) -> None:
        self.root = root
        self.runtime = runtime
        self._after_id: str | None = None

        self.root.title(gui_config.WINDOW_TITLE)
        self.root.geometry(gui_config.WINDOW_GEOMETRY)
        self.root.minsize(*gui_config.WINDOW_MIN_SIZE)

        self.status_var = tk.StringVar(value="Idle")
        self.connection_var = tk.StringVar(value="Disconnected")
        self.test_var = tk.StringVar(value="Stopped")

        snapshot = self.runtime.get_snapshot()
        startup_state = self._load_startup_state()
        startup_command = self._command_from_state(startup_state)
        startup_outer_command = self._outer_command_from_state(startup_state)
        startup_outer_config = self._outer_config_from_state(startup_state)
        startup_altitude_command = self._altitude_command_from_state(startup_state)
        startup_altitude_config = self._altitude_config_from_state(startup_state)
        startup_mixer_name = str(startup_state.get("mixer_name", DEFAULT_STARTUP_MIXER_NAME))

        self.base_rpm_var = tk.StringVar(value=f"{startup_command.base_rpm:.1f}")
        self.p_cmd_var = tk.StringVar(value=f"{startup_command.p_cmd_rad_s:.3f}")
        self.q_cmd_var = tk.StringVar(value=f"{startup_command.q_cmd_rad_s:.3f}")
        self.kp_p_var = tk.StringVar(value=f"{startup_command.kp_p:.3f}")
        self.kp_q_var = tk.StringVar(value=f"{startup_command.kp_q:.3f}")
        self.output_limit_var = tk.StringVar(value=f"{startup_command.output_limit:.3f}")
        self.mixer_name_var = tk.StringVar(value=startup_mixer_name)

        self.roll_cmd_deg_var = tk.StringVar(value=f"{startup_outer_command.roll_cmd_deg:.3f}")
        self.pitch_cmd_deg_var = tk.StringVar(value=f"{startup_outer_command.pitch_cmd_deg:.3f}")
        self.kp_roll_angle_var = tk.StringVar(value=f"{startup_outer_config.kp_roll_angle:.3f}")
        self.kp_pitch_angle_var = tk.StringVar(value=f"{startup_outer_config.kp_pitch_angle:.3f}")
        self.angle_rate_limit_var = tk.StringVar(value=f"{startup_outer_config.rate_limit_rad_s:.3f}")

        self.alt_cmd_m_var = tk.StringVar(value=f"{startup_altitude_command.alt_cmd_m:.3f}")
        self.hover_throttle_var = tk.StringVar(value=f"{startup_altitude_command.hover_throttle:.3f}")
        self.kp_alt_var = tk.StringVar(value=f"{startup_altitude_config.kp_alt:.3f}")
        self.vz_max_var = tk.StringVar(value=f"{startup_altitude_config.vz_max:.3f}")
        self.kp_vz_var = tk.StringVar(value=f"{startup_altitude_config.kp_vz:.3f}")
        self.ki_vz_var = tk.StringVar(value=f"{startup_altitude_config.ki_vz:.3f}")
        self.throttle_min_var = tk.StringVar(value=f"{startup_altitude_config.throttle_min:.3f}")
        self.throttle_max_var = tk.StringVar(value=f"{startup_altitude_config.throttle_max:.3f}")

        self.binding_vars = {
            "controller_tags": tk.StringVar(value="-"),
            "rotor_tags": tk.StringVar(value="-"),
            "initialized": tk.StringVar(value="False"),
            "vessel": tk.StringVar(value="-"),
        }
        self.telemetry_vars = {
            "roll_deg": tk.StringVar(value="-"),
            "pitch_deg": tk.StringVar(value="-"),
            "heading_deg": tk.StringVar(value="-"),
            "alt_m": tk.StringVar(value="-"),
            "vz_m_s": tk.StringVar(value="-"),
            "p_meas": tk.StringVar(value="-"),
            "q_meas": tk.StringVar(value="-"),
            "r_meas": tk.StringVar(value="-"),
            "roll_rate_deg_s": tk.StringVar(value="-"),
            "pitch_rate_deg_s": tk.StringVar(value="-"),
            "yaw_rate_deg_s": tk.StringVar(value="-"),
            "dt_s": tk.StringVar(value="-"),
        }
        self.controller_vars = {
            "p_cmd": tk.StringVar(value="-"),
            "error_p": tk.StringVar(value="-"),
            "kp_p": tk.StringVar(value="-"),
            "u_roll": tk.StringVar(value="-"),
            "q_cmd": tk.StringVar(value="-"),
            "error_q": tk.StringVar(value="-"),
            "kp_q": tk.StringVar(value="-"),
            "u_pitch": tk.StringVar(value="-"),
            "output_limit": tk.StringVar(value="-"),
        }
        self.outer_loop_vars = {
            "roll_error_deg": tk.StringVar(value="-"),
            "pitch_error_deg": tk.StringVar(value="-"),
            "p_cmd_from_angle": tk.StringVar(value="-"),
            "q_cmd_from_angle": tk.StringVar(value="-"),
        }
        self.altitude_loop_vars = {
            "alt_cmd_m": tk.StringVar(value="-"),
            "alt_m": tk.StringVar(value="-"),
            "vz_m_s": tk.StringVar(value="-"),
            "alt_error_m": tk.StringVar(value="-"),
            "vz_cmd_m_s": tk.StringVar(value="-"),
            "vz_error_m_s": tk.StringVar(value="-"),
            "throttle_correction": tk.StringVar(value="-"),
            "throttle_cmd": tk.StringVar(value="-"),
        }
        self.mixer_vars = {"name": tk.StringVar(value=snapshot.mixer_name)}
        self.actuator_vars = {
            "FL": tk.StringVar(value="-"),
            "FR": tk.StringVar(value="-"),
            "RL": tk.StringVar(value="-"),
            "RR": tk.StringVar(value="-"),
        }
        self.motor_bars: dict[str, ttk.Progressbar] = {}

        self._build_ui()
        self._apply_startup_defaults()
        self._render_snapshot(self.runtime.get_snapshot())
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._schedule_loop()
    def _load_startup_state(self) -> dict[str, object]:
        if not STARTUP_STATE_PATH.exists():
            return {}
        try:
            loaded = json.loads(STARTUP_STATE_PATH.read_text(encoding="utf-8"))
        except Exception:
            return {}
        return loaded if isinstance(loaded, dict) else {}

    def _command_from_state(self, state: dict[str, object]) -> RollRateTestCommand:
        command = state.get("command", {}) if isinstance(state.get("command", {}), dict) else {}
        return RollRateTestCommand(
            base_rpm=float(command.get("base_rpm", DEFAULT_STARTUP_COMMAND.base_rpm)),
            p_cmd_rad_s=float(command.get("p_cmd_rad_s", DEFAULT_STARTUP_COMMAND.p_cmd_rad_s)),
            q_cmd_rad_s=float(command.get("q_cmd_rad_s", DEFAULT_STARTUP_COMMAND.q_cmd_rad_s)),
            kp_p=float(command.get("kp_p", DEFAULT_STARTUP_COMMAND.kp_p)),
            kp_q=float(command.get("kp_q", DEFAULT_STARTUP_COMMAND.kp_q)),
            output_limit=float(command.get("output_limit", DEFAULT_STARTUP_COMMAND.output_limit)),
        )

    def _outer_command_from_state(self, state: dict[str, object]) -> AngleCommand:
        angle_command = state.get("angle_command", {}) if isinstance(state.get("angle_command", {}), dict) else {}
        return AngleCommand(
            roll_cmd_deg=float(angle_command.get("roll_cmd_deg", DEFAULT_STARTUP_OUTER_COMMAND.roll_cmd_deg)),
            pitch_cmd_deg=float(angle_command.get("pitch_cmd_deg", DEFAULT_STARTUP_OUTER_COMMAND.pitch_cmd_deg)),
        )

    def _outer_config_from_state(self, state: dict[str, object]) -> AngleOuterLoopConfig:
        outer_config = state.get("outer_config", {}) if isinstance(state.get("outer_config", {}), dict) else {}
        return AngleOuterLoopConfig(
            kp_roll_angle=float(outer_config.get("kp_roll_angle", DEFAULT_STARTUP_OUTER_CONFIG.kp_roll_angle)),
            kp_pitch_angle=float(outer_config.get("kp_pitch_angle", DEFAULT_STARTUP_OUTER_CONFIG.kp_pitch_angle)),
            rate_limit_rad_s=float(outer_config.get("rate_limit_rad_s", DEFAULT_STARTUP_OUTER_CONFIG.rate_limit_rad_s)),
        )

    def _altitude_command_from_state(self, state: dict[str, object]) -> AltitudeCommand:
        altitude_command = state.get("altitude_command", {}) if isinstance(state.get("altitude_command", {}), dict) else {}
        return AltitudeCommand(
            alt_cmd_m=float(altitude_command.get("alt_cmd_m", DEFAULT_STARTUP_ALTITUDE_COMMAND.alt_cmd_m)),
            hover_throttle=float(altitude_command.get("hover_throttle", DEFAULT_STARTUP_ALTITUDE_COMMAND.hover_throttle)),
        )

    def _altitude_config_from_state(self, state: dict[str, object]) -> AltitudeControlConfig:
        altitude_config = state.get("altitude_config", {}) if isinstance(state.get("altitude_config", {}), dict) else {}
        return AltitudeControlConfig(
            kp_alt=float(altitude_config.get("kp_alt", DEFAULT_STARTUP_ALTITUDE_CONFIG.kp_alt)),
            vz_max=float(altitude_config.get("vz_max", DEFAULT_STARTUP_ALTITUDE_CONFIG.vz_max)),
            kp_vz=float(altitude_config.get("kp_vz", DEFAULT_STARTUP_ALTITUDE_CONFIG.kp_vz)),
            ki_vz=float(altitude_config.get("ki_vz", DEFAULT_STARTUP_ALTITUDE_CONFIG.ki_vz)),
            throttle_min=float(altitude_config.get("throttle_min", DEFAULT_STARTUP_ALTITUDE_CONFIG.throttle_min)),
            throttle_max=float(altitude_config.get("throttle_max", DEFAULT_STARTUP_ALTITUDE_CONFIG.throttle_max)),
        )

    def _persist_startup_state(self) -> None:
        payload = {
            "command": {
                "base_rpm": self._parse_float(self.base_rpm_var.get()),
                "p_cmd_rad_s": self._parse_float(self.p_cmd_var.get()),
                "q_cmd_rad_s": self._parse_float(self.q_cmd_var.get()),
                "kp_p": self._parse_float(self.kp_p_var.get()),
                "kp_q": self._parse_float(self.kp_q_var.get()),
                "output_limit": self._parse_float(self.output_limit_var.get()),
            },
            "angle_command": {
                "roll_cmd_deg": self._parse_float(self.roll_cmd_deg_var.get()),
                "pitch_cmd_deg": self._parse_float(self.pitch_cmd_deg_var.get()),
            },
            "outer_config": {
                "kp_roll_angle": self._parse_float(self.kp_roll_angle_var.get()),
                "kp_pitch_angle": self._parse_float(self.kp_pitch_angle_var.get()),
                "rate_limit_rad_s": self._parse_float(self.angle_rate_limit_var.get()),
            },
            "altitude_command": {
                "alt_cmd_m": self._parse_float(self.alt_cmd_m_var.get()),
                "hover_throttle": self._parse_float(self.hover_throttle_var.get()),
            },
            "altitude_config": {
                "kp_alt": self._parse_float(self.kp_alt_var.get()),
                "vz_max": self._parse_float(self.vz_max_var.get()),
                "kp_vz": self._parse_float(self.kp_vz_var.get()),
                "ki_vz": self._parse_float(self.ki_vz_var.get()),
                "throttle_min": self._parse_float(self.throttle_min_var.get()),
                "throttle_max": self._parse_float(self.throttle_max_var.get()),
            },
            "mixer_name": self.mixer_name_var.get(),
        }
        try:
            STARTUP_STATE_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        except Exception:
            pass

    def _apply_startup_defaults(self) -> None:
        try:
            self.apply_inner_loop_parameters()
            self.apply_outer_loop_parameters()
            self.apply_altitude_loop_parameters()
        except Exception:
            pass

        try:
            available_mixers = tuple(self.runtime.get_mixer_names())
            if self.mixer_name_var.get() in available_mixers:
                self.runtime.set_mixer_candidate(self.mixer_name_var.get())
            elif DEFAULT_STARTUP_MIXER_NAME in available_mixers:
                self.runtime.set_mixer_candidate(DEFAULT_STARTUP_MIXER_NAME)
                self.mixer_name_var.set(DEFAULT_STARTUP_MIXER_NAME)
            elif available_mixers:
                self.mixer_name_var.set(str(available_mixers[0]))
            self._persist_startup_state()
        except Exception:
            pass

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        top = ttk.Frame(outer)
        top.pack(fill="x", pady=(0, 8))
        ttk.Label(top, textvariable=self.status_var, font=gui_config.STATUS_FONT).pack(side="left", padx=(0, 16))
        ttk.Label(top, textvariable=self.connection_var, font=gui_config.STATUS_FONT).pack(side="left", padx=(0, 16))
        ttk.Label(top, textvariable=self.test_var, font=gui_config.STATUS_FONT).pack(side="left", padx=(0, 16))

        main = ttk.Frame(outer)
        main.pack(fill="both", expand=True)
        for column in range(4):
            main.columnconfigure(column, weight=1, uniform="main")

        col1 = ttk.Frame(main, padding=(0, 0, 6, 0))
        col2 = ttk.Frame(main, padding=(6, 0, 6, 0))
        col3 = ttk.Frame(main, padding=(6, 0, 6, 0))
        col4 = ttk.Frame(main, padding=(6, 0, 0, 0))
        col1.grid(row=0, column=0, sticky="nsew")
        col2.grid(row=0, column=1, sticky="nsew")
        col3.grid(row=0, column=2, sticky="nsew")
        col4.grid(row=0, column=3, sticky="nsew")

        self._build_connection_controls(col1)
        self._build_binding_panel(col1)
        self._build_inner_loop_controls(col1)
        self._build_outer_loop_controls(col2)
        self._build_altitude_loop_controls(col2)
        self._build_telemetry_panel(col3)
        self._build_controller_panel(col3)
        self._build_mixer_panel(col4)
        self._build_actuator_panel(col4)
    def _build_connection_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Connection / Binding Controls", padding=8)
        frame.pack(fill="x", pady=4)
        for text, command in (
            ("Connect", self.connect),
            ("Disconnect", self.disconnect),
            ("Bind Tags", self.bind),
            ("Initialize", self.initialize),
            ("Emergency Stop", self.emergency_stop),
        ):
            ttk.Button(frame, text=text, command=command).pack(side="left", padx=3)

    def _build_inner_loop_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Inner Loop Controls", padding=8)
        frame.pack(fill="x", pady=4)
        self._entry_row(frame, 0, "Base RPM (manual collective)", self.base_rpm_var)
        self._entry_row(frame, 1, "p_cmd (rad/s)", self.p_cmd_var)
        self._entry_row(frame, 2, "q_cmd (rad/s)", self.q_cmd_var)
        self._entry_row(frame, 3, "kp_p", self.kp_p_var)
        self._entry_row(frame, 4, "kp_q", self.kp_q_var)
        self._entry_row(frame, 5, "Output Limit", self.output_limit_var)
        ttk.Label(frame, text="Mixer Candidate").grid(row=6, column=0, sticky="w", pady=4)
        ttk.Combobox(frame, values=self.runtime.get_mixer_names(), textvariable=self.mixer_name_var, state="readonly", width=22).grid(row=6, column=1, sticky="ew", pady=4)
        row = ttk.Frame(frame)
        row.grid(row=7, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(row, text="Apply/Update Parameters", command=self.apply_inner_loop_parameters).pack(side="left", padx=3)
        ttk.Button(row, text="Start Test", command=self.start_test).pack(side="left", padx=3)
        ttk.Button(row, text="Stop Test", command=self.stop_test).pack(side="left", padx=3)
        frame.columnconfigure(1, weight=1)

    def _build_outer_loop_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Outer Loop Controls", padding=8)
        frame.pack(fill="x", pady=4)
        ttk.Label(frame, text="Roll/pitch angle targets feed the inner-loop rate controller only when this loop is started.", justify="left").pack(anchor="w", pady=(0, 8))
        inputs = ttk.Frame(frame)
        inputs.pack(fill="x", pady=(0, 8))
        self._entry_row(inputs, 0, "Roll Cmd (deg)", self.roll_cmd_deg_var)
        self._entry_row(inputs, 1, "Pitch Cmd (deg)", self.pitch_cmd_deg_var)
        self._entry_row(inputs, 2, "kp_roll_angle", self.kp_roll_angle_var)
        self._entry_row(inputs, 3, "kp_pitch_angle", self.kp_pitch_angle_var)
        self._entry_row(inputs, 4, "Rate Limit (rad/s)", self.angle_rate_limit_var)
        inputs.columnconfigure(1, weight=1)
        button_row = ttk.Frame(frame)
        button_row.pack(fill="x", pady=(0, 8))
        ttk.Button(button_row, text="Apply/Update Parameters", command=self.apply_outer_loop_parameters).pack(side="left", padx=3)
        ttk.Button(button_row, text="Start Outer Loop", command=self.start_outer_loop).pack(side="left", padx=3)
        ttk.Button(button_row, text="Stop Outer Loop", command=self.stop_outer_loop).pack(side="left", padx=3)
        info = ttk.Frame(frame)
        info.pack(fill="x")
        rows = [
            ("roll_error (deg)", "roll_error_deg"),
            ("pitch_error (deg)", "pitch_error_deg"),
            ("p_cmd_from_angle (rad/s)", "p_cmd_from_angle"),
            ("q_cmd_from_angle (rad/s)", "q_cmd_from_angle"),
        ]
        for idx, (label, key) in enumerate(rows):
            ttk.Label(info, text=label).grid(row=idx, column=0, sticky="w", padx=(0, 10), pady=2)
            ttk.Label(info, textvariable=self.outer_loop_vars[key], font=gui_config.MONO_FONT).grid(row=idx, column=1, sticky="w", pady=2)
        info.columnconfigure(1, weight=1)

    def _build_altitude_loop_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Vertical Speed Loop Controls", padding=8)
        frame.pack(fill="x", pady=4)
        ttk.Label(frame, text="This stage uses a direct vertical-speed command and a simple P collective controller. The altitude outer loop remains disabled.", justify="left").pack(anchor="w", pady=(0, 8))
        inputs = ttk.Frame(frame)
        inputs.pack(fill="x", pady=(0, 8))
        self._entry_row(inputs, 0, "Vz Cmd (m/s)", self.alt_cmd_m_var)
        self._entry_row(inputs, 1, "Hover Throttle", self.hover_throttle_var)
        self._entry_row(inputs, 2, "kp_alt (reserved)", self.kp_alt_var)
        self._entry_row(inputs, 3, "vz_max (reserved)", self.vz_max_var)
        self._entry_row(inputs, 4, "kp_vz", self.kp_vz_var)
        self._entry_row(inputs, 5, "ki_vz (unused)", self.ki_vz_var)
        self._entry_row(inputs, 6, "Throttle Min", self.throttle_min_var)
        self._entry_row(inputs, 7, "Throttle Max", self.throttle_max_var)
        inputs.columnconfigure(1, weight=1)
        button_row = ttk.Frame(frame)
        button_row.pack(fill="x", pady=(0, 8))
        ttk.Button(button_row, text="Apply/Update Parameters", command=self.apply_altitude_loop_parameters).pack(side="left", padx=3)
        ttk.Button(button_row, text="Start Vz Loop", command=self.start_altitude_loop).pack(side="left", padx=3)
        ttk.Button(button_row, text="Stop Vz Loop", command=self.stop_altitude_loop).pack(side="left", padx=3)
        info = ttk.Frame(frame)
        info.pack(fill="x")
        rows = [
            ("vz_cmd_input (m/s)", "alt_cmd_m"),
            ("altitude telemetry (m)", "alt_m"),
            ("vz telemetry (m/s)", "vz_m_s"),
            ("alt_error (unused)", "alt_error_m"),
            ("vz_cmd (m/s)", "vz_cmd_m_s"),
            ("vz_error (m/s)", "vz_error_m_s"),
            ("throttle_correction", "throttle_correction"),
            ("throttle_cmd", "throttle_cmd"),
        ]
        for idx, (label, key) in enumerate(rows):
            ttk.Label(info, text=label).grid(row=idx, column=0, sticky="w", padx=(0, 10), pady=2)
            ttk.Label(info, textvariable=self.altitude_loop_vars[key], font=gui_config.MONO_FONT).grid(row=idx, column=1, sticky="w", pady=2)
        info.columnconfigure(1, weight=1)

    def _build_binding_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Binding / Hardware Layer", padding=8)
        frame.pack(fill="x", pady=4)
        rows = [("Vessel", "vessel"), ("Initialized", "initialized"), ("Controller tags bound", "controller_tags"), ("Rotor tags bound", "rotor_tags")]
        for idx, (label, key) in enumerate(rows):
            ttk.Label(frame, text=label).grid(row=idx, column=0, sticky="w", padx=(0, 10), pady=3)
            ttk.Label(frame, textvariable=self.binding_vars[key], font=gui_config.MONO_FONT).grid(row=idx, column=1, sticky="w", pady=3)

    def _build_telemetry_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Telemetry Layer", padding=8)
        frame.pack(fill="x", pady=4)
        angle_frame = ttk.LabelFrame(frame, text="RPY Angles", padding=8)
        angle_frame.pack(fill="x", pady=4)
        self._label_rows(angle_frame, [("Roll", "roll_deg"), ("Pitch", "pitch_deg"), ("Yaw / Heading", "heading_deg")], value_suffix=" deg", variable_group=self.telemetry_vars)
        altitude_frame = ttk.LabelFrame(frame, text="Altitude", padding=8)
        altitude_frame.pack(fill="x", pady=4)
        self._label_rows(altitude_frame, [("Altitude", "alt_m"), ("Vertical Speed", "vz_m_s")], value_suffix="", variable_group=self.telemetry_vars)
        body_rate_frame = ttk.LabelFrame(frame, text="Body Rates (FRD)", padding=8)
        body_rate_frame.pack(fill="x", pady=4)
        self._label_rows(body_rate_frame, [("p", "p_meas"), ("q", "q_meas"), ("r", "r_meas")], value_suffix=" rad/s", variable_group=self.telemetry_vars)
        euler_rate_frame = ttk.LabelFrame(frame, text="Euler Rates", padding=8)
        euler_rate_frame.pack(fill="x", pady=4)
        self._label_rows(euler_rate_frame, [("phi_dot", "roll_rate_deg_s"), ("theta_dot", "pitch_rate_deg_s"), ("psi_dot", "yaw_rate_deg_s")], value_suffix=" deg/s", variable_group=self.telemetry_vars)
        timing_frame = ttk.LabelFrame(frame, text="Timing", padding=8)
        timing_frame.pack(fill="x", pady=4)
        self._label_rows(timing_frame, [("dt", "dt_s")], value_suffix=" s", variable_group=self.telemetry_vars)
    def _build_controller_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Controller Layer", padding=8)
        frame.pack(fill="x", pady=4)
        roll_frame = ttk.LabelFrame(frame, text="Roll Channel", padding=8)
        roll_frame.pack(fill="x", pady=4)
        self._label_rows(roll_frame, [("p_cmd", "p_cmd"), ("error_p", "error_p"), ("kp_p", "kp_p"), ("u_roll", "u_roll")], value_suffix="", variable_group=self.controller_vars)
        pitch_frame = ttk.LabelFrame(frame, text="Pitch Channel", padding=8)
        pitch_frame.pack(fill="x", pady=4)
        self._label_rows(pitch_frame, [("q_cmd", "q_cmd"), ("error_q", "error_q"), ("kp_q", "kp_q"), ("u_pitch", "u_pitch")], value_suffix="", variable_group=self.controller_vars)
        common_frame = ttk.LabelFrame(frame, text="Common", padding=8)
        common_frame.pack(fill="x", pady=4)
        self._label_rows(common_frame, [("output_limit", "output_limit")], value_suffix="", variable_group=self.controller_vars)

    def _build_mixer_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Mixer Layer", padding=8)
        frame.pack(fill="both", expand=True, pady=4)
        ttk.Label(frame, text="Current mixer").grid(row=0, column=0, sticky="w", padx=(0, 10), pady=3)
        ttk.Label(frame, textvariable=self.mixer_vars["name"], font=gui_config.MONO_FONT).grid(row=0, column=1, sticky="w", pady=3)
        ttk.Label(frame, text="Mixer matrix").grid(row=1, column=0, sticky="nw", padx=(0, 10), pady=3)
        self.matrix_text = scrolledtext.ScrolledText(frame, height=gui_config.MATRIX_TEXT_HEIGHT, width=gui_config.MATRIX_TEXT_WIDTH, wrap=tk.WORD, font=gui_config.MONO_FONT)
        self.matrix_text.grid(row=1, column=1, sticky="nsew", pady=3)
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(1, weight=1)

    def _build_actuator_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Actuator Output Layer", padding=8)
        frame.pack(fill="x", pady=4)
        positions = {"FL": (0, 0), "FR": (0, 1), "RL": (1, 0), "RR": (1, 1)}
        for key, (row, column) in positions.items():
            card = ttk.LabelFrame(frame, text=key, padding=8)
            card.grid(row=row, column=column, sticky="nsew", padx=6, pady=6)
            ttk.Label(card, text=f"{key} Motor", font=gui_config.MONO_FONT).pack(pady=(0, 4))
            ttk.Label(card, textvariable=self.actuator_vars[key], font=gui_config.MONO_FONT).pack(pady=(0, 6))
            bar = ttk.Progressbar(card, orient="vertical", mode="determinate", maximum=RPM_MAX, length=140)
            bar.pack(fill="y", expand=True, pady=4)
            self.motor_bars[key] = bar
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

    def _entry_row(self, parent: ttk.Frame, row: int, label: str, variable: tk.StringVar) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(parent, textvariable=variable, width=gui_config.NUMERIC_ENTRY_WIDTH).grid(row=row, column=1, sticky="ew", pady=4)

    def _label_rows(self, parent: ttk.Frame, rows: list[tuple[str, str]], *, value_suffix: str, variable_group: dict[str, tk.StringVar]) -> None:
        for idx, (label, key) in enumerate(rows):
            ttk.Label(parent, text=label).grid(row=idx, column=0, sticky="w", padx=(0, 10), pady=2)
            ttk.Label(parent, textvariable=variable_group[key], font=gui_config.MONO_FONT).grid(row=idx, column=1, sticky="w", pady=2)
            if value_suffix.strip():
                ttk.Label(parent, text=value_suffix.strip()).grid(row=idx, column=2, sticky="w", pady=2)

    def connect(self) -> None:
        self._safe_action(self.runtime.connect)

    def disconnect(self) -> None:
        self._safe_action(self.runtime.disconnect)

    def bind(self) -> None:
        self._safe_action(self.runtime.bind)

    def initialize(self) -> None:
        self._safe_action(self.runtime.initialize)

    def emergency_stop(self) -> None:
        self._safe_action(self.runtime.emergency_stop)

    def apply_inner_loop_parameters(self) -> None:
        self.runtime.set_command(RollRateTestCommand(
            base_rpm=self._parse_float(self.base_rpm_var.get()),
            p_cmd_rad_s=self._parse_float(self.p_cmd_var.get()),
            q_cmd_rad_s=self._parse_float(self.q_cmd_var.get()),
            kp_p=self._parse_float(self.kp_p_var.get()),
            kp_q=self._parse_float(self.kp_q_var.get()),
            output_limit=self._parse_float(self.output_limit_var.get()),
        ))
        self.runtime.set_mixer_candidate(self.mixer_name_var.get())
        self._persist_startup_state()

    def apply_outer_loop_parameters(self) -> None:
        self.runtime.set_angle_command(AngleCommand(
            roll_cmd_deg=self._parse_float(self.roll_cmd_deg_var.get()),
            pitch_cmd_deg=self._parse_float(self.pitch_cmd_deg_var.get()),
        ))
        self.runtime.set_outer_loop_config(AngleOuterLoopConfig(
            kp_roll_angle=self._parse_float(self.kp_roll_angle_var.get()),
            kp_pitch_angle=self._parse_float(self.kp_pitch_angle_var.get()),
            rate_limit_rad_s=self._parse_float(self.angle_rate_limit_var.get()),
        ))
        self._persist_startup_state()

    def apply_altitude_loop_parameters(self) -> None:
        self.runtime.set_altitude_command(AltitudeCommand(
            alt_cmd_m=self._parse_float(self.alt_cmd_m_var.get()),
            hover_throttle=self._parse_float(self.hover_throttle_var.get()),
        ))
        self.runtime.set_altitude_config(AltitudeControlConfig(
            kp_alt=self._parse_float(self.kp_alt_var.get()),
            vz_max=self._parse_float(self.vz_max_var.get()),
            kp_vz=self._parse_float(self.kp_vz_var.get()),
            ki_vz=self._parse_float(self.ki_vz_var.get()),
            throttle_min=self._parse_float(self.throttle_min_var.get()),
            throttle_max=self._parse_float(self.throttle_max_var.get()),
        ))
        self._persist_startup_state()

    def start_test(self) -> None:
        def _start() -> None:
            self.apply_inner_loop_parameters()
            self.runtime.start_test()
        self._safe_action(_start)

    def stop_test(self) -> None:
        self._safe_action(self.runtime.stop_test)

    def start_outer_loop(self) -> None:
        def _start() -> None:
            self.apply_outer_loop_parameters()
            self.runtime.start_outer_loop()
        self._safe_action(_start)

    def stop_outer_loop(self) -> None:
        self._safe_action(self.runtime.stop_outer_loop)

    def start_altitude_loop(self) -> None:
        def _start() -> None:
            self.apply_altitude_loop_parameters()
            self.runtime.start_altitude_loop()
        self._safe_action(_start)

    def stop_altitude_loop(self) -> None:
        self._safe_action(self.runtime.stop_altitude_loop)
    def _safe_action(self, action) -> None:
        try:
            action()
        except Exception as exc:
            self.status_var.set(f"Error: {exc}")
            messagebox.showerror("Roll / Pitch Inner-Loop Test", str(exc))
        finally:
            self._render_snapshot(self.runtime.get_snapshot())

    def _schedule_loop(self) -> None:
        self._after_id = self.root.after(gui_config.GUI_STEP_MS, self._update_loop)

    def _update_loop(self) -> None:
        try:
            snapshot = self.runtime.get_snapshot()
            if getattr(snapshot.binding, "test_running", False):
                self.runtime.step()
            elif getattr(snapshot.binding, "connected", False):
                self.runtime.preview()
        except Exception as exc:
            self.status_var.set(f"Loop error: {exc}")
            self.runtime.stop_test()
            messagebox.showerror("Roll / Pitch Inner-Loop Test", str(exc))
        self._render_snapshot(self.runtime.get_snapshot())
        self._schedule_loop()

    def _render_snapshot(self, snapshot) -> None:
        binding = getattr(snapshot, "binding", None)
        telemetry = getattr(snapshot, "telemetry", None)
        controller = getattr(snapshot, "controller", None)
        command = getattr(snapshot, "command", None)
        outer_loop = getattr(snapshot, "outer_loop", None)
        altitude_loop = getattr(snapshot, "altitude_loop", None)
        motor_command = getattr(snapshot, "motor_command", None)

        self.status_var.set(getattr(snapshot, "last_status", "Idle"))
        self.connection_var.set("Connected" if getattr(binding, "connected", False) else "Disconnected")
        self.test_var.set("Running" if getattr(binding, "test_running", False) else "Stopped")

        self.binding_vars["vessel"].set(getattr(binding, "vessel_name", "-"))
        self.binding_vars["initialized"].set(str(getattr(binding, "initialized", False)))
        self.binding_vars["controller_tags"].set(self._format_tag_map(getattr(snapshot, "controller_tags_bound", {})))
        self.binding_vars["rotor_tags"].set(self._format_tag_map(getattr(snapshot, "rotor_tags_bound", {})))

        self.telemetry_vars["roll_deg"].set(f"{getattr(telemetry, 'roll_deg', 0.0):.3f}")
        self.telemetry_vars["pitch_deg"].set(f"{getattr(telemetry, 'pitch_deg', 0.0):.3f}")
        self.telemetry_vars["heading_deg"].set(f"{getattr(telemetry, 'heading_deg', 0.0):.3f}")
        self.telemetry_vars["alt_m"].set(f"{getattr(telemetry, 'alt_m', 0.0):.3f}")
        self.telemetry_vars["vz_m_s"].set(f"{getattr(telemetry, 'vz_m_s', 0.0):.3f}")
        self.telemetry_vars["p_meas"].set(f"{getattr(telemetry, 'p_meas_rad_s', 0.0):.4f}")
        self.telemetry_vars["q_meas"].set(f"{getattr(telemetry, 'q_meas_rad_s', 0.0):.4f}")
        self.telemetry_vars["r_meas"].set(f"{getattr(telemetry, 'r_meas_rad_s', 0.0):.4f}")
        self.telemetry_vars["roll_rate_deg_s"].set(f"{getattr(telemetry, 'roll_rate_deg_s', 0.0):.3f}")
        self.telemetry_vars["pitch_rate_deg_s"].set(f"{getattr(telemetry, 'pitch_rate_deg_s', 0.0):.3f}")
        self.telemetry_vars["yaw_rate_deg_s"].set(f"{getattr(telemetry, 'yaw_rate_deg_s', 0.0):.3f}")
        self.telemetry_vars["dt_s"].set(f"{getattr(telemetry, 'dt_s', 0.0):.4f}")

        self.outer_loop_vars["roll_error_deg"].set(f"{getattr(outer_loop, 'roll_error_deg', 0.0):.3f}")
        self.outer_loop_vars["pitch_error_deg"].set(f"{getattr(outer_loop, 'pitch_error_deg', 0.0):.3f}")
        self.outer_loop_vars["p_cmd_from_angle"].set(f"{getattr(outer_loop, 'p_cmd_rad_s', 0.0):.4f}")
        self.outer_loop_vars["q_cmd_from_angle"].set(f"{getattr(outer_loop, 'q_cmd_rad_s', 0.0):.4f}")

        self.altitude_loop_vars["alt_cmd_m"].set(f"{getattr(altitude_loop, 'alt_cmd_m', 0.0):.3f}")
        self.altitude_loop_vars["alt_m"].set(f"{getattr(altitude_loop, 'alt_m', 0.0):.3f}")
        self.altitude_loop_vars["vz_m_s"].set(f"{getattr(altitude_loop, 'vz_m_s', 0.0):.3f}")
        self.altitude_loop_vars["alt_error_m"].set(f"{getattr(altitude_loop, 'alt_error_m', 0.0):.3f}")
        self.altitude_loop_vars["vz_cmd_m_s"].set(f"{getattr(altitude_loop, 'vz_cmd_m_s', 0.0):.3f}")
        self.altitude_loop_vars["vz_error_m_s"].set(f"{getattr(altitude_loop, 'vz_error_m_s', 0.0):.3f}")
        self.altitude_loop_vars["throttle_correction"].set(f"{getattr(altitude_loop, 'throttle_correction', 0.0):.3f}")
        self.altitude_loop_vars["throttle_cmd"].set(f"{getattr(altitude_loop, 'throttle_cmd', 0.0):.3f}")

        p_cmd_display = getattr(outer_loop, 'p_cmd_rad_s', 0.0) if getattr(snapshot, 'outer_loop_running', False) else getattr(command, 'p_cmd_rad_s', 0.0)
        q_cmd_display = getattr(outer_loop, 'q_cmd_rad_s', 0.0) if getattr(snapshot, 'outer_loop_running', False) else getattr(command, 'q_cmd_rad_s', 0.0)
        self.controller_vars["p_cmd"].set(f"{p_cmd_display:.4f}")
        self.controller_vars["error_p"].set(f"{getattr(controller, 'error_p_rad_s', 0.0):.4f}")
        self.controller_vars["kp_p"].set(f"{getattr(command, 'kp_p', 0.0):.4f}")
        self.controller_vars["u_roll"].set(f"{getattr(controller, 'u_roll', 0.0):.4f}")
        self.controller_vars["q_cmd"].set(f"{q_cmd_display:.4f}")
        self.controller_vars["error_q"].set(f"{getattr(controller, 'error_q_rad_s', 0.0):.4f}")
        self.controller_vars["kp_q"].set(f"{getattr(command, 'kp_q', 0.0):.4f}")
        self.controller_vars["u_pitch"].set(f"{getattr(controller, 'u_pitch', 0.0):.4f}")
        self.controller_vars["output_limit"].set(f"{getattr(command, 'output_limit', 0.0):.4f}")

        self.mixer_vars["name"].set(getattr(snapshot, "mixer_name", "-"))
        self.matrix_text.delete("1.0", tk.END)
        self.matrix_text.insert(tk.END, getattr(snapshot, "mixer_matrix_text", ""))
        mapping = motor_command.as_mapping() if motor_command is not None else {}
        for key in ("FL", "FR", "RL", "RR"):
            rpm_value = float(mapping.get(key, 0.0))
            self.actuator_vars[key].set(f"{rpm_value:.2f} RPM")
            if key in self.motor_bars:
                self.motor_bars[key]["value"] = rpm_value

    def _format_tag_map(self, tags: object) -> str:
        if not isinstance(tags, dict) or not tags:
            return "-"
        return ", ".join(f"{key}:{value}" for key, value in tags.items())

    def _parse_float(self, value: str) -> float:
        return float(value.strip())

    def on_close(self) -> None:
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
        try:
            self._persist_startup_state()
            self.runtime.disconnect()
        finally:
            self.root.destroy()
