from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

from p_inner_loop_test_v1.data_api.models import RPM_MAX, RollRateTestCommand


DEFAULT_STARTUP_COMMAND = RollRateTestCommand(
    base_rpm=288.0,
    p_cmd_rad_s=0.0,
    q_cmd_rad_s=0.0,
    kp_p=40.0,
    kp_q=40.0,
    output_limit=50.0,
)
DEFAULT_STARTUP_MIXER_NAME = "quad_x_roll_pitch_a"
from p_inner_loop_test_v1.gui import gui_config
from p_inner_loop_test_v1.runtime.test_runtime import RollRateInnerLoopRuntime


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
        startup_command = DEFAULT_STARTUP_COMMAND
        startup_mixer_name = DEFAULT_STARTUP_MIXER_NAME

        self.base_rpm_var = tk.StringVar(value=f"{startup_command.base_rpm:.1f}")
        self.p_cmd_var = tk.StringVar(value=f"{startup_command.p_cmd_rad_s:.3f}")
        self.q_cmd_var = tk.StringVar(value=f"{startup_command.q_cmd_rad_s:.3f}")
        self.kp_p_var = tk.StringVar(value=f"{startup_command.kp_p:.3f}")
        self.kp_q_var = tk.StringVar(value=f"{startup_command.kp_q:.3f}")
        self.output_limit_var = tk.StringVar(value=f"{startup_command.output_limit:.3f}")
        self.mixer_name_var = tk.StringVar(value=startup_mixer_name)

        # Outer-loop placeholder inputs. These are intentionally not wired yet.
        self.roll_cmd_deg_var = tk.StringVar(value="0.0")
        self.pitch_cmd_deg_var = tk.StringVar(value="0.0")
        self.kp_roll_angle_var = tk.StringVar(value="0.0")
        self.kp_pitch_angle_var = tk.StringVar(value="0.0")
        self.angle_rate_limit_var = tk.StringVar(value="1.0")

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
        self.mixer_vars = {
            "name": tk.StringVar(value=snapshot.mixer_name),
        }
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

    def _apply_startup_defaults(self) -> None:
        """Preload the GUI and runtime with the preferred inner-loop test defaults."""
        try:
            self.runtime.set_command(DEFAULT_STARTUP_COMMAND)
        except Exception:
            pass

        try:
            available_mixers = tuple(self.runtime.get_mixer_names())
            if DEFAULT_STARTUP_MIXER_NAME in available_mixers:
                self.runtime.set_mixer_candidate(DEFAULT_STARTUP_MIXER_NAME)
                self.mixer_name_var.set(DEFAULT_STARTUP_MIXER_NAME)
            elif available_mixers:
                self.mixer_name_var.set(str(available_mixers[0]))
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

        main = ttk.Panedwindow(outer, orient=tk.HORIZONTAL)
        main.pack(fill="both", expand=True)

        left = ttk.Frame(main, padding=(0, 0, 6, 0))
        middle = ttk.Frame(main, padding=(6, 0, 6, 0))
        right = ttk.Frame(main, padding=(6, 0, 0, 0))
        main.add(left, weight=2)
        main.add(middle, weight=2)
        main.add(right, weight=2)

        self._build_connection_controls(left)
        self._build_inner_loop_controls(left)
        self._build_binding_panel(left)
        self._build_outer_loop_controls(left)
        self._build_telemetry_panel(middle)
        self._build_controller_panel(middle)
        self._build_mixer_panel(right)
        self._build_actuator_panel(right)

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

        self._entry_row(frame, 0, "Base RPM", self.base_rpm_var)
        self._entry_row(frame, 1, "p_cmd (rad/s)", self.p_cmd_var)
        self._entry_row(frame, 2, "q_cmd (rad/s)", self.q_cmd_var)
        self._entry_row(frame, 3, "kp_p", self.kp_p_var)
        self._entry_row(frame, 4, "kp_q", self.kp_q_var)
        self._entry_row(frame, 5, "Output Limit", self.output_limit_var)

        ttk.Label(frame, text="Mixer Candidate").grid(row=6, column=0, sticky="w", pady=4)
        ttk.Combobox(
            frame,
            values=self.runtime.get_mixer_names(),
            textvariable=self.mixer_name_var,
            state="readonly",
            width=22,
        ).grid(row=6, column=1, sticky="ew", pady=4)

        row = ttk.Frame(frame)
        row.grid(row=7, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(row, text="Apply/Update Parameters", command=self.apply_parameters).pack(side="left", padx=3)
        ttk.Button(row, text="Start Test", command=self.start_test).pack(side="left", padx=3)
        ttk.Button(row, text="Stop Test", command=self.stop_test).pack(side="left", padx=3)
        frame.columnconfigure(1, weight=1)

    def _build_outer_loop_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Outer Loop Controls", padding=8)
        frame.pack(fill="x", pady=4)

        ttk.Label(
            frame,
            text="Input placeholder only.\nOuter-loop angle control is not wired yet.",
            justify="left",
        ).pack(anchor="w", pady=(0, 8))

        inputs = ttk.Frame(frame)
        inputs.pack(fill="x", pady=(0, 8))

        self._entry_row(inputs, 0, "Roll Cmd (deg)", self.roll_cmd_deg_var)
        self._entry_row(inputs, 1, "Pitch Cmd (deg)", self.pitch_cmd_deg_var)
        self._entry_row(inputs, 2, "kp_roll_angle", self.kp_roll_angle_var)
        self._entry_row(inputs, 3, "kp_pitch_angle", self.kp_pitch_angle_var)
        self._entry_row(inputs, 4, "Rate Limit (rad/s)", self.angle_rate_limit_var)
        inputs.columnconfigure(1, weight=1)

        info = ttk.Frame(frame)
        info.pack(fill="x")

        rows = [
            ("roll_error (deg)", "-"),
            ("pitch_error (deg)", "-"),
            ("p_cmd_from_angle (rad/s)", "-"),
            ("q_cmd_from_angle (rad/s)", "-"),
        ]

        for idx, (label, value) in enumerate(rows):
            ttk.Label(info, text=label).grid(row=idx, column=0, sticky="w", padx=(0, 10), pady=2)
            ttk.Label(info, text=value, font=gui_config.MONO_FONT).grid(
                row=idx,
                column=1,
                sticky="w",
                pady=2,
            )

        info.columnconfigure(1, weight=1)

    def _build_binding_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Binding / Hardware Layer", padding=8)
        frame.pack(fill="x", pady=4)
        rows = [
            ("Vessel", "vessel"),
            ("Initialized", "initialized"),
            ("Controller tags bound", "controller_tags"),
            ("Rotor tags bound", "rotor_tags"),
        ]
        for idx, (label, key) in enumerate(rows):
            ttk.Label(frame, text=label).grid(row=idx, column=0, sticky="w", padx=(0, 10), pady=3)
            ttk.Label(frame, textvariable=self.binding_vars[key], font=gui_config.MONO_FONT).grid(
                row=idx,
                column=1,
                sticky="w",
                pady=3,
            )

    def _build_telemetry_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Telemetry Layer", padding=8)
        frame.pack(fill="x", pady=4)

        angle_frame = ttk.LabelFrame(frame, text="RPY Angles", padding=8)
        angle_frame.pack(fill="x", pady=4)
        self._label_rows(
            angle_frame,
            [
                ("Roll", "roll_deg"),
                ("Pitch", "pitch_deg"),
                ("Yaw / Heading", "heading_deg"),
            ],
            value_suffix=" deg",
            variable_group=self.telemetry_vars,
        )

        body_rate_frame = ttk.LabelFrame(frame, text="Body Rates (FRD)", padding=8)
        body_rate_frame.pack(fill="x", pady=4)
        self._label_rows(
            body_rate_frame,
            [
                ("p", "p_meas"),
                ("q", "q_meas"),
                ("r", "r_meas"),
            ],
            value_suffix=" rad/s",
            variable_group=self.telemetry_vars,
        )

        euler_rate_frame = ttk.LabelFrame(frame, text="Euler Rates", padding=8)
        euler_rate_frame.pack(fill="x", pady=4)
        self._label_rows(
            euler_rate_frame,
            [
                ("phi_dot", "roll_rate_deg_s"),
                ("theta_dot", "pitch_rate_deg_s"),
                ("psi_dot", "yaw_rate_deg_s"),
            ],
            value_suffix=" deg/s",
            variable_group=self.telemetry_vars,
        )

        timing_frame = ttk.LabelFrame(frame, text="Timing", padding=8)
        timing_frame.pack(fill="x", pady=4)
        self._label_rows(
            timing_frame,
            [("dt", "dt_s")],
            value_suffix=" s",
            variable_group=self.telemetry_vars,
        )

    def _build_controller_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Controller Layer", padding=8)
        frame.pack(fill="x", pady=4)

        roll_frame = ttk.LabelFrame(frame, text="Roll Channel", padding=8)
        roll_frame.pack(fill="x", pady=4)
        self._label_rows(
            roll_frame,
            [
                ("p_cmd", "p_cmd"),
                ("error_p", "error_p"),
                ("kp_p", "kp_p"),
                ("u_roll", "u_roll"),
            ],
            value_suffix="",
            variable_group=self.controller_vars,
        )

        pitch_frame = ttk.LabelFrame(frame, text="Pitch Channel", padding=8)
        pitch_frame.pack(fill="x", pady=4)
        self._label_rows(
            pitch_frame,
            [
                ("q_cmd", "q_cmd"),
                ("error_q", "error_q"),
                ("kp_q", "kp_q"),
                ("u_pitch", "u_pitch"),
            ],
            value_suffix="",
            variable_group=self.controller_vars,
        )

        common_frame = ttk.LabelFrame(frame, text="Common", padding=8)
        common_frame.pack(fill="x", pady=4)
        self._label_rows(
            common_frame,
            [("output_limit", "output_limit")],
            value_suffix="",
            variable_group=self.controller_vars,
        )

    def _build_mixer_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Mixer Layer", padding=8)
        frame.pack(fill="both", expand=True, pady=4)
        ttk.Label(frame, text="Current mixer").grid(row=0, column=0, sticky="w", padx=(0, 10), pady=3)
        ttk.Label(frame, textvariable=self.mixer_vars["name"], font=gui_config.MONO_FONT).grid(
            row=0,
            column=1,
            sticky="w",
            pady=3,
        )
        ttk.Label(frame, text="Mixer matrix").grid(row=1, column=0, sticky="nw", padx=(0, 10), pady=3)
        self.matrix_text = scrolledtext.ScrolledText(
            frame,
            height=gui_config.MATRIX_TEXT_HEIGHT,
            width=gui_config.MATRIX_TEXT_WIDTH,
            wrap=tk.WORD,
            font=gui_config.MONO_FONT,
        )
        self.matrix_text.grid(row=1, column=1, sticky="nsew", pady=3)
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(1, weight=1)

    def _build_actuator_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Actuator Output Layer", padding=8)
        frame.pack(fill="x", pady=4)

        positions = {
            "FL": (0, 0),
            "FR": (0, 1),
            "RL": (1, 0),
            "RR": (1, 1),
        }
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
        ttk.Entry(parent, textvariable=variable, width=gui_config.NUMERIC_ENTRY_WIDTH).grid(
            row=row,
            column=1,
            sticky="ew",
            pady=4,
        )

    def _label_rows(
        self,
        parent: ttk.Frame,
        rows: list[tuple[str, str]],
        *,
        value_suffix: str,
        variable_group: dict[str, tk.StringVar],
    ) -> None:
        for idx, (label, key) in enumerate(rows):
            ttk.Label(parent, text=label).grid(row=idx, column=0, sticky="w", padx=(0, 10), pady=2)
            ttk.Label(parent, textvariable=variable_group[key], font=gui_config.MONO_FONT).grid(
                row=idx,
                column=1,
                sticky="w",
                pady=2,
            )
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

    def apply_parameters(self) -> None:
        def _apply() -> None:
            self.runtime.set_command(
                RollRateTestCommand(
                    base_rpm=self._parse_float(self.base_rpm_var.get()),
                    p_cmd_rad_s=self._parse_float(self.p_cmd_var.get()),
                    q_cmd_rad_s=self._parse_float(self.q_cmd_var.get()),
                    kp_p=self._parse_float(self.kp_p_var.get()),
                    kp_q=self._parse_float(self.kp_q_var.get()),
                    output_limit=self._parse_float(self.output_limit_var.get()),
                )
            )
            self.runtime.set_mixer_candidate(self.mixer_name_var.get())

        self._safe_action(_apply)

    def start_test(self) -> None:
        def _start() -> None:
            self.apply_parameters()
            self.runtime.start_test()

        self._safe_action(_start)

    def stop_test(self) -> None:
        self._safe_action(self.runtime.stop_test)

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
            if self.runtime.get_snapshot().binding.test_running:
                self.runtime.step()
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
        self.telemetry_vars["p_meas"].set(f"{getattr(telemetry, 'p_meas_rad_s', 0.0):.4f}")
        self.telemetry_vars["q_meas"].set(f"{getattr(telemetry, 'q_meas_rad_s', 0.0):.4f}")
        self.telemetry_vars["r_meas"].set(f"{getattr(telemetry, 'r_meas_rad_s', 0.0):.4f}")
        self.telemetry_vars["roll_rate_deg_s"].set(f"{getattr(telemetry, 'roll_rate_deg_s', 0.0):.3f}")
        self.telemetry_vars["pitch_rate_deg_s"].set(f"{getattr(telemetry, 'pitch_rate_deg_s', 0.0):.3f}")
        self.telemetry_vars["yaw_rate_deg_s"].set(f"{getattr(telemetry, 'yaw_rate_deg_s', 0.0):.3f}")
        self.telemetry_vars["dt_s"].set(f"{getattr(telemetry, 'dt_s', 0.0):.4f}")

        self.controller_vars["p_cmd"].set(f"{getattr(command, 'p_cmd_rad_s', 0.0):.4f}")
        self.controller_vars["error_p"].set(f"{getattr(controller, 'error_p_rad_s', 0.0):.4f}")
        self.controller_vars["kp_p"].set(f"{getattr(command, 'kp_p', 0.0):.4f}")
        self.controller_vars["u_roll"].set(f"{getattr(controller, 'u_roll', 0.0):.4f}")
        self.controller_vars["q_cmd"].set(f"{getattr(command, 'q_cmd_rad_s', 0.0):.4f}")
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
            self.runtime.disconnect()
        finally:
            self.root.destroy()
