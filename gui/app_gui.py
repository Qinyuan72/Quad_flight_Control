from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

try:
    from data_api.models import (
        AltitudeCommand,
        AltitudeControlConfig,
        AngleCommand,
        AngleOuterLoopConfig,
        BodyVelocityCommand,
        BodyVelocityOuterLoopConfig,
        UneToControlPlaneConfig,
        RPM_MAX,
        RollRateTestCommand,
        WorldCommandPreprocessConfig,
        WorldVelocityCommand,
        YawOuterLoopConfig,
    )
    from runtime.runtime_service import RuntimeService
except ImportError:
    from ..data_api.models import (
        AltitudeCommand,
        AltitudeControlConfig,
        AngleCommand,
        AngleOuterLoopConfig,
        BodyVelocityCommand,
        BodyVelocityOuterLoopConfig,
        UneToControlPlaneConfig,
        RPM_MAX,
        RollRateTestCommand,
        WorldCommandPreprocessConfig,
        WorldVelocityCommand,
        YawOuterLoopConfig,
    )
    from ..runtime.runtime_service import RuntimeService

from . import gui_config


class RollRateTestApp:
    """Tkinter GUI for the focused roll / pitch inner-loop test harness."""

    def __init__(self, root: tk.Tk, runtime: RuntimeService) -> None:
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
        startup_command = snapshot.command
        startup_outer_command = snapshot.angle_command
        startup_body_velocity_command = snapshot.body_velocity_command
        startup_world_velocity_command = snapshot.world_velocity_command
        startup_world_command_preprocess_config = getattr(snapshot, "world_command_preprocess_config", WorldCommandPreprocessConfig())
        startup_une_to_control_plane_config = snapshot.une_to_control_plane_config
        startup_world_velocity_feedback_mode = getattr(snapshot, "world_velocity_feedback_mode", "raw_body")
        startup_outer_config = snapshot.outer_loop_config
        startup_body_velocity_outer_config = snapshot.body_velocity_outer_loop_config
        startup_yaw_outer_config = snapshot.yaw_outer_loop_config
        startup_altitude_command = snapshot.altitude_command
        startup_altitude_config = snapshot.altitude_control_config
        startup_mixer_name = snapshot.mixer_name

        self.base_rpm_var = tk.StringVar(value=f"{startup_command.base_rpm:.1f}")
        self.p_cmd_var = tk.StringVar(value=f"{startup_command.p_cmd_rad_s:.3f}")
        self.q_cmd_var = tk.StringVar(value=f"{startup_command.q_cmd_rad_s:.3f}")
        self.r_cmd_var = tk.StringVar(value=f"{startup_command.r_cmd_rad_s:.3f}")
        self.kp_p_var = tk.StringVar(value=f"{startup_command.kp_p:.3f}")
        self.kp_q_var = tk.StringVar(value=f"{startup_command.kp_q:.3f}")
        self.kp_r_var = tk.StringVar(value=f"{startup_command.kp_r:.3f}")
        self.ki_p_var = tk.StringVar(value=f"{startup_command.ki_p:.3f}")
        self.ki_q_var = tk.StringVar(value=f"{startup_command.ki_q:.3f}")
        self.ki_r_var = tk.StringVar(value=f"{startup_command.ki_r:.3f}")
        self.kd_p_var = tk.StringVar(value=f"{startup_command.kd_p:.3f}")
        self.kd_q_var = tk.StringVar(value=f"{startup_command.kd_q:.3f}")
        self.kd_r_var = tk.StringVar(value=f"{startup_command.kd_r:.3f}")
        self.int_limit_p_var = tk.StringVar(value=f"{startup_command.integrator_limit_p:.3f}")
        self.int_limit_q_var = tk.StringVar(value=f"{startup_command.integrator_limit_q:.3f}")
        self.int_limit_r_var = tk.StringVar(value=f"{startup_command.integrator_limit_r:.3f}")
        self.output_limit_var = tk.StringVar(value=f"{startup_command.output_limit:.3f}")
        self.mixer_name_var = tk.StringVar(value=startup_mixer_name)

        self.roll_cmd_deg_var = tk.StringVar(value=f"{startup_outer_command.roll_cmd_deg:.3f}")
        self.pitch_cmd_deg_var = tk.StringVar(value=f"{startup_outer_command.pitch_cmd_deg:.3f}")
        self.yaw_cmd_deg_var = tk.StringVar(value=f"{startup_outer_command.yaw_cmd_deg:.3f}")
        self.v_forward_cmd_var = tk.StringVar(value=f"{startup_body_velocity_command.v_forward_cmd_m_s:.3f}")
        self.v_right_cmd_var = tk.StringVar(value=f"{startup_body_velocity_command.v_right_cmd_m_s:.3f}")
        self.v_north_cmd_var = tk.StringVar(value=f"{startup_world_velocity_command.v_north_cmd_m_s:.3f}")
        self.v_east_cmd_var = tk.StringVar(value=f"{startup_world_velocity_command.v_east_cmd_m_s:.3f}")
        self.invert_world_north_var = tk.BooleanVar(value=startup_world_command_preprocess_config.invert_world_north)
        self.invert_world_east_var = tk.BooleanVar(value=startup_world_command_preprocess_config.invert_world_east)
        self.swap_ne_var = tk.BooleanVar(value=startup_une_to_control_plane_config.swap_ne)
        self.invert_x_var = tk.BooleanVar(value=startup_une_to_control_plane_config.sign_x < 0.0)
        self.invert_y_var = tk.BooleanVar(value=startup_une_to_control_plane_config.sign_y < 0.0)
        self.world_feedback_mode_var = tk.StringVar(value=startup_world_velocity_feedback_mode)
        self.kp_roll_angle_var = tk.StringVar(value=f"{startup_outer_config.kp_roll_angle:.3f}")
        self.kp_pitch_angle_var = tk.StringVar(value=f"{startup_outer_config.kp_pitch_angle:.3f}")
        self.ki_roll_angle_var = tk.StringVar(value=f"{startup_outer_config.ki_roll_angle:.3f}")
        self.ki_pitch_angle_var = tk.StringVar(value=f"{startup_outer_config.ki_pitch_angle:.3f}")
        self.kd_roll_angle_var = tk.StringVar(value=f"{startup_outer_config.kd_roll_angle:.3f}")
        self.kd_pitch_angle_var = tk.StringVar(value=f"{startup_outer_config.kd_pitch_angle:.3f}")
        self.int_limit_roll_angle_var = tk.StringVar(value=f"{startup_outer_config.integrator_limit_roll:.3f}")
        self.int_limit_pitch_angle_var = tk.StringVar(value=f"{startup_outer_config.integrator_limit_pitch:.3f}")
        self.kp_v_forward_var = tk.StringVar(value=f"{startup_body_velocity_outer_config.kp_v_forward:.3f}")
        self.kp_v_right_var = tk.StringVar(value=f"{startup_body_velocity_outer_config.kp_v_right:.3f}")
        self.velocity_angle_limit_var = tk.StringVar(value=f"{startup_body_velocity_outer_config.velocity_angle_limit_deg:.3f}")
        self.kp_yaw_var = tk.StringVar(value=f"{startup_yaw_outer_config.kp_yaw:.3f}")
        self.angle_rate_limit_var = tk.StringVar(value=f"{startup_outer_config.rate_limit_rad_s:.3f}")
        self.yaw_rate_limit_var = tk.StringVar(value=f"{startup_yaw_outer_config.yaw_rate_limit_rad_s:.3f}")

        self.alt_cmd_m_var = tk.StringVar(value=f"{startup_altitude_command.alt_cmd_m:.3f}")
        self.hover_throttle_var = tk.StringVar(value=f"{startup_altitude_command.hover_throttle:.3f}")
        self.kp_alt_var = tk.StringVar(value=f"{startup_altitude_config.kp_alt:.3f}")
        self.vz_max_var = tk.StringVar(value=f"{startup_altitude_config.vz_max:.3f}")
        self.kp_vz_var = tk.StringVar(value=f"{startup_altitude_config.kp_vz:.3f}")
        self.ki_vz_var = tk.StringVar(value=f"{startup_altitude_config.ki_vz:.3f}")
        self.kd_vz_var = tk.StringVar(value=f"{startup_altitude_config.kd_vz:.3f}")
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
            "ground_speed_m_s": tk.StringVar(value="-"),
            "p_meas": tk.StringVar(value="-"),
            "q_meas": tk.StringVar(value="-"),
            "r_meas": tk.StringVar(value="-"),
            "roll_rate_deg_s": tk.StringVar(value="-"),
            "pitch_rate_deg_s": tk.StringVar(value="-"),
            "yaw_rate_deg_s": tk.StringVar(value="-"),
            "vx_body_m_s": tk.StringVar(value="-"),
            "vy_body_m_s": tk.StringVar(value="-"),
            "vz_body_m_s": tk.StringVar(value="-"),
            "v_up_une_m_s": tk.StringVar(value="-"),
            "v_north_une_m_s": tk.StringVar(value="-"),
            "v_east_une_m_s": tk.StringVar(value="-"),
            "v_north_cmd_m_s": tk.StringVar(value="-"),
            "v_east_cmd_m_s": tk.StringVar(value="-"),
            "v_north_cmd_preprocessed_m_s": tk.StringVar(value="-"),
            "v_east_cmd_preprocessed_m_s": tk.StringVar(value="-"),
            "v_forward_cmd_projected_m_s": tk.StringVar(value="-"),
            "v_right_cmd_projected_m_s": tk.StringVar(value="-"),
            "vx_control_plane_cmd_m_s": tk.StringVar(value="-"),
            "vy_control_plane_cmd_m_s": tk.StringVar(value="-"),
            "vx_control_plane_meas_m_s": tk.StringVar(value="-"),
            "vy_control_plane_meas_m_s": tk.StringVar(value="-"),
            "vx_projected_une_meas_m_s": tk.StringVar(value="-"),
            "vy_projected_une_meas_m_s": tk.StringVar(value="-"),
            "vx_raw_body_meas_m_s": tk.StringVar(value="-"),
            "vy_raw_body_meas_m_s": tk.StringVar(value="-"),
            "world_feedback_mode": tk.StringVar(value="-"),
            "v_forward_cmd_from_world_m_s": tk.StringVar(value="-"),
            "v_right_cmd_from_world_m_s": tk.StringVar(value="-"),
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
            "r_cmd": tk.StringVar(value="-"),
            "error_r": tk.StringVar(value="-"),
            "kp_r": tk.StringVar(value="-"),
            "u_yaw": tk.StringVar(value="-"),
            "output_limit": tk.StringVar(value="-"),
        }
        self.outer_loop_vars = {
            "v_forward_error_m_s": tk.StringVar(value="-"),
            "v_right_error_m_s": tk.StringVar(value="-"),
            "pitch_cmd_from_velocity_deg": tk.StringVar(value="-"),
            "roll_cmd_from_velocity_deg": tk.StringVar(value="-"),
            "roll_error_deg": tk.StringVar(value="-"),
            "pitch_error_deg": tk.StringVar(value="-"),
            "yaw_error_deg": tk.StringVar(value="-"),
            "p_cmd_from_angle": tk.StringVar(value="-"),
            "q_cmd_from_angle": tk.StringVar(value="-"),
            "r_cmd_from_heading": tk.StringVar(value="-"),
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
        self._render_snapshot(snapshot)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._schedule_loop()

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
        self._build_inner_loop_controls(col1)
        self._build_outer_loop_controls(col2)
        self._build_altitude_loop_controls(col3)
        self._build_controller_panel(col3)
        self._build_telemetry_panel(col1)
        self._build_mixer_panel(col4)
        self._build_actuator_panel(col4)
        
    def _build_connection_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="System", padding=8)
        frame.pack(fill="x", pady=4)

        # Top toolbar
        toolbar = ttk.Frame(frame)
        toolbar.pack(fill="x", pady=(0, 8))

        left_buttons = ttk.Frame(toolbar)
        left_buttons.pack(side="left", fill="x", expand=True)

        for text, command in (
            ("Connect", self.connect),
            ("Disconnect", self.disconnect),
            ("Bind Tags", self.bind),
            ("Initialize", self.initialize),
        ):
            ttk.Button(left_buttons, text=text, command=command).pack(side="left", padx=(0, 6))

        # Keep emergency stop visually separated
        ttk.Button(toolbar, text="Emergency Stop", command=self.emergency_stop).pack(side="right")

        # Status summary row
        summary = ttk.Frame(frame)
        summary.pack(fill="x", pady=(0, 6))

        ttk.Label(summary, text="Vessel:").grid(row=0, column=0, sticky="w", padx=(0, 6), pady=2)
        ttk.Label(summary, textvariable=self.binding_vars["vessel"], font=gui_config.MONO_FONT).grid(
            row=0, column=1, sticky="w", pady=2
        )

        ttk.Label(summary, text="Initialized:").grid(row=0, column=2, sticky="w", padx=(16, 6), pady=2)
        ttk.Label(summary, textvariable=self.binding_vars["initialized"], font=gui_config.MONO_FONT).grid(
            row=0, column=3, sticky="w", pady=2
        )

        summary.columnconfigure(1, weight=1)
        summary.columnconfigure(3, weight=1)

        # Detailed mapping row
        details = ttk.Frame(frame)
        details.pack(fill="x")

        ttk.Label(details, text="Controller Tags:").grid(row=0, column=0, sticky="nw", padx=(0, 6), pady=2)
        ttk.Label(
            details,
            textvariable=self.binding_vars["controller_tags"],
            font=gui_config.MONO_FONT,
            wraplength=320,
            justify="left",
        ).grid(row=0, column=1, sticky="w", pady=2)

        ttk.Label(details, text="Rotor Tags:").grid(row=1, column=0, sticky="nw", padx=(0, 6), pady=2)
        ttk.Label(
            details,
            textvariable=self.binding_vars["rotor_tags"],
            font=gui_config.MONO_FONT,
            wraplength=320,
            justify="left",
        ).grid(row=1, column=1, sticky="w", pady=2)

        details.columnconfigure(1, weight=1)

    def _build_inner_loop_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Commands", padding=8)
        frame.pack(fill="x", pady=4)
        collective = ttk.LabelFrame(frame, text="Collective", padding=8)
        collective.pack(fill="x", pady=4)
        self._entry_row(collective, 0, "Base RPM", self.base_rpm_var)
        collective.columnconfigure(1, weight=1)

        body_rate = ttk.LabelFrame(frame, text="Body-Rate Command", padding=8)
        body_rate.pack(fill="x", pady=4)
        self._entry_row(body_rate, 0, "p_cmd (rad/s)", self.p_cmd_var)
        self._entry_row(body_rate, 1, "q_cmd (rad/s)", self.q_cmd_var)
        self._entry_row(body_rate, 2, "r_cmd (rad/s)", self.r_cmd_var)
        body_rate.columnconfigure(1, weight=1)

        row = ttk.Frame(frame)
        row.pack(fill="x", pady=(8, 0))
        ttk.Button(row, text="Apply Manual Commands", command=self.apply_manual_commands).pack(side="left", padx=3)
        ttk.Button(row, text="Start Test", command=self.start_test).pack(side="left", padx=3)
        ttk.Button(row, text="Stop Test", command=self.stop_test).pack(side="left", padx=3)

    def _build_outer_loop_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Outer Loop", padding=8)
        frame.pack(fill="x", pady=4)

        # =========================
        # Attitude Outer Loop
        # =========================
        attitude = ttk.LabelFrame(frame, text="Attitude Outer Loop", padding=6)
        attitude.pack(fill="x", pady=4)
        attitude.columnconfigure(0, weight=1)
        attitude.columnconfigure(1, weight=1)

        # Top area: command column + tuning column
        cmd_frame = ttk.Frame(attitude)
        cmd_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=(0, 6))
        cmd_frame.columnconfigure(1, weight=1)

        self._entry_row(cmd_frame, 0, "Roll Cmd (deg)", self.roll_cmd_deg_var)
        self._entry_row(cmd_frame, 1, "Pitch Cmd (deg)", self.pitch_cmd_deg_var)
        self._entry_row(cmd_frame, 2, "Yaw Cmd (deg)", self.yaw_cmd_deg_var)

        tuning_frame = ttk.Frame(attitude)
        tuning_frame.grid(row=0, column=1, sticky="nsew", padx=(4, 0), pady=(0, 6))
        tuning_frame.columnconfigure(1, weight=1)

        self._entry_row(tuning_frame, 0, "kp_yaw", self.kp_yaw_var)
        self._entry_row(tuning_frame, 1, "Rate Limit (rad/s)", self.angle_rate_limit_var)
        self._entry_row(tuning_frame, 2, "Yaw Rate Limit (rad/s)", self.yaw_rate_limit_var)

        # Roll / Pitch outer PID table
        outer_pid = ttk.LabelFrame(attitude, text="Roll / Pitch Outer PID", padding=6)
        outer_pid.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 6))

        ttk.Label(outer_pid, text="Axis").grid(row=0, column=0, sticky="w", padx=(0, 4), pady=(0, 3))
        ttk.Label(outer_pid, text="Kp").grid(row=0, column=1, sticky="w", padx=3, pady=(0, 3))
        ttk.Label(outer_pid, text="Ki").grid(row=0, column=2, sticky="w", padx=3, pady=(0, 3))
        ttk.Label(outer_pid, text="Kd").grid(row=0, column=3, sticky="w", padx=3, pady=(0, 3))
        ttk.Label(outer_pid, text="Int Lim").grid(row=0, column=4, sticky="w", padx=3, pady=(0, 3))

        outer_pid_rows = [
            ("roll", self.kp_roll_angle_var, self.ki_roll_angle_var, self.kd_roll_angle_var, self.int_limit_roll_angle_var),
            ("pitch", self.kp_pitch_angle_var, self.ki_pitch_angle_var, self.kd_pitch_angle_var, self.int_limit_pitch_angle_var),
        ]
        for row_idx, (axis_label, kp_var, ki_var, kd_var, int_limit_var) in enumerate(outer_pid_rows, start=1):
            ttk.Label(outer_pid, text=axis_label).grid(row=row_idx, column=0, sticky="w", padx=(0, 4), pady=2)
            ttk.Entry(outer_pid, textvariable=kp_var, width=8).grid(row=row_idx, column=1, sticky="ew", padx=3, pady=2)
            ttk.Entry(outer_pid, textvariable=ki_var, width=8).grid(row=row_idx, column=2, sticky="ew", padx=3, pady=2)
            ttk.Entry(outer_pid, textvariable=kd_var, width=8).grid(row=row_idx, column=3, sticky="ew", padx=3, pady=2)
            ttk.Entry(outer_pid, textvariable=int_limit_var, width=8).grid(row=row_idx, column=4, sticky="ew", padx=3, pady=2)

        outer_pid.columnconfigure(1, weight=1)
        outer_pid.columnconfigure(2, weight=1)
        outer_pid.columnconfigure(3, weight=1)
        outer_pid.columnconfigure(4, weight=1)

        attitude_buttons = ttk.Frame(attitude)
        attitude_buttons.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        ttk.Button(
            attitude_buttons,
            text="Apply Attitude Outer",
            command=self.apply_attitude_outer_loop_parameters
        ).pack(side="left", padx=3)
        ttk.Button(
            attitude_buttons,
            text="Start Outer Loop",
            command=self.start_outer_loop
        ).pack(side="left", padx=3)
        ttk.Button(
            attitude_buttons,
            text="Stop Outer Loop",
            command=self.stop_outer_loop
        ).pack(side="left", padx=3)

        # Compact 2-column info area
        attitude_info = ttk.LabelFrame(attitude, text="Attitude Feedback", padding=6)
        attitude_info.grid(row=3, column=0, columnspan=2, sticky="ew")

        left_rows = [
            ("roll_error (deg)", "roll_error_deg"),
            ("pitch_error (deg)", "pitch_error_deg"),
            ("yaw_error (deg)", "yaw_error_deg"),
        ]
        right_rows = [
            ("p_cmd_from_angle (rad/s)", "p_cmd_from_angle"),
            ("q_cmd_from_angle (rad/s)", "q_cmd_from_angle"),
            ("r_cmd_from_heading (rad/s)", "r_cmd_from_heading"),
        ]

        left_info = ttk.Frame(attitude_info)
        left_info.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        right_info = ttk.Frame(attitude_info)
        right_info.grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        for idx, (label, key) in enumerate(left_rows):
            ttk.Label(left_info, text=label).grid(row=idx, column=0, sticky="w", padx=(0, 8), pady=2)
            ttk.Label(left_info, textvariable=self.outer_loop_vars[key], font=gui_config.MONO_FONT).grid(
                row=idx, column=1, sticky="w", pady=2
            )

        for idx, (label, key) in enumerate(right_rows):
            ttk.Label(right_info, text=label).grid(row=idx, column=0, sticky="w", padx=(0, 8), pady=2)
            ttk.Label(right_info, textvariable=self.outer_loop_vars[key], font=gui_config.MONO_FONT).grid(
                row=idx, column=1, sticky="w", pady=2
            )

        attitude_info.columnconfigure(0, weight=1)
        attitude_info.columnconfigure(1, weight=1)

        # =========================
        # Body Velocity Outer Loop
        # =========================
        velocity = ttk.LabelFrame(frame, text="Body Velocity Outer Loop", padding=8)
        velocity.pack(fill="x", pady=4)
        self._entry_row(velocity, 0, "v_forward_cmd (m/s)", self.v_forward_cmd_var)
        self._entry_row(velocity, 1, "v_right_cmd (m/s)", self.v_right_cmd_var)
        self._entry_row(velocity, 2, "kp_v_forward", self.kp_v_forward_var)
        self._entry_row(velocity, 3, "kp_v_right", self.kp_v_right_var)
        self._entry_row(velocity, 4, "Angle Limit (deg)", self.velocity_angle_limit_var)
        velocity.columnconfigure(1, weight=1)

        velocity_buttons = ttk.Frame(velocity)
        velocity_buttons.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(
            velocity_buttons,
            text="Apply Velocity Outer",
            command=self.apply_body_velocity_outer_loop_parameters
        ).pack(side="left", padx=3)
        ttk.Button(
            velocity_buttons,
            text="Start Velocity Outer",
            command=self.start_body_velocity_outer_loop
        ).pack(side="left", padx=3)
        ttk.Button(
            velocity_buttons,
            text="Stop Velocity Outer",
            command=self.stop_body_velocity_outer_loop
        ).pack(side="left", padx=3)

        velocity_info = ttk.Frame(velocity)
        velocity_info.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        velocity_rows = [
            ("v_forward_error (m/s)", "v_forward_error_m_s"),
            ("v_right_error (m/s)", "v_right_error_m_s"),
            ("pitch_cmd_from_velocity (deg)", "pitch_cmd_from_velocity_deg"),
            ("roll_cmd_from_velocity (deg)", "roll_cmd_from_velocity_deg"),
        ]
        for idx, (label, key) in enumerate(velocity_rows):
            ttk.Label(velocity_info, text=label).grid(row=idx, column=0, sticky="w", padx=(0, 10), pady=2)
            ttk.Label(velocity_info, textvariable=self.outer_loop_vars[key], font=gui_config.MONO_FONT).grid(
                row=idx, column=1, sticky="w", pady=2
            )
        velocity_info.columnconfigure(1, weight=1)

        # =========================
        # World Velocity / Mapper
        # =========================
        world_velocity = ttk.LabelFrame(frame, text="World Velocity Control", padding=8)
        world_velocity.pack(fill="x", pady=4)
        world_velocity.columnconfigure(0, weight=1)

        world_command = ttk.LabelFrame(world_velocity, text="World Velocity Command", padding=8)
        world_command.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        self._entry_row(world_command, 0, "north cmd (m/s)", self.v_north_cmd_var)
        self._entry_row(world_command, 1, "east cmd (m/s)", self.v_east_cmd_var)
        world_command.columnconfigure(1, weight=1)
        ttk.Button(
            world_command,
            text="Apply World Velocity",
            command=self.apply_world_velocity_command,
        ).grid(row=2, column=0, columnspan=2, sticky="w", pady=(8, 0))

        world_signs = ttk.LabelFrame(world_velocity, text="World Command Sign Fix", padding=8)
        world_signs.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        ttk.Label(world_signs, text="Invert World North").grid(row=0, column=0, sticky="w", pady=4)
        ttk.Checkbutton(world_signs, variable=self.invert_world_north_var).grid(row=0, column=1, sticky="w", pady=4)
        ttk.Label(world_signs, text="Invert World East").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Checkbutton(world_signs, variable=self.invert_world_east_var).grid(row=1, column=1, sticky="w", pady=4)
        ttk.Button(
            world_signs,
            text="Apply World Signs",
            command=self.apply_world_command_preprocess_config,
        ).grid(row=2, column=0, columnspan=2, sticky="w", pady=(8, 0))

        mapper = ttk.LabelFrame(world_velocity, text="UNE Mapper Config", padding=8)
        mapper.grid(row=2, column=0, sticky="ew", pady=(0, 6))
        ttk.Label(mapper, text="Swap N/E").grid(row=0, column=0, sticky="w", pady=4)
        ttk.Checkbutton(mapper, variable=self.swap_ne_var).grid(row=0, column=1, sticky="w", pady=4)
        ttk.Label(mapper, text="Invert X").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Checkbutton(mapper, variable=self.invert_x_var).grid(row=1, column=1, sticky="w", pady=4)
        ttk.Label(mapper, text="Invert Y").grid(row=2, column=0, sticky="w", pady=4)
        ttk.Checkbutton(mapper, variable=self.invert_y_var).grid(row=2, column=1, sticky="w", pady=4)
        ttk.Button(
            mapper,
            text="Apply Mapper",
            command=self.apply_mapper_config,
        ).grid(row=3, column=0, columnspan=2, sticky="w", pady=(8, 0))

        lifecycle = ttk.LabelFrame(world_velocity, text="Control Lifecycle", padding=8)
        lifecycle.grid(row=3, column=0, sticky="ew")
        ttk.Label(lifecycle, text="World Feedback Mode").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=(0, 6))
        ttk.Combobox(
            lifecycle,
            textvariable=self.world_feedback_mode_var,
            values=("projected_une", "raw_body"),
            state="readonly",
            width=18,
        ).grid(row=0, column=1, sticky="w", pady=(0, 6))
        ttk.Button(
            lifecycle,
            text="Apply Feedback Mode",
            command=self.apply_world_feedback_mode,
        ).grid(row=0, column=2, sticky="w", padx=(8, 0), pady=(0, 6))
        ttk.Button(
            lifecycle,
            text="Start World Velocity Control",
            command=self.start_world_velocity_control,
        ).grid(row=1, column=0, sticky="w", padx=(0, 6))
        ttk.Button(
            lifecycle,
            text="Stop World Velocity Control",
            command=self.stop_world_velocity_control,
        ).grid(row=1, column=1, sticky="w")

        # =========================
        # Altitude Outer Loop
        # =========================
        altitude = ttk.LabelFrame(frame, text="Altitude Outer Loop", padding=8)
        altitude.pack(fill="x", pady=4)
        self._entry_row(altitude, 0, "Alt Cmd (m)", self.alt_cmd_m_var)
        self._entry_row(altitude, 1, "Hover Throttle", self.hover_throttle_var)
        self._entry_row(altitude, 2, "kp_alt", self.kp_alt_var)
        self._entry_row(altitude, 3, "vz_max", self.vz_max_var)
        altitude.columnconfigure(1, weight=1)

        altitude_buttons = ttk.Frame(altitude)
        altitude_buttons.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(
            altitude_buttons,
            text="Apply Altitude Outer",
            command=self.apply_altitude_outer_loop_parameters
        ).pack(side="left", padx=3)
        ttk.Button(
            altitude_buttons,
            text="Start Altitude Loop",
            command=self.start_altitude_loop
        ).pack(side="left", padx=3)
        ttk.Button(
            altitude_buttons,
            text="Stop Altitude Loop",
            command=self.stop_altitude_loop
        ).pack(side="left", padx=3)

        altitude_info = ttk.Frame(altitude)
        altitude_info.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        altitude_rows = [
            ("alt_error (m)", "alt_error_m"),
            ("vz_cmd (m/s)", "vz_cmd_m_s"),
        ]
        for idx, (label, key) in enumerate(altitude_rows):
            ttk.Label(altitude_info, text=label).grid(row=idx, column=0, sticky="w", padx=(0, 10), pady=2)
            ttk.Label(altitude_info, textvariable=self.altitude_loop_vars[key], font=gui_config.MONO_FONT).grid(
                row=idx, column=1, sticky="w", pady=2
            )
        altitude_info.columnconfigure(1, weight=1)

    def _build_altitude_loop_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Inner Loop", padding=8)
        frame.pack(fill="x", pady=4)
        frame.columnconfigure(0, weight=1, uniform="inner_loop")
        frame.columnconfigure(1, weight=1, uniform="inner_loop")

        body_rate = ttk.LabelFrame(frame, text="Body-Rate Inner Loop", padding=8)
        body_rate.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=4)
        ttk.Label(body_rate, text="Axis").grid(row=0, column=0, sticky="w", padx=(0, 6), pady=(0, 4))
        ttk.Label(body_rate, text="Kp").grid(row=0, column=1, sticky="w", padx=4, pady=(0, 4))
        ttk.Label(body_rate, text="Ki").grid(row=0, column=2, sticky="w", padx=4, pady=(0, 4))
        ttk.Label(body_rate, text="Kd").grid(row=0, column=3, sticky="w", padx=4, pady=(0, 4))
        ttk.Label(body_rate, text="Int Lim").grid(row=0, column=4, sticky="w", padx=4, pady=(0, 4))
        body_rate_rows = [
            ("p", self.kp_p_var, self.ki_p_var, self.kd_p_var, self.int_limit_p_var),
            ("q", self.kp_q_var, self.ki_q_var, self.kd_q_var, self.int_limit_q_var),
            ("r", self.kp_r_var, self.ki_r_var, self.kd_r_var, self.int_limit_r_var),
        ]
        for row_idx, (axis_label, kp_var, ki_var, kd_var, int_limit_var) in enumerate(body_rate_rows, start=1):
            ttk.Label(body_rate, text=axis_label).grid(row=row_idx, column=0, sticky="w", padx=(0, 6), pady=2)
            ttk.Entry(body_rate, textvariable=kp_var, width=10).grid(row=row_idx, column=1, sticky="ew", padx=4, pady=2)
            ttk.Entry(body_rate, textvariable=ki_var, width=10).grid(row=row_idx, column=2, sticky="ew", padx=4, pady=2)
            ttk.Entry(body_rate, textvariable=kd_var, width=10).grid(row=row_idx, column=3, sticky="ew", padx=4, pady=2)
            ttk.Entry(body_rate, textvariable=int_limit_var, width=10).grid(row=row_idx, column=4, sticky="ew", padx=4, pady=2)
        self._entry_row(body_rate, 5, "Output Limit", self.output_limit_var)
        body_rate.columnconfigure(1, weight=1)
        body_rate.columnconfigure(2, weight=1)
        body_rate.columnconfigure(3, weight=1)
        body_rate.columnconfigure(4, weight=1)
        body_rate_buttons = ttk.Frame(body_rate)
        body_rate_buttons.grid(row=6, column=0, columnspan=5, sticky="ew", pady=(8, 0))
        ttk.Button(body_rate_buttons, text="Apply", command=self.apply_inner_loop_parameters).pack(side="left", padx=3)

        vertical = ttk.LabelFrame(frame, text="Altitude / Vertical-Speed Loop", padding=8)
        vertical.grid(row=0, column=1, sticky="nsew", padx=(4, 0), pady=4)
        ttk.Label(vertical, text="Gain").grid(row=0, column=0, sticky="w", padx=(0, 6), pady=(0, 4))
        ttk.Label(vertical, text="Value").grid(row=0, column=1, sticky="w", padx=4, pady=(0, 4))
        self._entry_row(vertical, 1, "kp_vz", self.kp_vz_var)
        self._entry_row(vertical, 2, "ki_vz", self.ki_vz_var)
        self._entry_row(vertical, 3, "kd_vz", self.kd_vz_var)
        self._entry_row(vertical, 4, "Throttle Min", self.throttle_min_var)
        self._entry_row(vertical, 5, "Throttle Max", self.throttle_max_var)
        vertical.columnconfigure(1, weight=1)
        vertical_buttons = ttk.Frame(vertical)
        vertical_buttons.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(vertical_buttons, text="Apply", command=self.apply_altitude_loop_parameters).pack(side="left", padx=3)

        vertical_info = ttk.Frame(vertical)
        vertical_info.grid(row=7, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        rows = [
            ("vz_error (m/s)", "vz_error_m_s"),
            ("throttle_correction", "throttle_correction"),
            ("throttle_cmd", "throttle_cmd"),
        ]
        for idx, (label, key) in enumerate(rows):
            ttk.Label(vertical_info, text=label).grid(row=idx, column=0, sticky="w", padx=(0, 10), pady=2)
            ttk.Label(vertical_info, textvariable=self.altitude_loop_vars[key], font=gui_config.MONO_FONT).grid(row=idx, column=1, sticky="w", pady=2)
        vertical_info.columnconfigure(1, weight=1)

    def _build_telemetry_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Telemetry", padding=8)
        frame.pack(fill="x", pady=4)

        frame.columnconfigure(0, weight=1, uniform="telemetry_grid")
        frame.columnconfigure(1, weight=1, uniform="telemetry_grid")

        angle_frame = ttk.LabelFrame(frame, text="RPY Angles", padding=8)
        angle_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=4)
        self._label_rows(
            angle_frame,
            [("Roll", "roll_deg"), ("Pitch", "pitch_deg"), ("Yaw / Heading", "heading_deg")],
            value_suffix=" deg",
            variable_group=self.telemetry_vars,
        )

        altitude_frame = ttk.LabelFrame(frame, text="Altitude", padding=8)
        altitude_frame.grid(row=0, column=1, sticky="nsew", padx=(4, 0), pady=4)
        self._label_rows(
            altitude_frame,
            [("Altitude", "alt_m"),
             ("Ground Speed", "ground_speed_m_s")
             ],
            value_suffix="",
            variable_group=self.telemetry_vars,
        )

        velocity_frame = ttk.LabelFrame(frame, text="Velocity", padding=8)
        velocity_frame.grid(row=2, column=0, sticky="nsew", padx=(4, 0), pady=4)
        self._label_rows(
            velocity_frame,
            [
                ("Vertical Speed", "vz_m_s"),
                ("Ground Speed", "ground_speed_m_s"),
                ("v_forward", "vx_body_m_s"),
                ("v_right", "vy_body_m_s"),
                ("v_down", "vz_body_m_s"),
                ("proj_v_forward", "v_forward_cmd_from_world_m_s"),
                ("proj_v_right", "v_right_cmd_from_world_m_s"),
            ],
            value_suffix="",
            variable_group=self.telemetry_vars,
        )

        debug_frame = ttk.LabelFrame(frame, text="UNE / Control Plane Debug", padding=8)
        debug_frame.grid(row=3, column=0, columnspan=2, sticky="nsew", pady=4)

        debug_left = ttk.Frame(debug_frame)
        debug_left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self._label_rows(
            debug_left,
            [
                ("world cmd north", "v_north_cmd_m_s"),
                ("world cmd east", "v_east_cmd_m_s"),
                ("cmd north pre", "v_north_cmd_preprocessed_m_s"),
                ("cmd east pre", "v_east_cmd_preprocessed_m_s"),
                ("cmd proj fwd", "v_forward_cmd_projected_m_s"),
                ("cmd proj right", "v_right_cmd_projected_m_s"),
                ("UNE meas north", "v_north_une_m_s"),
                ("UNE meas east", "v_east_une_m_s"),
            ],
            value_suffix="",
            variable_group=self.telemetry_vars,
        )

        debug_right = ttk.Frame(debug_frame)
        debug_right.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        self._label_rows(
            debug_right,
            [
                ("active mode", "world_feedback_mode"),
                ("ctrl cmd x", "vx_control_plane_cmd_m_s"),
                ("ctrl cmd y", "vy_control_plane_cmd_m_s"),
                ("active meas x", "vx_control_plane_meas_m_s"),
                ("active meas y", "vy_control_plane_meas_m_s"),
                ("proj UNE x", "vx_projected_une_meas_m_s"),
                ("proj UNE y", "vy_projected_une_meas_m_s"),
                ("raw body x", "vx_raw_body_meas_m_s"),
                ("raw body y", "vy_raw_body_meas_m_s"),
            ],
            value_suffix="",
            variable_group=self.telemetry_vars,
        )

        debug_outputs = ttk.Frame(debug_frame)
        debug_outputs.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Label(debug_outputs, text="vel->pitch_cmd").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=2)
        ttk.Label(debug_outputs, textvariable=self.outer_loop_vars["pitch_cmd_from_velocity_deg"], font=gui_config.MONO_FONT).grid(row=0, column=1, sticky="w", pady=2)
        ttk.Label(debug_outputs, text="vel->roll_cmd").grid(row=0, column=2, sticky="w", padx=(16, 8), pady=2)
        ttk.Label(debug_outputs, textvariable=self.outer_loop_vars["roll_cmd_from_velocity_deg"], font=gui_config.MONO_FONT).grid(row=0, column=3, sticky="w", pady=2)

        body_rate_frame = ttk.LabelFrame(frame, text="Body Rates (FRD)", padding=8)
        body_rate_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 4), pady=4)
        self._label_rows(
            body_rate_frame,
            [("p", "p_meas"), ("q", "q_meas"), ("r", "r_meas")],
            value_suffix=" rad/s",
            variable_group=self.telemetry_vars,
        )

        rates_frame = ttk.LabelFrame(frame, text="Euler Rates", padding=8)
        rates_frame.grid(row=1, column=1, sticky="nsew", padx=4, pady=4)
        self._label_rows(
            rates_frame,
            [("phi_dot", "roll_rate_deg_s"), ("theta_dot", "pitch_rate_deg_s"), ("psi_dot", "yaw_rate_deg_s")],
            value_suffix="",
            variable_group=self.telemetry_vars,
        )

        timing_frame = ttk.LabelFrame(frame, text="Timing", padding=8)
        timing_frame.grid(row=2, column=1, sticky="nsew", padx=(4, 0), pady=4)
        self._label_rows(
            timing_frame,
            [("dt", "dt_s")],
            value_suffix="",
            variable_group=self.telemetry_vars,
        )

    def _build_controller_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Inner-Loop Outputs", padding=8)
        frame.pack(fill="x", pady=4)
        frame.columnconfigure(0, weight=1, uniform="controller_channels")
        frame.columnconfigure(1, weight=1, uniform="controller_channels")
        frame.columnconfigure(2, weight=1, uniform="controller_channels")

        roll_frame = ttk.LabelFrame(frame, text="Roll Channel", padding=8)
        roll_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=4)
        self._label_rows(roll_frame, [("p_cmd", "p_cmd"), ("error_p", "error_p"), ("kp_p", "kp_p"), ("u_roll", "u_roll")], value_suffix="", variable_group=self.controller_vars)
        pitch_frame = ttk.LabelFrame(frame, text="Pitch Channel", padding=8)
        pitch_frame.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        self._label_rows(pitch_frame, [("q_cmd", "q_cmd"), ("error_q", "error_q"), ("kp_q", "kp_q"), ("u_pitch", "u_pitch")], value_suffix="", variable_group=self.controller_vars)
        yaw_frame = ttk.LabelFrame(frame, text="Yaw Channel", padding=8)
        yaw_frame.grid(row=0, column=2, sticky="nsew", padx=(4, 0), pady=4)
        self._label_rows(yaw_frame, [("r_cmd", "r_cmd"), ("error_r", "error_r"), ("kp_r", "kp_r"), ("u_yaw", "u_yaw")], value_suffix="", variable_group=self.controller_vars)
        common_frame = ttk.LabelFrame(frame, text="Common", padding=8)
        common_frame.grid(row=1, column=0, columnspan=3, sticky="ew", pady=4)
        self._label_rows(common_frame, [("output_limit", "output_limit")], value_suffix="", variable_group=self.controller_vars)

    def _build_mixer_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Mixer", padding=8)
        frame.pack(fill="both", expand=True, pady=4)
        ttk.Label(frame, text="Current mixer").grid(row=0, column=0, sticky="w", padx=(0, 10), pady=3)
        ttk.Label(frame, textvariable=self.mixer_vars["name"], font=gui_config.MONO_FONT).grid(row=0, column=1, sticky="w", pady=3)
        ttk.Label(frame, text="Candidate").grid(row=1, column=0, sticky="w", padx=(0, 10), pady=3)
        ttk.Combobox(frame, values=self.runtime.get_mixer_names(), textvariable=self.mixer_name_var, state="readonly", width=22).grid(row=1, column=1, sticky="ew", pady=3)
        ttk.Button(frame, text="Apply Mixer", command=lambda: self.runtime.set_mixer_candidate(self.mixer_name_var.get())).grid(row=2, column=0, columnspan=2, sticky="w", pady=(4, 6))
        ttk.Label(frame, text="Mixer matrix").grid(row=3, column=0, sticky="nw", padx=(0, 10), pady=3)
        self.matrix_text = scrolledtext.ScrolledText(frame, height=gui_config.MATRIX_TEXT_HEIGHT, width=gui_config.MATRIX_TEXT_WIDTH, wrap=tk.WORD, font=gui_config.MONO_FONT)
        self.matrix_text.grid(row=3, column=1, sticky="nsew", pady=3)
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(3, weight=1)

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
            r_cmd_rad_s=self._parse_float(self.r_cmd_var.get()),
            kp_p=self._parse_float(self.kp_p_var.get()),
            kp_q=self._parse_float(self.kp_q_var.get()),
            kp_r=self._parse_float(self.kp_r_var.get()),
            ki_p=self._parse_float(self.ki_p_var.get()),
            ki_q=self._parse_float(self.ki_q_var.get()),
            ki_r=self._parse_float(self.ki_r_var.get()),
            kd_p=self._parse_float(self.kd_p_var.get()),
            kd_q=self._parse_float(self.kd_q_var.get()),
            kd_r=self._parse_float(self.kd_r_var.get()),
            integrator_limit_p=self._parse_float(self.int_limit_p_var.get()),
            integrator_limit_q=self._parse_float(self.int_limit_q_var.get()),
            integrator_limit_r=self._parse_float(self.int_limit_r_var.get()),
            output_limit=self._parse_float(self.output_limit_var.get()),
        ))

    def apply_manual_commands(self) -> None:
        self.apply_inner_loop_parameters()

    def apply_attitude_outer_loop_parameters(self) -> None:
        self.runtime.set_angle_command(AngleCommand(
            roll_cmd_deg=self._parse_float(self.roll_cmd_deg_var.get()),
            pitch_cmd_deg=self._parse_float(self.pitch_cmd_deg_var.get()),
            yaw_cmd_deg=self._parse_float(self.yaw_cmd_deg_var.get()),
        ))
        self.runtime.set_outer_loop_config(AngleOuterLoopConfig(
            kp_roll_angle=self._parse_float(self.kp_roll_angle_var.get()),
            kp_pitch_angle=self._parse_float(self.kp_pitch_angle_var.get()),
            ki_roll_angle=self._parse_float(self.ki_roll_angle_var.get()),
            ki_pitch_angle=self._parse_float(self.ki_pitch_angle_var.get()),
            kd_roll_angle=self._parse_float(self.kd_roll_angle_var.get()),
            kd_pitch_angle=self._parse_float(self.kd_pitch_angle_var.get()),
            integrator_limit_roll=self._parse_float(self.int_limit_roll_angle_var.get()),
            integrator_limit_pitch=self._parse_float(self.int_limit_pitch_angle_var.get()),
            rate_limit_rad_s=self._parse_float(self.angle_rate_limit_var.get()),
        ))
        self.runtime.set_yaw_outer_loop_config(YawOuterLoopConfig(
            kp_yaw=self._parse_float(self.kp_yaw_var.get()),
            yaw_rate_limit_rad_s=self._parse_float(self.yaw_rate_limit_var.get()),
        ))

    def apply_body_velocity_outer_loop_parameters(self) -> None:
        self.runtime.set_body_velocity_command(BodyVelocityCommand(
            v_forward_cmd_m_s=self._parse_float(self.v_forward_cmd_var.get()),
            v_right_cmd_m_s=self._parse_float(self.v_right_cmd_var.get()),
        ))
        self.apply_velocity_outer_loop_gains()

    def apply_velocity_outer_loop_gains(self) -> None:
        self.runtime.set_body_velocity_outer_loop_config(BodyVelocityOuterLoopConfig(
            kp_v_forward=self._parse_float(self.kp_v_forward_var.get()),
            kp_v_right=self._parse_float(self.kp_v_right_var.get()),
            velocity_angle_limit_deg=self._parse_float(self.velocity_angle_limit_var.get()),
        ))

    def apply_world_velocity_command(self) -> None:
        self.runtime.set_world_velocity_command(WorldVelocityCommand(
            v_north_cmd_m_s=self._parse_float(self.v_north_cmd_var.get()),
            v_east_cmd_m_s=self._parse_float(self.v_east_cmd_var.get()),
        ))
        self._render_snapshot(self.runtime.get_snapshot())

    def apply_world_command_preprocess_config(self) -> None:
        self.runtime.set_world_command_preprocess_config(WorldCommandPreprocessConfig(
            invert_world_north=bool(self.invert_world_north_var.get()),
            invert_world_east=bool(self.invert_world_east_var.get()),
        ))
        self._render_snapshot(self.runtime.get_snapshot())

    def apply_mapper_config(self) -> None:
        self.runtime.set_une_to_control_plane_config(UneToControlPlaneConfig(
            swap_ne=bool(self.swap_ne_var.get()),
            sign_x=-1.0 if self.invert_x_var.get() else 1.0,
            sign_y=-1.0 if self.invert_y_var.get() else 1.0,
        ))
        self._render_snapshot(self.runtime.get_snapshot())

    def apply_world_feedback_mode(self) -> None:
        self.runtime.set_world_velocity_feedback_mode(self.world_feedback_mode_var.get())
        self._render_snapshot(self.runtime.get_snapshot())

    def apply_altitude_outer_loop_parameters(self) -> None:
        self.runtime.set_altitude_command(AltitudeCommand(
            alt_cmd_m=self._parse_float(self.alt_cmd_m_var.get()),
            hover_throttle=self._parse_float(self.hover_throttle_var.get()),
        ))
        self.runtime.set_altitude_config(AltitudeControlConfig(
            kp_alt=self._parse_float(self.kp_alt_var.get()),
            vz_max=self._parse_float(self.vz_max_var.get()),
            kp_vz=self._parse_float(self.kp_vz_var.get()),
            ki_vz=self._parse_float(self.ki_vz_var.get()),
            kd_vz=self._parse_float(self.kd_vz_var.get()),
            throttle_min=self._parse_float(self.throttle_min_var.get()),
            throttle_max=self._parse_float(self.throttle_max_var.get()),
        ))

    def apply_altitude_loop_parameters(self) -> None:
        current_snapshot = self.runtime.get_snapshot()
        current_config = current_snapshot.altitude_control_config
        self.runtime.set_altitude_config(AltitudeControlConfig(
            kp_alt=current_config.kp_alt,
            vz_max=current_config.vz_max,
            kp_vz=self._parse_float(self.kp_vz_var.get()),
            ki_vz=self._parse_float(self.ki_vz_var.get()),
            kd_vz=self._parse_float(self.kd_vz_var.get()),
            throttle_min=self._parse_float(self.throttle_min_var.get()),
            throttle_max=self._parse_float(self.throttle_max_var.get()),
        ))

    def start_test(self) -> None:
        def _start() -> None:
            self.apply_manual_commands()
            self.runtime.start_test()
        self._safe_action(_start)

    def stop_test(self) -> None:
        self._safe_action(self.runtime.stop_test)

    def start_outer_loop(self) -> None:
        def _start() -> None:
            self.apply_manual_commands()
            self.apply_attitude_outer_loop_parameters()
            self.runtime.start_outer_loop()
        self._safe_action(_start)

    def stop_outer_loop(self) -> None:
        self._safe_action(self.runtime.stop_outer_loop)

    def start_body_velocity_outer_loop(self) -> None:
        def _start() -> None:
            self.apply_body_velocity_outer_loop_parameters()
            self.runtime.start_body_velocity_outer_loop()
        self._safe_action(_start)

    def stop_body_velocity_outer_loop(self) -> None:
        self._safe_action(self.runtime.stop_body_velocity_outer_loop)

    def start_world_velocity_control(self) -> None:
        def _start() -> None:
            self.apply_velocity_outer_loop_gains()
            self.apply_world_velocity_command()
            self.apply_world_command_preprocess_config()
            self.apply_mapper_config()
            self.apply_world_feedback_mode()
            self.runtime.start_world_velocity_control()
        self._safe_action(_start)

    def stop_world_velocity_control(self) -> None:
        self._safe_action(self.runtime.stop_world_velocity_control)

    def start_altitude_loop(self) -> None:
        def _start() -> None:
            self.apply_altitude_outer_loop_parameters()
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
        except Exception as exc:
            self.status_var.set(f"Refresh error: {exc}")
            messagebox.showerror("Roll / Pitch Inner-Loop Test", str(exc))
            self._schedule_loop()
            return

        self._render_snapshot(snapshot)
        pending_error = self.runtime.consume_pending_error()
        if pending_error:
            messagebox.showerror("Roll / Pitch Inner-Loop Test", pending_error)
        self._schedule_loop()

    def _render_snapshot(self, snapshot) -> None:
        binding = getattr(snapshot, "binding", None)
        telemetry = getattr(snapshot, "telemetry", None)
        controller = getattr(snapshot, "controller", None)
        command = getattr(snapshot, "command", None)
        outer_loop = getattr(snapshot, "outer_loop", None)
        body_velocity_outer_loop = getattr(snapshot, "body_velocity_outer_loop", None)
        control_plane_velocity_debug = getattr(snapshot, "control_plane_velocity_debug", None)
        world_velocity_command = getattr(snapshot, "world_velocity_command", None)
        world_velocity_projection = getattr(snapshot, "world_velocity_projection", None)
        yaw_outer_loop = getattr(snapshot, "yaw_outer_loop", None)
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
        self.telemetry_vars["ground_speed_m_s"].set(f"{getattr(telemetry, 'ground_speed_m_s', 0.0):.3f}")
        self.telemetry_vars["p_meas"].set(f"{getattr(telemetry, 'p_meas_rad_s', 0.0):.4f}")
        self.telemetry_vars["q_meas"].set(f"{getattr(telemetry, 'q_meas_rad_s', 0.0):.4f}")
        self.telemetry_vars["r_meas"].set(f"{getattr(telemetry, 'r_meas_rad_s', 0.0):.4f}")
        self.telemetry_vars["roll_rate_deg_s"].set(f"{getattr(telemetry, 'roll_rate_deg_s', 0.0):.3f}")
        self.telemetry_vars["pitch_rate_deg_s"].set(f"{getattr(telemetry, 'pitch_rate_deg_s', 0.0):.3f}")
        self.telemetry_vars["yaw_rate_deg_s"].set(f"{getattr(telemetry, 'yaw_rate_deg_s', 0.0):.3f}")
        self.telemetry_vars["v_up_une_m_s"].set(f"{getattr(telemetry, 'v_up_une_m_s', 0.0):+.3f}")
        self.telemetry_vars["v_north_une_m_s"].set(f"{getattr(telemetry, 'v_north_une_m_s', 0.0):+.3f}")
        self.telemetry_vars["v_east_une_m_s"].set(f"{getattr(telemetry, 'v_east_une_m_s', 0.0):+.3f}")
        self.telemetry_vars["vx_body_m_s"].set(f"{getattr(telemetry, 'vx_body_m_s', 0.0):+.3f}")
        self.telemetry_vars["vy_body_m_s"].set(f"{getattr(telemetry, 'vy_body_m_s', 0.0):+.3f}")
        self.telemetry_vars["vz_body_m_s"].set(f"{getattr(telemetry, 'vz_body_m_s', 0.0):+.3f}")
        self.telemetry_vars["v_north_cmd_m_s"].set(f"{getattr(world_velocity_command, 'v_north_cmd_m_s', 0.0):+.3f}")
        self.telemetry_vars["v_east_cmd_m_s"].set(f"{getattr(world_velocity_command, 'v_east_cmd_m_s', 0.0):+.3f}")
        self.telemetry_vars["v_north_cmd_preprocessed_m_s"].set(f"{getattr(control_plane_velocity_debug, 'v_north_cmd_preprocessed_m_s', 0.0):+.3f}")
        self.telemetry_vars["v_east_cmd_preprocessed_m_s"].set(f"{getattr(control_plane_velocity_debug, 'v_east_cmd_preprocessed_m_s', 0.0):+.3f}")
        self.telemetry_vars["v_forward_cmd_projected_m_s"].set(f"{getattr(control_plane_velocity_debug, 'v_forward_cmd_projected_m_s', 0.0):+.3f}")
        self.telemetry_vars["v_right_cmd_projected_m_s"].set(f"{getattr(control_plane_velocity_debug, 'v_right_cmd_projected_m_s', 0.0):+.3f}")
        self.telemetry_vars["world_feedback_mode"].set(getattr(control_plane_velocity_debug, "world_feedback_mode", getattr(snapshot, "world_velocity_feedback_mode", "-")))
        self.telemetry_vars["vx_control_plane_cmd_m_s"].set(f"{getattr(control_plane_velocity_debug, 'vx_control_plane_cmd_m_s', 0.0):+.3f}")
        self.telemetry_vars["vy_control_plane_cmd_m_s"].set(f"{getattr(control_plane_velocity_debug, 'vy_control_plane_cmd_m_s', 0.0):+.3f}")
        self.telemetry_vars["vx_control_plane_meas_m_s"].set(f"{getattr(control_plane_velocity_debug, 'vx_control_plane_meas_m_s', 0.0):+.3f}")
        self.telemetry_vars["vy_control_plane_meas_m_s"].set(f"{getattr(control_plane_velocity_debug, 'vy_control_plane_meas_m_s', 0.0):+.3f}")
        self.telemetry_vars["vx_projected_une_meas_m_s"].set(f"{getattr(control_plane_velocity_debug, 'vx_projected_une_meas_m_s', 0.0):+.3f}")
        self.telemetry_vars["vy_projected_une_meas_m_s"].set(f"{getattr(control_plane_velocity_debug, 'vy_projected_une_meas_m_s', 0.0):+.3f}")
        self.telemetry_vars["vx_raw_body_meas_m_s"].set(f"{getattr(control_plane_velocity_debug, 'vx_raw_body_meas_m_s', 0.0):+.3f}")
        self.telemetry_vars["vy_raw_body_meas_m_s"].set(f"{getattr(control_plane_velocity_debug, 'vy_raw_body_meas_m_s', 0.0):+.3f}")
        self.telemetry_vars["v_forward_cmd_from_world_m_s"].set(f"{getattr(world_velocity_projection, 'v_forward_cmd_from_world_m_s', 0.0):+.3f}")
        self.telemetry_vars["v_right_cmd_from_world_m_s"].set(f"{getattr(world_velocity_projection, 'v_right_cmd_from_world_m_s', 0.0):+.3f}")
        self.telemetry_vars["dt_s"].set(f"{getattr(telemetry, 'dt_s', 0.0):.4f}")

        self.outer_loop_vars["v_forward_error_m_s"].set(f"{getattr(body_velocity_outer_loop, 'v_forward_error_m_s', 0.0):.3f}")
        self.outer_loop_vars["v_right_error_m_s"].set(f"{getattr(body_velocity_outer_loop, 'v_right_error_m_s', 0.0):.3f}")
        self.outer_loop_vars["pitch_cmd_from_velocity_deg"].set(f"{getattr(body_velocity_outer_loop, 'pitch_cmd_from_velocity_deg', 0.0):.3f}")
        self.outer_loop_vars["roll_cmd_from_velocity_deg"].set(f"{getattr(body_velocity_outer_loop, 'roll_cmd_from_velocity_deg', 0.0):.3f}")
        self.outer_loop_vars["roll_error_deg"].set(f"{getattr(outer_loop, 'roll_error_deg', 0.0):.3f}")
        self.outer_loop_vars["pitch_error_deg"].set(f"{getattr(outer_loop, 'pitch_error_deg', 0.0):.3f}")
        self.outer_loop_vars["yaw_error_deg"].set(f"{getattr(yaw_outer_loop, 'yaw_error_deg', 0.0):.3f}")
        self.outer_loop_vars["p_cmd_from_angle"].set(f"{getattr(outer_loop, 'p_cmd_rad_s', 0.0):.4f}")
        self.outer_loop_vars["q_cmd_from_angle"].set(f"{getattr(outer_loop, 'q_cmd_rad_s', 0.0):.4f}")
        self.outer_loop_vars["r_cmd_from_heading"].set(f"{getattr(yaw_outer_loop, 'r_cmd_rad_s', 0.0):.4f}")

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
        r_cmd_display = getattr(yaw_outer_loop, 'r_cmd_rad_s', 0.0) if getattr(snapshot, 'outer_loop_running', False) else getattr(command, 'r_cmd_rad_s', 0.0)
        self.controller_vars["p_cmd"].set(f"{p_cmd_display:.4f}")
        self.controller_vars["error_p"].set(f"{getattr(controller, 'error_p_rad_s', 0.0):.4f}")
        self.controller_vars["kp_p"].set(f"{getattr(command, 'kp_p', 0.0):.4f}")
        self.controller_vars["u_roll"].set(f"{getattr(controller, 'u_roll', 0.0):.4f}")
        self.controller_vars["q_cmd"].set(f"{q_cmd_display:.4f}")
        self.controller_vars["error_q"].set(f"{getattr(controller, 'error_q_rad_s', 0.0):.4f}")
        self.controller_vars["kp_q"].set(f"{getattr(command, 'kp_q', 0.0):.4f}")
        self.controller_vars["u_pitch"].set(f"{getattr(controller, 'u_pitch', 0.0):.4f}")
        self.controller_vars["r_cmd"].set(f"{r_cmd_display:.4f}")
        self.controller_vars["error_r"].set(f"{getattr(controller, 'error_r_rad_s', 0.0):.4f}")
        self.controller_vars["kp_r"].set(f"{getattr(command, 'kp_r', 0.0):.4f}")
        self.controller_vars["u_yaw"].set(f"{getattr(controller, 'u_yaw', 0.0):.4f}")
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
            self.runtime.shutdown()
        finally:
            self.root.destroy()
