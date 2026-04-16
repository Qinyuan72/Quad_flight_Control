from __future__ import annotations

import sys
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table

from data_api.models import AltitudeCommand, AltitudeControlConfig, RollRateTestCommand
from runtime.runtime_service import RuntimeService


console = Console()
EVENT_LINES = 4
LEFT_TABLE_ROWS = 16
RIGHT_TABLE_ROWS = 10


@dataclass(frozen=True)
class TimedCommand:
    label: str
    value: float
    duration_s: float


@dataclass(frozen=True)
class HoverWaitConfig:
    min_margin_m: float = 2.0
    alt_tolerance_m: float = 1.5
    vz_tolerance_m_s: float = 0.75
    stable_time_s: float = 2.0
    timeout_s: float = 15.0


@dataclass(frozen=True)
class DisplayConfig:
    poll_s: float = 0.20
    refresh_hz: float = 2.0


@dataclass(frozen=True)
class YawTestConfig:
    enabled: bool = True
    kp_r: float = 50
    output_limit: float = 200
    steps: tuple[TimedCommand, ...] = (
        TimedCommand("neutral_hold", 0.00, 1.5),
        TimedCommand("yaw_right", 1, 20),
        TimedCommand("recover_center", 0.00, 10),
        TimedCommand("yaw_left", -1, 20),
        TimedCommand("recover_center", 0.00, 20),
    )


@dataclass(frozen=True)
class AltitudeProfileConfig:
    enabled: bool = True
    steps: tuple[TimedCommand, ...] = (
        TimedCommand("climb_check", 119.0, 12.0),
        TimedCommand("descend_check", 89.0, 8.0),
        TimedCommand("return_hover", 74.0, 6.0),
    )


@dataclass(frozen=True)
class FlightConfig:
    hover_throttle: float = 290.0
    hover_alt_m: float = 74.0
    recovery_alt_m: float = 71.0
    hover_settle_s: float = 2.0
    recovery_hold_s: float = 4.0
    altitude: AltitudeControlConfig = field(
        default_factory=lambda: AltitudeControlConfig(
            kp_alt=0.5,
            vz_max=5.0,
            kp_vz=20.0,
            ki_vz=0.0,
            throttle_min=60.0,
            throttle_max=400.0,
        )
    )


@dataclass(frozen=True)
class RunFlags:
    run_takeoff: bool = True
    run_hover_settle: bool = True
    run_yaw_test: bool = True
    run_altitude_profile: bool = True
    run_recovery: bool = True
    print_final_summary: bool = True


@dataclass(frozen=True)
class AutoModeConfig:
    display: DisplayConfig = field(default_factory=DisplayConfig)
    hover_wait: HoverWaitConfig = field(default_factory=HoverWaitConfig)
    flight: FlightConfig = field(default_factory=FlightConfig)
    yaw_test: YawTestConfig = field(default_factory=YawTestConfig)
    altitude_profile: AltitudeProfileConfig = field(default_factory=AltitudeProfileConfig)
    flags: RunFlags = field(default_factory=RunFlags)


DEFAULT_CONFIG = AutoModeConfig()


@dataclass
class DashboardContext:
    snapshot: object
    phase: str = "STARTUP"
    test_name: str = "NONE"
    step_label: str = "-"
    step_elapsed_s: float = 0.0
    step_duration_s: float = 0.0
    module_title: str = "Module"
    module_rows: list[tuple[str, str]] = field(default_factory=list)
    recent_events: deque[str] = field(default_factory=lambda: deque(maxlen=EVENT_LINES))

    def add_event(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.recent_events.appendleft(f"[{stamp}] {message}")


ModuleBuilder = Callable[[DashboardContext], None]


def _fmt_float(value: float, width: int = 7, precision: int = 3) -> str:
    return f"{float(value):{width}.{precision}f}"


def _get_yaw_differential(snapshot: object) -> float:
    motor_command = getattr(snapshot, "motor_command", None)
    if motor_command is None:
        return 0.0
    motors = motor_command.as_mapping()
    return (
        float(motors.get("FL", 0.0))
        - float(motors.get("FR", 0.0))
        - float(motors.get("RL", 0.0))
        + float(motors.get("RR", 0.0))
    )


def _refresh_context(service: RuntimeService, context: DashboardContext) -> None:
    context.snapshot = service.get_snapshot()


def _pad_rows(rows: list[tuple[str, str]], target_rows: int) -> list[tuple[str, str]]:
    padded = rows[:target_rows]
    while len(padded) < target_rows:
        padded.append(("", ""))
    return padded


def _build_default_rows(context: DashboardContext) -> list[tuple[str, str]]:
    snapshot = context.snapshot
    binding = getattr(snapshot, "binding", None)
    telemetry = getattr(snapshot, "telemetry", None)
    altitude_loop = getattr(snapshot, "altitude_loop", None)

    rows: list[tuple[str, str]] = [
        ("Phase", context.phase),
        ("Test", context.test_name),
        ("Step", context.step_label),
        ("Vessel", str(getattr(binding, "vessel_name", "-"))),
        ("Running", str(getattr(binding, "test_running", False))),
        ("Status", str(getattr(snapshot, "last_status", "Idle"))),
        ("Error", str(getattr(snapshot, "last_error", "")) or "-"),
        (
            "Alt / Cmd",
            f"{_fmt_float(getattr(telemetry, 'alt_m', 0.0), 8, 2)} / {_fmt_float(getattr(altitude_loop, 'alt_cmd_m', 0.0), 8, 2)} m",
        ),
        (
            "Vz / Thr",
            f"{_fmt_float(getattr(telemetry, 'vz_m_s', 0.0), 7, 2)} m/s  |  {_fmt_float(getattr(altitude_loop, 'throttle_cmd', 0.0), 7, 1)}",
        ),
        (
            "Roll / Pitch",
            f"{_fmt_float(getattr(telemetry, 'roll_deg', 0.0), 7, 2)} / {_fmt_float(getattr(telemetry, 'pitch_deg', 0.0), 7, 2)} deg",
        ),
        ("Heading", f"{_fmt_float(getattr(telemetry, 'heading_deg', 0.0), 8, 2)} deg"),
        (
            "Rates p/q/r",
            f"{_fmt_float(getattr(telemetry, 'p_meas_rad_s', 0.0), 7, 3)} / "
            f"{_fmt_float(getattr(telemetry, 'q_meas_rad_s', 0.0), 7, 3)} / "
            f"{_fmt_float(getattr(telemetry, 'r_meas_rad_s', 0.0), 7, 3)} rad/s",
        ),
    ]

    event_rows = list(context.recent_events)
    while len(event_rows) < EVENT_LINES:
        event_rows.append("-")
    rows.extend((f"Log {idx + 1}", entry) for idx, entry in enumerate(event_rows[:EVENT_LINES]))
    return _pad_rows(rows, LEFT_TABLE_ROWS)


def _make_panel(title: str, rows: list[tuple[str, str]], *, field_width: int, value_width: int, border_style: str) -> Panel:
    inner = Table(show_header=False, box=None, expand=False, pad_edge=False)
    inner.add_column(width=field_width, no_wrap=True, style="cyan")
    inner.add_column(width=value_width, no_wrap=True, style="white")
    for field_name, value in rows:
        inner.add_row(field_name, value)
    return Panel(inner, title=title, border_style=border_style, padding=(0, 1))


def build_dashboard(context: DashboardContext) -> Layout:
    default_rows = _build_default_rows(context)
    module_rows = _pad_rows(context.module_rows, RIGHT_TABLE_ROWS)

    layout = Layout()
    layout.split_row(
        Layout(name="default", ratio=3),
        Layout(name="module", ratio=2),
    )
    layout["default"].update(
        _make_panel("Flight Overview", default_rows, field_width=12, value_width=42, border_style="green")
    )
    layout["module"].update(
        _make_panel(context.module_title, module_rows, field_width=14, value_width=34, border_style="magenta")
    )
    return layout


def set_idle_module(context: DashboardContext) -> None:
    context.module_title = "Module"
    context.module_rows = [
        ("Mode", "Monitoring"),
        (
            "Progress",
            f"{context.step_elapsed_s:4.1f} / {context.step_duration_s:4.1f} s"
            if context.step_duration_s > 0.0
            else "-",
        ),
        ("Active Test", context.test_name),
        ("Active Step", context.step_label),
        ("Yaw Diff", f"{_fmt_float(_get_yaw_differential(context.snapshot), 8, 2)}"),
    ]


def set_yaw_module(context: DashboardContext) -> None:
    snapshot = context.snapshot
    telemetry = getattr(snapshot, "telemetry", None)
    command = getattr(snapshot, "command", None)
    controller = getattr(snapshot, "controller", None)
    motor_command = getattr(snapshot, "motor_command", None)
    motors = motor_command.as_mapping() if motor_command is not None else {}

    context.module_title = "Yaw Test"
    context.module_rows = [
        ("Step", context.step_label),
        ("Progress", f"{context.step_elapsed_s:4.1f} / {context.step_duration_s:4.1f} s"),
        ("r_cmd", f"{_fmt_float(getattr(command, 'r_cmd_rad_s', 0.0), 8, 4)} rad/s"),
        ("r_meas", f"{_fmt_float(getattr(telemetry, 'r_meas_rad_s', 0.0), 8, 4)} rad/s"),
        ("error_r", f"{_fmt_float(getattr(controller, 'error_r_rad_s', 0.0), 8, 4)} rad/s"),
        ("u_yaw", f"{_fmt_float(getattr(controller, 'u_yaw', 0.0), 8, 4)}"),
        ("Yaw Diff", f"{_fmt_float(_get_yaw_differential(snapshot), 8, 2)}"),
        (
            "Motors",
            " ".join(
                [
                    f"FL:{float(motors.get('FL', 0.0)):6.1f}",
                    f"FR:{float(motors.get('FR', 0.0)):6.1f}",
                    f"RL:{float(motors.get('RL', 0.0)):6.1f}",
                    f"RR:{float(motors.get('RR', 0.0)):6.1f}",
                ]
            ),
        ),
    ]


def set_altitude_module(context: DashboardContext) -> None:
    snapshot = context.snapshot
    telemetry = getattr(snapshot, "telemetry", None)
    altitude_loop = getattr(snapshot, "altitude_loop", None)

    context.module_title = "Altitude Profile"
    context.module_rows = [
        ("Step", context.step_label),
        ("Progress", f"{context.step_elapsed_s:4.1f} / {context.step_duration_s:4.1f} s"),
        ("Alt Cmd", f"{_fmt_float(getattr(altitude_loop, 'alt_cmd_m', 0.0), 8, 2)} m"),
        ("Altitude", f"{_fmt_float(getattr(telemetry, 'alt_m', 0.0), 8, 2)} m"),
        ("Alt Error", f"{_fmt_float(getattr(altitude_loop, 'alt_error_m', 0.0), 8, 2)} m"),
        ("Vz Cmd", f"{_fmt_float(getattr(altitude_loop, 'vz_cmd_m_s', 0.0), 8, 2)} m/s"),
        ("Vz", f"{_fmt_float(getattr(telemetry, 'vz_m_s', 0.0), 8, 2)} m/s"),
        ("Throttle", f"{_fmt_float(getattr(altitude_loop, 'throttle_cmd', 0.0), 8, 1)}"),
    ]


def update_live(
    live: Live,
    service: RuntimeService,
    context: DashboardContext,
    module_builder: ModuleBuilder,
) -> None:
    _refresh_context(service, context)
    module_builder(context)
    live.update(build_dashboard(context), refresh=True)


def wait_until_hover_ready(
    service: RuntimeService,
    live: Live,
    context: DashboardContext,
    *,
    flight: FlightConfig,
    hover_wait: HoverWaitConfig,
    display: DisplayConfig,
) -> None:
    target_alt_m = flight.hover_alt_m
    min_alt_m = target_alt_m - hover_wait.min_margin_m
    deadline = time.time() + hover_wait.timeout_s
    stable_since: float | None = None

    context.phase = "TAKEOFF"
    context.test_name = "TAKEOFF_HOVER"
    context.step_label = "climb_to_hover"
    context.step_duration_s = hover_wait.timeout_s
    start_t = time.time()

    while time.time() < deadline:
        context.step_elapsed_s = min(time.time() - start_t, hover_wait.timeout_s)
        update_live(live, service, context, set_idle_module)

        snapshot = context.snapshot
        telemetry = getattr(snapshot, "telemetry", None)
        alt_m = float(getattr(telemetry, "alt_m", 0.0))
        vz_m_s = float(getattr(telemetry, "vz_m_s", 0.0))

        high_enough = alt_m >= min_alt_m
        near_target = abs(alt_m - target_alt_m) <= hover_wait.alt_tolerance_m
        vz_small = abs(vz_m_s) <= hover_wait.vz_tolerance_m_s

        if high_enough and near_target and vz_small:
            if stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since >= hover_wait.stable_time_s:
                context.add_event("Hover stable and ready.")
                context.phase = "HOVER"
                context.step_label = "hover_ready"
                context.step_duration_s = 0.0
                context.step_elapsed_s = 0.0
                update_live(live, service, context, set_idle_module)
                return
        else:
            stable_since = None

        time.sleep(display.poll_s)

    raise RuntimeError(f"Hover readiness timeout at target {target_alt_m:.2f} m.")


def hold_phase(
    service: RuntimeService,
    live: Live,
    context: DashboardContext,
    *,
    phase: str,
    test_name: str,
    step_label: str,
    duration_s: float,
    module_builder: ModuleBuilder,
    display: DisplayConfig,
) -> None:
    context.phase = phase
    context.test_name = test_name
    context.step_label = step_label
    context.step_duration_s = duration_s
    context.step_elapsed_s = 0.0
    start_t = time.time()
    while True:
        elapsed = time.time() - start_t
        context.step_elapsed_s = min(elapsed, duration_s)
        update_live(live, service, context, module_builder)
        if elapsed >= duration_s:
            break
        time.sleep(display.poll_s)
    context.step_duration_s = 0.0
    context.step_elapsed_s = 0.0


def run_hover_yaw_test(
    service: RuntimeService,
    live: Live,
    context: DashboardContext,
    *,
    flight: FlightConfig,
    hover_wait: HoverWaitConfig,
    yaw_test: YawTestConfig,
    display: DisplayConfig,
) -> None:
    snapshot = service.get_snapshot()
    binding = getattr(snapshot, "binding", None)
    telemetry = getattr(snapshot, "telemetry", None)
    base_command = snapshot.command

    if not getattr(binding, "test_running", False):
        raise RuntimeError("Yaw test requires test loop to be running.")
    if float(getattr(telemetry, "alt_m", 0.0)) < flight.hover_alt_m - hover_wait.min_margin_m:
        raise RuntimeError(f"Yaw test requires altitude >= {flight.hover_alt_m - hover_wait.min_margin_m:.2f} m.")

    original_command = base_command
    yaw_command = RollRateTestCommand(
        base_rpm=base_command.base_rpm,
        p_cmd_rad_s=0.0,
        q_cmd_rad_s=0.0,
        r_cmd_rad_s=0.0,
        kp_p=base_command.kp_p,
        kp_q=base_command.kp_q,
        kp_r=yaw_test.kp_r,
        output_limit=yaw_test.output_limit,
    )

    context.phase = "YAW_TEST"
    context.test_name = "HOVER_YAW"
    context.add_event("Yaw hover test started.")

    try:
        for step in yaw_test.steps:
            service.set_command(
                RollRateTestCommand(
                    base_rpm=yaw_command.base_rpm,
                    p_cmd_rad_s=0.0,
                    q_cmd_rad_s=0.0,
                    r_cmd_rad_s=step.value,
                    kp_p=yaw_command.kp_p,
                    kp_q=yaw_command.kp_q,
                    kp_r=yaw_command.kp_r,
                    output_limit=yaw_command.output_limit,
                )
            )
            context.add_event(f"Yaw step {step.label}: r_cmd={step.value:+.2f} rad/s.")
            hold_phase(
                service,
                live,
                context,
                phase="YAW_TEST",
                test_name="HOVER_YAW",
                step_label=step.label,
                duration_s=step.duration_s,
                module_builder=set_yaw_module,
                display=display,
            )

            _refresh_context(service, context)
            controller = getattr(context.snapshot, "controller", None)
            telemetry = getattr(context.snapshot, "telemetry", None)
            context.add_event(
                "Yaw done: "
                f"r={float(getattr(telemetry, 'r_meas_rad_s', 0.0)):+.3f}, "
                f"err={float(getattr(controller, 'error_r_rad_s', 0.0)):+.3f}, "
                f"u={float(getattr(controller, 'u_yaw', 0.0)):+.2f}"
            )
            update_live(live, service, context, set_yaw_module)
    finally:
        service.set_command(original_command)
        context.test_name = "NONE"
        context.step_label = "-"
        context.step_duration_s = 0.0
        context.step_elapsed_s = 0.0
        context.add_event("Yaw hover test finished.")
        update_live(live, service, context, set_idle_module)


def run_altitude_profile(
    service: RuntimeService,
    live: Live,
    context: DashboardContext,
    *,
    flight: FlightConfig,
    altitude_profile: AltitudeProfileConfig,
    display: DisplayConfig,
) -> None:
    context.phase = "MISSION"
    context.test_name = "ALT_PROFILE"
    context.add_event("Altitude profile started.")

    for step in altitude_profile.steps:
        service.set_altitude_command(AltitudeCommand(alt_cmd_m=step.value, hover_throttle=flight.hover_throttle))
        context.add_event(f"Alt step {step.label}: alt_cmd={step.value:.1f} m.")
        hold_phase(
            service,
            live,
            context,
            phase="MISSION",
            test_name="ALT_PROFILE",
            step_label=step.label,
            duration_s=step.duration_s,
            module_builder=set_altitude_module,
            display=display,
        )

    context.add_event("Altitude profile finished.")
    context.test_name = "NONE"
    context.step_label = "-"
    update_live(live, service, context, set_idle_module)


def print_final_summary(service: RuntimeService, context: DashboardContext) -> None:
    snapshot = service.get_snapshot()
    binding = getattr(snapshot, "binding", None)
    telemetry = getattr(snapshot, "telemetry", None)
    altitude_loop = getattr(snapshot, "altitude_loop", None)
    controller = getattr(snapshot, "controller", None)

    print("\nAuto Mode Summary", file=sys.stderr)
    print(f"  vessel: {getattr(binding, 'vessel_name', '-')}", file=sys.stderr)
    print(f"  phase: {context.phase}", file=sys.stderr)
    print(
        f"  alt / cmd: {float(getattr(telemetry, 'alt_m', 0.0)):.2f} / {float(getattr(altitude_loop, 'alt_cmd_m', 0.0)):.2f} m",
        file=sys.stderr,
    )
    print(
        f"  vz / r: {float(getattr(telemetry, 'vz_m_s', 0.0)):.2f} m/s  |  {float(getattr(telemetry, 'r_meas_rad_s', 0.0)):.3f} rad/s",
        file=sys.stderr,
    )
    print(
        f"  yaw err / u: {float(getattr(controller, 'error_r_rad_s', 0.0)):.3f} / {float(getattr(controller, 'u_yaw', 0.0)):.2f}",
        file=sys.stderr,
    )
    print(f"  yaw diff: {_get_yaw_differential(snapshot):.2f}", file=sys.stderr)
    print(f"  last status: {getattr(snapshot, 'last_status', 'Idle')}", file=sys.stderr)
    if getattr(snapshot, "last_error", ""):
        print(f"  last error: {getattr(snapshot, 'last_error', '')}", file=sys.stderr)


def run_auto_mode(service: RuntimeService, config: AutoModeConfig = DEFAULT_CONFIG) -> None:
    service.startup()
    context = DashboardContext(snapshot=service.get_snapshot())
    context.add_event("Startup complete.")

    try:
        with Live(build_dashboard(context), console=console, auto_refresh=False, transient=False) as live:
            service.prepare_hardware()
            context.phase = "PREFLIGHT"
            context.add_event("Hardware prepared.")
            update_live(live, service, context, set_idle_module)

            service.set_altitude_config(config.flight.altitude)
            context.add_event(
                "Altitude cfg: "
                f"kp_alt={config.flight.altitude.kp_alt:.2f}, "
                f"kp_vz={config.flight.altitude.kp_vz:.2f}, "
                f"vz_max={config.flight.altitude.vz_max:.1f}"
            )
            update_live(live, service, context, set_idle_module)

            service.start_test()
            service.start_outer_loop()
            service.start_altitude_loop()
            context.add_event("Loops started.")
            update_live(live, service, context, set_idle_module)

            if config.flags.run_takeoff:
                service.set_altitude_command(
                    AltitudeCommand(
                        alt_cmd_m=config.flight.hover_alt_m,
                        hover_throttle=config.flight.hover_throttle,
                    )
                )
                context.add_event(f"Takeoff target set to {config.flight.hover_alt_m:.1f} m.")
                wait_until_hover_ready(
                    service,
                    live,
                    context,
                    flight=config.flight,
                    hover_wait=config.hover_wait,
                    display=config.display,
                )

            if config.flags.run_hover_settle:
                hold_phase(
                    service,
                    live,
                    context,
                    phase="HOVER_CHECK",
                    test_name="STABILIZE",
                    step_label="hover_settle",
                    duration_s=config.flight.hover_settle_s,
                    module_builder=set_idle_module,
                    display=config.display,
                )
                context.add_event("Hover settle complete.")

            if config.flags.run_yaw_test and config.yaw_test.enabled:
                run_hover_yaw_test(
                    service,
                    live,
                    context,
                    flight=config.flight,
                    hover_wait=config.hover_wait,
                    yaw_test=config.yaw_test,
                    display=config.display,
                )

            if config.flags.run_altitude_profile and config.altitude_profile.enabled:
                run_altitude_profile(
                    service,
                    live,
                    context,
                    flight=config.flight,
                    altitude_profile=config.altitude_profile,
                    display=config.display,
                )

            if config.flags.run_recovery:
                service.set_altitude_command(
                    AltitudeCommand(
                        alt_cmd_m=config.flight.recovery_alt_m,
                        hover_throttle=config.flight.hover_throttle,
                    )
                )
                context.add_event(f"Recovery target set to {config.flight.recovery_alt_m:.1f} m.")
                hold_phase(
                    service,
                    live,
                    context,
                    phase="RECOVERY",
                    test_name="RECOVERY",
                    step_label="stabilize_low_hover",
                    duration_s=config.flight.recovery_hold_s,
                    module_builder=set_altitude_module,
                    display=config.display,
                )

            context.phase = "DONE"
            context.test_name = "NONE"
            context.step_label = "complete"
            context.add_event("Auto mode sequence complete.")
            update_live(live, service, context, set_idle_module)

        if config.flags.print_final_summary:
            print_final_summary(service, context)

    except Exception as exc:
        context.phase = "FAILED"
        context.add_event(f"Auto mode failed: {exc}")
        raise
    finally:
        try:
            service.stop_altitude_loop()
            service.stop_outer_loop()
            service.stop_test()
        except Exception:
            pass
        service.shutdown()
