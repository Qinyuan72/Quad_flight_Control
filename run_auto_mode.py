from __future__ import annotations
import sys
import time

from rich.console import Console
from rich.table import Table
from rich.live import Live

from data_api.models import AltitudeCommand
from runtime.runtime_service import RuntimeService

def run_auto_mode(service: RuntimeService) -> None:
    service.startup()
    try:
        service.prepare_hardware()

        # 如有需要，在这里设置默认参数
        # service.set_command(...)
        # service.set_angle_command(...)
        # service.set_outer_loop_config(...)
        # service.set_altitude_config(...)

        service.start_test()
        service.start_outer_loop()
        service.start_altitude_loop()

        run_vz_profile(
            service,
            profile=[
                (5.0, 5.0),
                (0.0, 60.0),
                (-1.0, 55.0),
            ],
            hover_throttle=277.0,   # 按你当前系统实际值调整
            refresh_period_s=0.2,
        )

        print("\nFinal snapshot:", file=sys.stderr)
        print_snapshot_summary(service.get_snapshot())

        service.stop_altitude_loop()
        service.stop_outer_loop()
        service.stop_test()

    except Exception as exc:
        print(f"Auto mode failed: {exc}", file=sys.stderr)
        raise
    finally:
        service.shutdown()

console = Console()
def run_vz_profile(
    service,
    profile: list[tuple[float, float]],
    hover_throttle: float,
    refresh_period_s: float = 0.2,
) -> None:
    from data_api.models import AltitudeCommand

    with Live(build_telemetry_table(service.get_snapshot()), console=console, refresh_per_second=4) as live:
        for vz_cmd, duration_s in profile:
            service.set_altitude_command(
                AltitudeCommand(
                    alt_cmd_m=vz_cmd,   # 你当前项目里这个字段实际上当作 vz_cmd 在用
                    hover_throttle=hover_throttle,
                )
            )

            t_end = time.time() + duration_s
            while time.time() < t_end:
                snapshot = service.get_snapshot()
                live.update(build_telemetry_table(snapshot))
                time.sleep(refresh_period_s)

def print_snapshot_summary(snapshot) -> None:
    binding = getattr(snapshot, "binding", None)
    telemetry = getattr(snapshot, "telemetry", None)
    motor_command = getattr(snapshot, "motor_command", None)
    motor_mapping = motor_command.as_mapping() if motor_command is not None else {}

    print("Auto Mode Snapshot")
    print(f"  connected: {getattr(binding, 'connected', False)}")
    print(f"  bound: {getattr(binding, 'bound', False)}")
    print(f"  initialized: {getattr(binding, 'initialized', False)}")
    print(f"  test_running: {getattr(binding, 'test_running', False)}")
    print(f"  vessel: {getattr(binding, 'vessel_name', '-')}")
    print(
        "  attitude_deg: "
        f"roll={getattr(telemetry, 'roll_deg', 0.0):.3f}, "
        f"pitch={getattr(telemetry, 'pitch_deg', 0.0):.3f}, "
        f"heading={getattr(telemetry, 'heading_deg', 0.0):.3f}"
    )
    print(
        "  body_rates_rad_s: "
        f"p={getattr(telemetry, 'p_meas_rad_s', 0.0):.4f}, "
        f"q={getattr(telemetry, 'q_meas_rad_s', 0.0):.4f}, "
        f"r={getattr(telemetry, 'r_meas_rad_s', 0.0):.4f}"
    )
    print(
        "  motors: "
        f"FL={float(motor_mapping.get('FL', 0.0)):.2f}, "
        f"FR={float(motor_mapping.get('FR', 0.0)):.2f}, "
        f"RL={float(motor_mapping.get('RL', 0.0)):.2f}, "
        f"RR={float(motor_mapping.get('RR', 0.0)):.2f}"
    )
    print(f"  last_status: {getattr(snapshot, 'last_status', 'Idle')}")
    print(f"  last_error: {getattr(snapshot, 'last_error', '')}")


def build_telemetry_table(snapshot) -> Table:
    binding = getattr(snapshot, "binding", None)
    telemetry = getattr(snapshot, "telemetry", None)
    altitude_loop = getattr(snapshot, "altitude_loop", None)
    motor_command = getattr(snapshot, "motor_command", None)

    motors = motor_command.as_mapping() if motor_command is not None else {}

    table = Table(title="Quad Flight Control Telemetry", expand=True)
    table.add_column("Field", style="cyan", no_wrap=True)
    table.add_column("Value", style="magenta")

    table.add_row("Connected", str(getattr(binding, "connected", False)))
    table.add_row("Running", str(getattr(binding, "test_running", False)))
    table.add_row("Vessel", str(getattr(binding, "vessel_name", "-")))

    table.add_row("Roll (deg)", f"{getattr(telemetry, 'roll_deg', 0.0):7.3f}")
    table.add_row("Pitch (deg)", f"{getattr(telemetry, 'pitch_deg', 0.0):7.3f}")
    table.add_row("Heading (deg)", f"{getattr(telemetry, 'heading_deg', 0.0):7.3f}")

    table.add_row("Altitude (m)", f"{getattr(telemetry, 'alt_m', 0.0):8.3f}")
    table.add_row("Vz (m/s)", f"{getattr(telemetry, 'vz_m_s', 0.0):7.3f}")
    table.add_row("Vz Cmd (m/s)", f"{getattr(altitude_loop, 'vz_cmd_m_s', 0.0):7.3f}")
    table.add_row("Throttle Cmd", f"{getattr(altitude_loop, 'throttle_cmd', 0.0):7.3f}")

    table.add_row("p (rad/s)", f"{getattr(telemetry, 'p_meas_rad_s', 0.0):7.4f}")
    table.add_row("q (rad/s)", f"{getattr(telemetry, 'q_meas_rad_s', 0.0):7.4f}")
    table.add_row("r (rad/s)", f"{getattr(telemetry, 'r_meas_rad_s', 0.0):7.4f}")

    table.add_row("Motor FL", f"{float(motors.get('FL', 0.0)):6.2f}")
    table.add_row("Motor FR", f"{float(motors.get('FR', 0.0)):6.2f}")
    table.add_row("Motor RL", f"{float(motors.get('RL', 0.0)):6.2f}")
    table.add_row("Motor RR", f"{float(motors.get('RR', 0.0)):6.2f}")

    table.add_row("Last Status", str(getattr(snapshot, "last_status", "Idle")))
    table.add_row("Last Error", str(getattr(snapshot, "last_error", "")))

    return table