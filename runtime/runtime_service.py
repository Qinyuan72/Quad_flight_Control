from __future__ import annotations

import sys
import threading
from typing import Any

try:
    from data_api.models import (
        AltitudeCommand,
        AltitudeControlConfig,
        AngleCommand,
        AngleOuterLoopConfig,
        RollRateTestCommand,
        RollRateTestState,
    )
    from runtime.settings_store import load_settings, save_settings
    from runtime.test_runtime import RollRateInnerLoopRuntime
except ImportError:
    from ..data_api.models import (
        AltitudeCommand,
        AltitudeControlConfig,
        AngleCommand,
        AngleOuterLoopConfig,
        RollRateTestCommand,
        RollRateTestState,
    )
    from .settings_store import load_settings, save_settings
    from .test_runtime import RollRateInnerLoopRuntime


class RuntimeService:
    """GUI-facing wrapper that owns loop scheduling and persisted settings."""

    def __init__(self, core: RollRateInnerLoopRuntime, *, loop_interval_s: float = 0.1) -> None:
        self.core = core
        self.loop_interval_s = max(0.01, float(loop_interval_s))
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._pending_error: str | None = None
        self._started = False

    def startup(self) -> None:
        with self._lock:
            if self._started:
                return
            self._apply_settings_locked(load_settings())
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._loop,
                name="runtime-service-loop",
                daemon=True,
            )
            self._thread.start()
            self._started = True

    def start(self) -> None:
        self.startup()

    def shutdown(self) -> None:
        with self._lock:
            if not self._started and self._thread is None:
                return
            self._stop_event.set()
            if getattr(self.core.get_snapshot().binding, "test_running", False):
                try:
                    self.core.stop_test()
                except Exception as exc:
                    self.core.record_background_error(str(exc))
            self._save_settings_locked()
            try:
                self.core.disconnect()
            except Exception as exc:
                self.core.record_background_error(str(exc))
            thread = self._thread
            self._thread = None
            self._started = False

        if thread is not None and thread.is_alive():
            thread.join(timeout=max(1.0, self.loop_interval_s * 5.0))

    def get_snapshot(self) -> RollRateTestState:
        with self._lock:
            return self.core.get_snapshot()

    def consume_pending_error(self) -> str | None:
        with self._lock:
            pending = self._pending_error
            self._pending_error = None
            return pending

    def connect(self) -> None:
        self._call_locked(self.core.connect)

    def disconnect(self) -> None:
        self._call_locked(self.core.disconnect)

    def bind(self) -> None:
        self._call_locked(self.core.bind)

    def initialize(self) -> None:
        self._call_locked(self.core.initialize)

    def prepare_hardware(self) -> None:
        stages = (
            ("connect", self.core.connect),
            ("bind", self.core.bind),
            ("initialize", self.core.initialize),
        )
        with self._lock:
            for stage_name, action in stages:
                try:
                    action()
                except Exception as exc:
                    message = f"Hardware setup failed during {stage_name}: {exc}"
                    print(message, file=sys.stderr)
                    self.core.record_background_error(message)
                    self._pending_error = message
                    raise

    def emergency_stop(self) -> None:
        self._call_locked(self.core.emergency_stop)

    def start_test(self) -> None:
        self._call_locked(self.core.start_test)

    def stop_test(self) -> None:
        self._call_locked(self.core.stop_test)

    def set_command(self, command: RollRateTestCommand) -> None:
        self._call_locked(self.core.set_command, command, persist=True)

    def set_angle_command(self, angle_command: AngleCommand) -> None:
        self._call_locked(self.core.set_angle_command, angle_command, persist=True)

    def set_outer_loop_config(self, config: AngleOuterLoopConfig) -> None:
        self._call_locked(self.core.set_outer_loop_config, config, persist=True)

    def start_outer_loop(self) -> None:
        self._call_locked(self.core.start_outer_loop)

    def stop_outer_loop(self) -> None:
        self._call_locked(self.core.stop_outer_loop)

    def set_altitude_command(self, altitude_command: AltitudeCommand) -> None:
        self._call_locked(self.core.set_altitude_command, altitude_command, persist=True)

    def set_altitude_config(self, config: AltitudeControlConfig) -> None:
        self._call_locked(self.core.set_altitude_config, config, persist=True)

    def start_altitude_loop(self) -> None:
        self._call_locked(self.core.start_altitude_loop)

    def stop_altitude_loop(self) -> None:
        self._call_locked(self.core.stop_altitude_loop)

    def set_mixer_candidate(self, name: str) -> None:
        self._call_locked(self.core.set_mixer_candidate, name, persist=True)

    def get_mixer_names(self) -> list[str]:
        with self._lock:
            return self.core.get_mixer_names()

    def _call_locked(self, action, *args: Any, persist: bool = False) -> Any:
        with self._lock:
            result = action(*args)
            self._pending_error = None
            if persist:
                self._save_settings_locked()
            return result

    def _loop(self) -> None:
        while not self._stop_event.wait(self.loop_interval_s):
            with self._lock:
                snapshot = self.core.get_snapshot()
                if not getattr(snapshot.binding, "connected", False):
                    continue

                try:
                    if getattr(snapshot.binding, "test_running", False):
                        self.core.step_once()
                    else:
                        self.core.preview_once()
                except Exception as exc:
                    message = str(exc)
                    try:
                        if getattr(snapshot.binding, "test_running", False):
                            self.core.stop_test()
                    except Exception:
                        pass
                    self.core.record_background_error(message)
                    if self._pending_error is None:
                        self._pending_error = message

    def _apply_settings_locked(self, settings: dict[str, Any]) -> None:
        command = settings.get("command", {})
        self.core.set_command(
            RollRateTestCommand(
                base_rpm=float(command.get("base_rpm", 0.0)),
                p_cmd_rad_s=float(command.get("p_cmd_rad_s", 0.0)),
                q_cmd_rad_s=float(command.get("q_cmd_rad_s", 0.0)),
                kp_p=float(command.get("kp_p", 0.0)),
                kp_q=float(command.get("kp_q", 0.0)),
                output_limit=float(command.get("output_limit", 0.0)),
            )
        )

        angle_command = settings.get("angle_command", {})
        self.core.set_angle_command(
            AngleCommand(
                roll_cmd_deg=float(angle_command.get("roll_cmd_deg", 0.0)),
                pitch_cmd_deg=float(angle_command.get("pitch_cmd_deg", 0.0)),
            )
        )

        outer_config = settings.get("outer_config", {})
        self.core.set_outer_loop_config(
            AngleOuterLoopConfig(
                kp_roll_angle=float(outer_config.get("kp_roll_angle", 0.0)),
                kp_pitch_angle=float(outer_config.get("kp_pitch_angle", 0.0)),
                rate_limit_rad_s=float(outer_config.get("rate_limit_rad_s", 0.0)),
            )
        )

        altitude_command = settings.get("altitude_command", {})
        self.core.set_altitude_command(
            AltitudeCommand(
                alt_cmd_m=float(altitude_command.get("alt_cmd_m", 0.0)),
                hover_throttle=float(altitude_command.get("hover_throttle", 0.0)),
            )
        )

        altitude_config = settings.get("altitude_config", {})
        self.core.set_altitude_config(
            AltitudeControlConfig(
                kp_alt=float(altitude_config.get("kp_alt", 0.0)),
                vz_max=float(altitude_config.get("vz_max", 0.0)),
                kp_vz=float(altitude_config.get("kp_vz", 0.0)),
                ki_vz=float(altitude_config.get("ki_vz", 0.0)),
                throttle_min=float(altitude_config.get("throttle_min", 0.0)),
                throttle_max=float(altitude_config.get("throttle_max", 0.0)),
            )
        )

        mixer_name = str(settings.get("mixer_name", ""))
        available_mixers = tuple(self.core.get_mixer_names())
        if mixer_name in available_mixers:
            self.core.set_mixer_candidate(mixer_name)
        elif available_mixers:
            self.core.set_mixer_candidate(str(available_mixers[0]))

        self._save_settings_locked()

    def _save_settings_locked(self) -> None:
        snapshot = self.core.get_snapshot()
        save_settings(
            {
                "command": {
                    "base_rpm": snapshot.command.base_rpm,
                    "p_cmd_rad_s": snapshot.command.p_cmd_rad_s,
                    "q_cmd_rad_s": snapshot.command.q_cmd_rad_s,
                    "kp_p": snapshot.command.kp_p,
                    "kp_q": snapshot.command.kp_q,
                    "output_limit": snapshot.command.output_limit,
                },
                "angle_command": {
                    "roll_cmd_deg": snapshot.angle_command.roll_cmd_deg,
                    "pitch_cmd_deg": snapshot.angle_command.pitch_cmd_deg,
                },
                "outer_config": {
                    "kp_roll_angle": snapshot.outer_loop_config.kp_roll_angle,
                    "kp_pitch_angle": snapshot.outer_loop_config.kp_pitch_angle,
                    "rate_limit_rad_s": snapshot.outer_loop_config.rate_limit_rad_s,
                },
                "altitude_command": {
                    "alt_cmd_m": snapshot.altitude_command.alt_cmd_m,
                    "hover_throttle": snapshot.altitude_command.hover_throttle,
                },
                "altitude_config": {
                    "kp_alt": snapshot.altitude_control_config.kp_alt,
                    "vz_max": snapshot.altitude_control_config.vz_max,
                    "kp_vz": snapshot.altitude_control_config.kp_vz,
                    "ki_vz": snapshot.altitude_control_config.ki_vz,
                    "throttle_min": snapshot.altitude_control_config.throttle_min,
                    "throttle_max": snapshot.altitude_control_config.throttle_max,
                },
                "mixer_name": snapshot.mixer_name,
            }
        )
