from __future__ import annotations

from dataclasses import dataclass
from typing import Any

try:
    import krpc
except ImportError:  # pragma: no cover - depends on local environment
    krpc = None

from .models import DEFAULT_CONTROLLER_TAGS, DEFAULT_ROTOR_TAGS, BindingStatus, MotorCommand


class BindingError(RuntimeError):
    """Raised when the expected exact tagged hardware cannot be found."""


@dataclass
class KALControllerBinding:
    key: str
    part: Any
    module: Any

    @property
    def tag(self) -> str:
        return str(self.part.tag)

    def set_play_position(self, value: float) -> None:
        self.module.set_field_float("Play Position", float(value))

    def get_play_position(self) -> float:
        return self._parse_float(dict(self.module.fields).get("Play Position", "0"))

    def set_play_speed_percent(self, value: float) -> None:
        self.module.set_field_float("Play Speed", float(value))

    def set_enabled(self, enabled: bool) -> None:
        self.module.set_field_bool("Enabled", bool(enabled))

    def play(self) -> None:
        self.module.set_action("Play Sequence")

    def stop(self) -> None:
        self.module.set_action("Stop Sequence")

    def set_direction_forward(self) -> None:
        self.module.set_action("Set Play Direction to Forward")

    def set_direction_reverse(self) -> None:
        self.module.set_action("Set Play Direction to Reverse")

    @staticmethod
    def _parse_float(value: object, default: float = 0.0) -> float:
        try:
            return float(str(value).strip().replace("%", ""))
        except Exception:
            return default


@dataclass
class RotorBinding:
    key: str
    part: Any
    rotor: Any

    @property
    def tag(self) -> str:
        return str(self.part.tag)

    def set_motor_engaged(self, engaged: bool) -> None:
        self.rotor.motor_engaged = bool(engaged)

    def set_torque_limit(self, value: float) -> None:
        self.rotor.torque_limit = float(value)

    def set_locked(self, locked: bool) -> None:
        self.rotor.locked = bool(locked)


class KrpcQuadHardware:
    """All raw kRPC interaction for KAL controller and rotor binding/writes."""

    def __init__(self) -> None:
        self.conn: Any | None = None
        self.vessel: Any | None = None
        self.body: Any | None = None
        self.controllers: dict[str, KALControllerBinding] = {}
        self.rotors: dict[str, RotorBinding] = {}
        self.controller_tags = dict(DEFAULT_CONTROLLER_TAGS)
        self.rotor_tags = dict(DEFAULT_ROTOR_TAGS)
        self.initialized = False

    def connect(self, name: str = "p-axis inner-loop test") -> None:
        if self.conn is not None:
            return
        if krpc is None:
            raise RuntimeError("kRPC Python package is not installed.")
        self.conn = krpc.connect(name=name)
        self.vessel = self.conn.space_center.active_vessel
        self.body = self.vessel.orbit.body
        self.initialized = False

    def disconnect(self) -> None:
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass
        self.conn = None
        self.vessel = None
        self.body = None
        self.controllers.clear()
        self.rotors.clear()
        self.initialized = False

    def is_connected(self) -> bool:
        return self.conn is not None and self.vessel is not None and self.body is not None

    def is_bound(self) -> bool:
        return len(self.controllers) == 4 and len(self.rotors) == 4

    def bind(self) -> None:
        self._require_connected()
        assert self.vessel is not None
        self.controllers = {key: self._find_controller(key, tag) for key, tag in self.controller_tags.items()}
        self.rotors = {key: self._find_rotor(key, tag) for key, tag in self.rotor_tags.items()}

    def initialize(
        self,
        *,
        play_speed_percent: float = 100.0,
        zero_play_position: float = 0.0,
        motor_engaged: bool = True,
        torque_limit: float = 100.0,
    ) -> None:
        self._require_bound()
        for rotor in self.rotors.values():
            rotor.set_motor_engaged(motor_engaged)
            rotor.set_torque_limit(torque_limit)
            rotor.set_locked(False)
        for controller in self.controllers.values():
            controller.set_enabled(True)
            controller.set_play_speed_percent(play_speed_percent)
            controller.set_direction_forward()
            controller.set_play_position(zero_play_position)
            #controller.play()
        self.initialized = True

    def write_motor_command(self, command: MotorCommand) -> None:
        self._require_bound()
        for key, rpm in command.as_mapping().items():
            self.controllers[key].set_play_position(rpm)

    def zero_outputs(self) -> None:
        self._require_bound()
        for controller in self.controllers.values():
            controller.set_play_position(0.0)

    def emergency_stop(self) -> None:
        if not self.is_bound():
            return
        self.zero_outputs()
        for controller in self.controllers.values():
            controller.stop()
        for rotor in self.rotors.values():
            rotor.set_motor_engaged(False)

    def get_binding_status(self, *, test_running: bool) -> BindingStatus:
        vessel_name = self.vessel.name if self.vessel is not None else "-"
        controller_tags_bound = {key: binding.tag for key, binding in self.controllers.items()}
        rotor_tags_bound = {key: binding.tag for key, binding in self.rotors.items()}
        return BindingStatus(
            connected=self.is_connected(),
            bound=self.is_bound(),
            initialized=self.initialized,
            test_running=test_running,
            vessel_name=vessel_name,
            controller_tags=controller_tags_bound or dict(self.controller_tags),
            rotor_tags=rotor_tags_bound or dict(self.rotor_tags),
        )

    def _find_controller(self, key: str, tag: str) -> KALControllerBinding:
        assert self.vessel is not None
        parts = list(self.vessel.parts.with_tag(tag))
        if len(parts) != 1:
            raise BindingError(f"Expected exactly one controller with tag {tag}, got {len(parts)}")
        part = parts[0]
        try:
            module = next(module for module in part.modules if module.name == "ModuleRoboticController")
        except StopIteration as exc:
            raise BindingError(f"Part {tag} does not have ModuleRoboticController") from exc
        return KALControllerBinding(key=key, part=part, module=module)

    def _find_rotor(self, key: str, tag: str) -> RotorBinding:
        assert self.vessel is not None
        parts = list(self.vessel.parts.with_tag(tag))
        if len(parts) != 1:
            raise BindingError(f"Expected exactly one rotor with tag {tag}, got {len(parts)}")
        part = parts[0]
        rotor = part.robotic_rotor
        if rotor is None:
            raise BindingError(f"Part {tag} is not a robotic rotor")
        return RotorBinding(key=key, part=part, rotor=rotor)

    def _require_connected(self) -> None:
        if not self.is_connected():
            raise RuntimeError("Not connected to kRPC.")

    def _require_bound(self) -> None:
        if not self.is_bound():
            raise RuntimeError("Hardware is not bound. Call bind() first.")
