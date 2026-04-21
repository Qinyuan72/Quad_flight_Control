from __future__ import annotations

from dataclasses import dataclass

try:
    from data_api.models import UneToControlPlaneConfig
except ImportError:
    from ..data_api.models import UneToControlPlaneConfig


@dataclass
class UneToControlPlaneMapper:
    """
    Map raw UNE horizontal quantities into the controller-facing horizontal plane.

    The current control plane is a fixed north-aligned horizontal plane used as a
    compatibility layer for the existing velocity outer loop. It is not the raw
    UNE frame and it is not the craft body frame.

    This mapper only performs explicit axis swap/sign translation. It does not
    apply heading rotation and it does not contain any control law.
    """

    config: UneToControlPlaneConfig = UneToControlPlaneConfig()

    def map_horizontal(self, v_north_m_s: float, v_east_m_s: float) -> tuple[float, float]:
        x_component = float(v_north_m_s)
        y_component = float(v_east_m_s)
        if self.config.swap_ne:
            x_component, y_component = y_component, x_component
        return (
            float(self.config.sign_x) * x_component,
            float(self.config.sign_y) * y_component,
        )
