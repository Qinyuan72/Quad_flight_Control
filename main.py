from __future__ import annotations

import argparse
import tkinter as tk

from run_auto_mode import run_auto_mode

if __package__ in {None, ""}:
    from gui.app_gui import RollRateTestApp
    from runtime.runtime_service import RuntimeService
    from runtime.test_runtime import RollRateInnerLoopRuntime
else:
    from .gui.app_gui import RollRateTestApp
    from .runtime.runtime_service import RuntimeService
    from .runtime.test_runtime import RollRateInnerLoopRuntime


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Quad flight control harness")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--gui", action="store_true", help="run the Tkinter GUI")
    mode.add_argument("--auto", action="store_true", help="run a minimal headless test flow")
    return parser.parse_args()


def build_service() -> RuntimeService:
    return RuntimeService(RollRateInnerLoopRuntime())

def main() -> None:
    args = parse_args()
    service = build_service()
    if args.auto:
        run_auto_mode(service)
        return
    run_gui_mode(service)


if __name__ == "__main__":
    main()
