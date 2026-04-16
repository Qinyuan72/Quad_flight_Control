from __future__ import annotations

import tkinter as tk

if __package__ in {None, ""}:
    import sys
    from pathlib import Path

    package_root = Path(__file__).resolve().parent.parent
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

    from Control_loop_test_v1.gui.app_gui import RollRateTestApp
    from Control_loop_test_v1.runtime.test_runtime import RollRateInnerLoopRuntime
else:
    from .gui.app_gui import RollRateTestApp
    from .runtime.test_runtime import RollRateInnerLoopRuntime


def main() -> None:
    runtime = RollRateInnerLoopRuntime()
    root = tk.Tk()
    RollRateTestApp(root, runtime)
    root.mainloop()


if __name__ == "__main__":
    main()
