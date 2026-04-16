# p_inner_loop_test_v1

`p_inner_loop_test_v1` is a narrow engineering harness for validating only the single-axis roll-rate inner loop:

`p_cmd -> P controller -> matrix mixer -> 4 KAL Play Position outputs`

with `p_meas` fed back from the vessel body-rate telemetry path.

## Scope

Implemented:
- kRPC connect/disconnect
- exact-tag binding for 4 KAL controllers and 4 rotors
- rotor/controller initialization using validated ideas
- minimum p-axis telemetry with a known-good body-rate frame approach
- P-only roll-rate controller
- configurable matrix-based mixer candidates
- Tkinter GUI with layered displays and visible mixer matrix text

Intentionally not implemented:
- q/r inner loops
- PID
- outer attitude loop
- altitude or position control
- legacy project migration

## Directory layout

- `control/`: P controller and matrix mixer logic
- `data_api/`: dataclasses, raw kRPC bindings, and minimal telemetry reader
- `gui/`: Tkinter application and GUI constants
- `runtime/`: one-step orchestration for the test harness
- `main.py`: entry point

## Design notes

- The GUI talks only to the runtime.
- Raw kRPC hardware operations stay in `data_api/krpc_bindings.py`.
- Roll-rate measurement and frame handling stay in `data_api/telemetry.py`.
- The mixer uses a 4x4 matrix over `[base, roll, pitch, yaw]`, even though only `roll` is active now.
- `Play Position` is used as the actuator command path.

## Expected tags

Controller tags:
- `ControllerFL`
- `ControllerFR`
- `ControllerRL`
- `ControllerRR`

Rotor tags:
- `rotorFL`
- `rotorFR`
- `rotorRL`
- `rotorRR`

## How to run

From the project root:

```powershell
python -m p_inner_loop_test_v1.main
```

Or directly:

```powershell
python -u "c:\Develop\kPRC\ConnectionTest\p_inner_loop_test_v1\main.py"
```

## Typical workflow

1. Connect.
2. Bind Tags.
3. Initialize.
4. Set `base_rpm`, `p_cmd`, `kp_p`, and `output_limit`.
5. Select a mixer candidate.
6. Start Test.
7. Observe `p_meas`, `error_p`, `u_roll`, the visible matrix, and the four KAL output commands.
8. Use Emergency Stop if needed.
