# F450 Drone Docking Demo

This repository captures the first demo implementation for a drone-to-cleaning-module docking system.

The first milestone is intentionally conservative:

- F450 frame with Holybro Pixhawk 6C.
- ArduPilot-first integration path.
- Tabletop mechanical validation before powered flight.
- Light payload or dummy payload, target mass <= 300 g.
- Mechanical guide, buffer, lock, and sensor verification before any lift test.

## Repository Map

- `docs/01_requirements.md` - confirmed requirements and success criteria.
- `docs/02_system_architecture.md` - subsystem split, state flow, and interfaces.
- `docs/03_bom.md` - staged purchase list and items to defer.
- `docs/04_experiment_protocol.md` - phase-by-phase experiment procedure.
- `docs/05_risk_register.md` - design blind spots and mitigations.
- `docs/06_acceptance_matrix.md` - verification matrix for tabletop and low-altitude tests.
- `docs/07_ardupilot_integration.md` - ArduPilot/Pixhawk 6C integration notes.
- `docs/09_jetson_terminal_docking_runtime.md` - Jetson log-only/advisory runtime design.
- `docs/10_jetson_pixhawk_mavlink_motor_test.md` - Jetson/Pixhawk MAVLink and no-prop motor-test procedure.
- `.agents/` - project manager and engineering agent role definitions.
- `hardware/connector_test_fixture.md` - tabletop fixture requirements.
- `templates/experiment_log.csv` - per-test logging template.
- `templates/sensor_truth_table.csv` - first-stage sensor validation template.
- `software/companion/docking_state_machine.py` - dependency-free companion-computer state machine prototype.
- `software/companion/jetson/` - Jetson terminal docking range gate, filter, supervisor, and sensor contracts.
- `software/companion/simulate_tabletop_demo.py` - example dry-run sequence.
- `tests/test_docking_state_machine.py` - unit tests for success and safety aborts.

## Quick Start

Run the software checks from the repository root:

```bash
python3 -m unittest discover -s tests
python3 software/companion/simulate_tabletop_demo.py
```

Jetson-only MAVLink tools require:

```bash
python3 -m pip install --user -r requirements-jetson.txt
```

## Safety Boundary

The current software is a deterministic prototype for tabletop and simulation work. It does not directly command a live aircraft. Before connecting to Pixhawk, wire the companion computer through a supervised MAVLink bridge, keep a pilot override active, and verify every state transition on a non-powered fixture.
