# Agents Directory

This directory defines the project-management and engineering agents for the F450 drone docking demo.

## How To Use

- Start with `project-manager/agent.md` for any new feature, hardware change, integration task, or test campaign.
- The project manager must confirm requirements with the user and produce a concrete specification before assigning implementation work.
- Engineer agents are invoked only when their specialty is needed for planning, implementation, review, or validation.

## Agent Map

| Agent | File | Primary Use |
| --- | --- | --- |
| Project Manager | `project-manager/agent.md` | Requirements, specs, task assignment, milestone tracking, GitHub workflow, progress reporting |
| Firmware RD Engineer | `engineers/firmware-rd.md` | Firmware logic, performance, APIs, register notes, defect fixes |
| Software/Firmware QA Engineer | `engineers/software-firmware-qa.md` | Functional, corner-case, stress, compatibility, and automation testing |
| System Integration Engineer | `engineers/system-integration.md` | Pixhawk, ArduPilot, companion computer, ESP32, MAVLink, sensor integration |
| Hardware RD Engineer | `engineers/hardware-rd.md` | F450 payload, power, wiring, connector mechanics, sensor placement, safety design |
| Control/Vision Engineer | `engineers/control-vision.md` | AprilTag/ArUco, ranging fusion, outer-loop PID, visual servoing, wind disturbance strategy |

## File Governance

Use these locations unless the project manager explicitly updates the structure:

- Requirements, specifications, risk, architecture, acceptance criteria: `docs/`
- Mechanical fixture and connector notes: `hardware/`
- Companion-computer software: `software/companion/`
- Firmware or microcontroller software, when added: `software/firmware/`
- Test code and automated checks: `tests/`
- Reusable experiment records and tables: `templates/`
- Field logs, photos, and run evidence, when added: `experiments/`
- Agent definitions and role instructions: `.agents/`

## GitHub Policy

GitHub updates are controlled automation, not blind pushing:

1. Check repository state before work.
2. Make scoped changes.
3. Run the relevant tests.
4. Commit with a clear message when tests pass.
5. Push only when a Git remote is configured and the user has approved the target repository or branch.
6. If the folder is not a Git repository, or no remote exists, stop and ask the user for the GitHub repository URL.
