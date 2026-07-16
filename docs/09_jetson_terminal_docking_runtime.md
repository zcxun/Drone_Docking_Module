# Jetson 末端對接 Runtime

## 目的

Jetson 第一版只負責最後 30-80 cm 的末端對接定位與狀態機決策。Pixhawk/ArduPilot 仍負責穩定飛行、arm、模式安全、遙控器 failsafe 與飛手接管。

目前 runtime 是 log-only / advisory：會輸出 `ControlCommand`、CSV log 與後續可接 dashboard 的資料，不直接送 MAVLink、不自動 arm、不繞過飛手。

## 模組分工

| 模組 | 位置 | 責任 |
| --- | --- | --- |
| vision node | `software/companion/jetson/vision_node.py` | 包裝 AprilTag pose estimator，輸出 `VisionObservation` |
| sensor node | `software/companion/jetson/sensor_node.py` | 定義 ToF、對接感測、Pixhawk safety telemetry 的輸入資料 |
| range gate | `software/companion/jetson/range_gate.py` | 讓 ToF/LiDAR 成為下降距離主來源，AprilTag range 只做交叉檢查 |
| filter | `software/companion/jetson/filtering.py` | median + EMA 平滑有效 pose，拒絕過大跳變 |
| supervisor | `software/companion/jetson/supervisor_node.py` | 串接 observation、range gate、filter、`SensorSnapshot`、狀態機與 CSV log |

## Runtime 流程

```text
camera frame
-> AprilTag calibrated pose
-> stale/latency gate
-> ToF/LiDAR range gate
-> median/EMA filter
-> SensorSnapshot
-> DockingStateMachine
-> advisory ControlCommand + CSV log
```

第一版的下降距離安全規則：

- 沒有 ToF/LiDAR、ToF invalid、ToF timestamp 過舊，`range_valid=false`。
- ToF range 超出設定範圍，`range_valid=false`。
- AprilTag calibrated range 與 ToF 差距超過 `max_range_disagreement_m`，`range_valid=false`。
- `HORIZONTAL_ALIGN`、`YAW_ALIGN`、`DESCEND` 任一階段遇到 `range_valid=false`，狀態機不得繼續下降。

## 第一版預設參數

| 參數 | 預設 | 說明 |
| --- | --- | --- |
| `max_range_age_s` | `0.25` | ToF/LiDAR 讀值與影像時間的最大差距 |
| `min_range_m` | `0.05` | 太近或無效讀值不得下降 |
| `max_range_m` | `1.50` | 第一版末端對接上限 |
| `max_range_disagreement_m` | `0.08` | AprilTag range 與 ToF 的最大容許差 |
| `max_observation_latency_ms` | `300` | 超過只允許 log，不作有效定位 |
| `median_window_size` | `3` | 低延遲 median window |
| `ema_alpha` | `0.45` | 平滑係數 |
| `max_offset_jump_m` | `0.12` | 單次 offset 跳變上限 |
| `max_range_jump_m` | `0.20` | 單次 range 跳變上限 |
| `max_yaw_jump_deg` | `20` | 單次 yaw 跳變上限 |

## 測試順序

1. 先跑純 Python regression：

```bash
python3 -m unittest discover -s tests
```

2. 在 Jetson 或筆電上跑 AprilTag 相機 log，確認 pose 是 calibrated `OK`，不是 `UNCALIBRATED_ESTIMATE`。
3. 接 ToF/LiDAR，但只跑 supervisor log，不接 MAVLink。
4. 用 CSV 檢查 `vision_status`、`range_valid`、`state`、`vx_mps`、`vy_mps`、`yaw_rate_dps`、`vz_mps`。
5. 完成桌上方向測試與感測真值表後，才進入低空繫留 advisory。
6. 低空繫留通過後，才可規劃 guarded MAVLink setpoint；仍不得讓 Jetson 自動 arm。

## Abort 條件

以下情況都必須維持 abort/hold 行為：

- target lost 或 frame stale。
- range invalid、range stale、range mismatch。
- roll/pitch 超過安全值。
- 電壓過低或電流過高。
- 飛手介入。
- 接觸時水平/yaw 偏差超過允許值。
- contact/seated 在鎖定流程中消失。
- `lock_switch_closed` 與 `hall_locked` 不一致。
