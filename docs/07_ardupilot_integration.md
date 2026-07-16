# ArduPilot / Pixhawk 6C 整合備忘

## 整合原則

- Pixhawk 6C 保持飛行穩定與 failsafe 核心。
- 伴隨電腦只負責視覺、狀態機與高層 setpoint。
- 第一版不要讓程式自動 arm。
- 每次測試前確認飛手可用遙控器 override。
- 所有伺服與伴隨電腦供電都走獨立 BEC，不吃 Pixhawk peripheral port 電流。

## Pixhawk 6C 可用介面

Pixhawk 6C 適合這個 demo，因為它提供：

- 多組 serial/TELEM，可接伴隨電腦與遙測。
- I2C/CAN，可接測距與其他外部感測。
- MAIN/AUX PWM，可接伺服鎖扣或 gripper 類機構。
- 支援 ArduPilot 4.2.3 以上。

## ArduPilot 功能對應

| 需求 | ArduPilot 對應功能 | 第一版策略 |
| --- | --- | --- |
| 精準降落/定位 | Precision Landing / LANDING_TARGET | 先 log-only，再進入低空 setpoint 測試 |
| 鎖扣伺服 | Servo Gripper / DO_GRIPPER | 可用同概念，但先由狀態機輸出抽象命令 |
| 測距 | Rangefinder | 先實測有效範圍再填參數 |
| 飛手介入 | RC override / mode switch | 必須能立即中止自動流程 |

## 第一版參數注意事項

不要直接匯入未驗證參數。每台 F450 的 ESC、馬達、槳、電池、震動與安裝方向不同，參數必須由實機校正後填入。

整合時至少檢查：

- frame type 與 motor order。
- accelerometer、compass、RC、ESC calibration。
- battery monitor 與 failsafe。
- rangefinder orientation、min/max range。
- companion TELEM baud rate。
- gripper/servo output channel 與 PWM range。
- Precision Landing 相關設定只在 bench 或繫留測試通過後啟用。

## MAVLink Bridge 里程碑

1. `log-only`：只讀飛控姿態/高度，並輸出狀態機 log。
2. `advisory`：狀態機產生建議命令，但不送入飛控。
3. `guarded setpoint`：只有在地面端與飛手同意時，送低速水平/yaw/垂直 setpoint。
4. `lock command`：只有 seated 後才送鎖扣命令。
5. `lift test`：只有雙感測鎖定後，才允許 10-30 cm 起吊測試。

每個里程碑都要能獨立 abort。

## Jetson Bench Motor Test

Jetson 與 Pixhawk 的第一個 powered hardware 練習只允許做無槳 bench motor test。實作與流程放在 `docs/10_jetson_pixhawk_mavlink_motor_test.md`。

限制：

- 只用 `MAV_CMD_DO_MOTOR_TEST`。
- 不自動 arm、不 force arm、不切換飛行模式。
- 每次只測一顆馬達，低油門、短時間。
- 必須先完成只讀 heartbeat/telemetry monitor。
