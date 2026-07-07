# 系統架構

## 子系統分工

| 子系統 | 責任 | 第一版實作 |
| --- | --- | --- |
| Pixhawk 6C / ArduPilot | 穩定飛行、模式切換、遙控器 failsafe、伺服/PWM 輸出 | 保持飛行控制核心，不放客製視覺邏輯 |
| 伴隨電腦 | 影像辨識、目標偏移估計、對接狀態機、MAVLink 指令 | Raspberry Pi 5 或 Jetson Orin Nano |
| 連接器機構 | 導正、緩衝、鎖定、釋放 | 3D 列印導正件 + 伺服插銷或機械卡扣 |
| 感測節點 | 接觸、定位、鎖定訊號彙整 | ESP32/Arduino 或直接 GPIO |
| 地面端 | 參數設定、遙測、測試紀錄、人工中止 | Mission Planner / MAVProxy / QGroundControl 擇一 |

## 狀態機責任

伴隨電腦狀態機是第一版的決策核心，但不得繞過飛手與飛控 failsafe。狀態機只在 `auto_enabled=true` 時啟動，任何安全條件不成立就輸出 abort。

主要狀態：

- `MANUAL_APPROACH`：等待飛手手動接近與自動流程開啟。
- `TARGET_SEARCH`：尋找 AprilTag/ArUco 或等效固定點。
- `HORIZONTAL_ALIGN`：把目標中心移到允許範圍內。
- `YAW_ALIGN`：角度對準。
- `DESCEND`：慢速下降。
- `CONTACT_DETECTED`：接觸成立後停止下降。
- `MECHANICAL_GUIDE`：等待機構導正與定位訊號。
- `LOCKING`：驅動伺服鎖扣。
- `LOCK_VERIFY`：用雙感測確認鎖定。
- `LIFT_TEST`：只做 10-30 cm 低空起吊驗證。
- `COMPLETE`：demo 成功。
- `ABORT`：自動流程停止，由飛手或地面端處理。

## 感測介面

第一版輸入訊號：

- `target_visible`：是否看到定位標記。
- `target_offset_x_m`, `target_offset_y_m`：目標相對中心偏移。
- `yaw_error_deg`：對接角度偏差。
- `range_m`, `range_valid`：下視距離。
- `contact`：是否接觸模組/治具。
- `seated`：是否已由機構導正到位。
- `lock_switch_closed`：機械開關鎖定訊號。
- `hall_locked`：霍爾或磁簧鎖定訊號。
- `roll_deg`, `pitch_deg`：機體姿態保護。
- `battery_voltage_v`, `current_a`：電源保護。
- `pilot_override`：飛手介入。

第一版鎖定判定必須同時滿足 `lock_switch_closed=true` 與 `hall_locked=true`。

## 控制輸出

第一版軟體原型只輸出抽象命令：

- 飛行模式建議：`MANUAL`、`LOITER`、`GUIDED`、`ABORT`。
- 水平速度建議：用於後續 MAVLink velocity setpoint。
- yaw rate 建議：用於後續角度修正。
- 垂直速度建議：慢速下降或低空起吊測試。
- 鎖扣命令：`OPEN_LOCK`、`CLOSE_LOCK`、`HOLD_LOCK`、`RELEASE_LOCK`。

實機整合時，這些命令必須先經過 MAVLink bridge、模式限制、地面端 armed 狀態確認與飛手 override gate。
