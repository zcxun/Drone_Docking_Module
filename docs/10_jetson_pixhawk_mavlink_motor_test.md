# Jetson / Pixhawk MAVLink 通訊與無槳馬達測試

## 目的與安全邊界

本階段目標是學會 Jetson Nano 透過 TELEM UART 與 Pixhawk/ArduPilot 做 MAVLink 通訊，並在無槳、固定機架、低油門、短時間條件下做 bench motor test。

本工具只做：

- 讀取 Pixhawk `HEARTBEAT` 與基本 telemetry。
- 對 ArduPilot 發送 `MAV_CMD_DO_MOTOR_TEST`。
- 對 PX4 發送 `MAV_CMD_ACTUATOR_TEST`。
- 等待 `COMMAND_ACK`。

本工具不做：

- 不自動起飛。
- 不提供自由 throttle/RC override。
- 不切換飛行模式。
- 不 force arm。
- 不繞過 ArduPilot safety checks。

## 程式同步到 Jetson

Mac/Codex 端：

```bash
cd "/Users/zhuangchengxun/Desktop/CleanGO/無人機連接模組和定位系統"
python3 -m unittest discover -s tests
git status --short
git add README.md docs software templates tests requirements-jetson.txt
git commit -m "Add Jetson Pixhawk MAVLink bench tools"
git push origin main
```

Jetson 端第一次下載：

```bash
cd ~
git clone https://github.com/zcxun/Drone_Docking_Module.git
cd Drone_Docking_Module
python3 -m pip install --user -r requirements-jetson.txt
python3 -m unittest discover -s tests
```

Jetson 端之後更新：

```bash
cd ~/Drone_Docking_Module
git pull origin main
python3 -m pip install --user -r requirements-jetson.txt
python3 -m unittest discover -s tests
```

## TELEM UART 接線

第一版預設用 Jetson Nano UART 接 Pixhawk TELEM port：

- Jetson TX -> Pixhawk TELEM RX。
- Jetson RX -> Pixhawk TELEM TX。
- Jetson GND -> Pixhawk TELEM GND。
- 不用 Pixhawk TELEM 的 5V 腳供 Jetson。
- Jetson 與伺服/外設使用獨立 BEC 或可靠供電。

Jetson 預設 serial device：

```text
/dev/ttyTHS1
```

Pixhawk 對應 TELEM port 需設定成 MAVLink2，baud 要和 Jetson CLI 一致。第一版預設：

```text
921600 baud
```

若使用的是不同 TELEM port 或不同 Jetson image，請用實際裝置覆蓋 `--device` 與 `--baud`。

## 只讀通訊測試

先不要讓馬達動。先跑 60 秒只讀 monitor：

```bash
python3 -m software.companion.jetson.mavlink_monitor \
  --device /dev/ttyTHS1 \
  --baud 921600 \
  --duration-s 60
```

通過標準：

- 能看到 heartbeat。
- `copter=True`。
- 能看到 armed 狀態。
- 若有電池與姿態資料，會列印 battery/current/roll/pitch/yaw。

若沒有 heartbeat：

- 檢查 TX/RX 是否交叉。
- 檢查 GND 是否共地。
- 檢查 Pixhawk TELEM port 的 `SERIALx_PROTOCOL` 與 `SERIALx_BAUD`。
- 檢查 Jetson 使用者是否有 serial device 權限。

## 無槳馬達測試

測試前 checklist：

- 螺旋槳已全部拆下。
- 機架已固定，不會因馬達震動移動。
- 電池可快速拔除。
- 周圍沒有人靠近馬達。
- 已經用 QGroundControl 做過 motor test，知道馬達順序與轉向。
- 先跑過 `mavlink_monitor` 且 heartbeat 穩定。

dry-run，不接 Pixhawk、不轉馬達：

```bash
python3 -m software.companion.jetson.motor_test_cli \
  --dry-run \
  --motor 1
```

如果 Pixhawk 是 ArduPilot，使用 `motor_test_cli.py` 做實際單顆馬達測試：

```bash
python3 -m software.companion.jetson.motor_test_cli \
  --device /dev/ttyTHS1 \
  --baud 921600 \
  --motor 1 \
  --throttle-percent 10 \
  --duration-s 2 \
  --confirm PROPS_REMOVED
```

如果 Pixhawk 是 PX4，`MAV_CMD_DO_MOTOR_TEST` 可能會回 `MAV_RESULT_UNSUPPORTED`。這時改用 PX4 actuator test：

```bash
python3 -m software.companion.jetson.px4_actuator_test_cli \
  --dry-run \
  --device /dev/ttyTHS1 \
  --baud 57600 \
  --motor 1
```

PX4 實際單顆馬達測試：

```bash
python3 -m software.companion.jetson.px4_actuator_test_cli \
  --device /dev/ttyTHS1 \
  --baud 57600 \
  --motor 1 \
  --value 0.10 \
  --timeout-s 2 \
  --confirm PROPS_REMOVED
```

安全限制：

- `--motor` 預設只接受 `1-4`。
- `--throttle-percent` 只接受 `5-15`。
- `--duration-s` 只接受 `1-3`。
- 每次只測一顆馬達。
- 沒有 `--confirm PROPS_REMOVED` 不會送命令。
- 如果 Pixhawk 回報已 armed，工具會拒絕測試。
- 如果 `COMMAND_ACK` rejected 或 timeout，工具會停止並報錯。
- PX4 actuator test 的 `--value` 是 normalized output value，預設 `0.10`，硬性上限 `0.15`。

建議順序：

```bash
# motor 1
python3 -m software.companion.jetson.motor_test_cli --motor 1 --confirm PROPS_REMOVED

# motor 2
python3 -m software.companion.jetson.motor_test_cli --motor 2 --confirm PROPS_REMOVED

# motor 3
python3 -m software.companion.jetson.motor_test_cli --motor 3 --confirm PROPS_REMOVED

# motor 4
python3 -m software.companion.jetson.motor_test_cli --motor 4 --confirm PROPS_REMOVED
```

每顆馬達測完都記錄：

- 是否收到 accepted ACK。
- 實際轉動的是哪顆馬達。
- 轉向是否和 QGroundControl / ArduPilot motor order 一致。
- 有無異音、ESC 重啟、brownout、Pixhawk reboot。

## 參考

- ArduPilot Companion Computers: https://ardupilot.org/dev/docs/companion-computers.html
- ArduPilot MAVLink Interface: https://ardupilot.org/dev/docs/mavlink-commands.html
- MAVLink Command Protocol: https://mavlink.io/en/services/command.html
- ArduPilot motor test 拆槳要求: https://ardupilot.org/copter/docs/connect-escs-and-motors.html#checking-the-motor-numbering-with-the-mission-planner-motor-test
- MAVLink common.xml `MAV_CMD_DO_MOTOR_TEST`: https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml
- MAVLink `MAV_CMD_ACTUATOR_TEST`: https://mavlink.io/en/messages/common.html#MAV_CMD_ACTUATOR_TEST
