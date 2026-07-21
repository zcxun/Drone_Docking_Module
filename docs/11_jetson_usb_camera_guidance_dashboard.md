# Jetson USB 相機定位導引 Dashboard

## 目的

本階段用 Jetson Nano + USB 黑白相機做 AprilTag 定位測試，並讓電腦瀏覽器看到即時畫面與直覺操作導引。

此階段只做 log-only / 人工導引：

- 不送 Pixhawk/PX4 控制命令。
- 不啟動馬達。
- 不進入自動飛行。

畫面會同時顯示未來可能送給 Pixhawk/PX4 的 command preview，方便之後比對自動控制邏輯。

## Jetson 環境檢查

進入專案與 venv：

```bash
cd ~/Drone_Docking_Module
source .venv/bin/activate
```

檢查 OpenCV 與 ArUco/AprilTag 支援：

```bash
python3 -c "import cv2; print(cv2.__version__); print(hasattr(cv2, 'aruco')); print(hasattr(cv2.aruco, 'ArucoDetector'))"
```

若沒有 `cv2`，優先使用 Jetson 系統套件：

```bash
sudo apt update
sudo apt install -y python3-opencv
```

若 venv 看不到系統 OpenCV，重建 venv 時允許使用 system site packages：

```bash
cd ~/Drone_Docking_Module
deactivate 2>/dev/null || true
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements-jetson.txt
```

## 掃描 USB 相機

```bash
python3 -m software.companion.vision.jetson_usb_guidance_server --scan-cameras
```

找到可開啟的 camera index，例如：

```text
camera_index=0 opened=true width=640 height=480 channels=1
```

黑白相機 `channels=1` 是正常的，程式會直接用灰階做 AprilTag 偵測。

## 啟動 Dashboard

預設使用 `tag36h11 id=0`，黑框邊長 `100 mm`：

```bash
python3 -m software.companion.vision.jetson_usb_guidance_server \
  --host 0.0.0.0 \
  --port 8765 \
  --camera-index 0 \
  --tag-id 0 \
  --tag-size-m 0.100 \
  --camera-forward up \
  --output experiments/jetson_usb_camera/guidance_log.csv
```

Terminal 會印出：

```text
Computer/LAN URL: http://<jetson-ip>:8765/dashboard
```

在電腦瀏覽器開啟該網址，即可看到：

- 即時 USB 相機畫面。
- AprilTag 中心與畫面中心。
- 人工導引：往前/後/左/右幾 cm。
- 高度：目前高度與建議上升/下降。
- 角度：順時針/逆時針轉幾度。
- 傾斜：roll/pitch 是否安全。
- Pixhawk/PX4 command preview。

## 方向校正

預設座標：

```text
畫面上方 = 機頭前方
畫面右方 = 機體右方
```

若相機安裝方向不同，用：

```bash
--camera-forward up|down|left|right
```

如果左右或上下顛倒，用：

```bash
--mirror-x
--mirror-y
```

方向測試方式：

1. 把 tag 放在畫面上方，導引應顯示「往前」。
2. 把 tag 放在畫面下方，導引應顯示「往後」。
3. 把 tag 放在畫面右方，導引應顯示「往右」。
4. 把 tag 放在畫面左方，導引應顯示「往左」。

若方向不對，先調整 `--camera-forward`，再用 mirror 參數微調。

## 高度與可信度

沒有相機校正檔時，畫面會顯示：

```text
可用於人工導引，尚不可自動控制
```

這是預期行為。未校正狀態可以用來練習人工對準，但不能作為自動控制依據。

若要得到可信的公尺級 offset/range，需要為 USB 相機建立 calibration JSON，然後啟動時加入：

```bash
--calibration-json path/to/usb_camera_calibration.json
```

## 可選 Pixhawk/PX4 姿態讀取

若要在同一頁顯示 Pixhawk/PX4 的 roll/pitch/battery，可加上只讀 MAVLink telemetry：

```bash
python3 -m software.companion.vision.jetson_usb_guidance_server \
  --host 0.0.0.0 \
  --port 8765 \
  --camera-index 0 \
  --mavlink-device /dev/ttyTHS1 \
  --mavlink-baud 57600
```

這只讀 telemetry，不會送控制命令。

## 驗收標準

- 電腦可以開 `http://<jetson-ip>:8765/dashboard`。
- 看得到 USB 黑白相機畫面。
- 無 tag 時顯示「找不到定位標記」。
- tag 出現時顯示前/後/左/右導引。
- 移動 tag 時導引方向正確。
- CSV log 寫入 `experiments/jetson_usb_camera/guidance_log.csv`。
- 不送 MAVLink 控制、不啟動馬達。

