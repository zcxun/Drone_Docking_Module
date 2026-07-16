# AprilTag 手機相機定位實驗規格

## 目的與範圍

本實驗目的是用手機作為臨時相機，在筆電上以 Python/OpenCV 辨識 AprilTag，先驗證定位輸出是否足以支援後續 docking state machine。此階段只做桌上與固定架測試，不接無人機、不接 Pixhawk、不輸出 MAVLink、不控制馬達或鎖扣。

本規格屬於階段 D「視覺與飛控整合」的前置小型實驗。成功後，才進入 companion computer 視覺程式、adapter、重播測試與日後低空繫留驗證。

## 已確認設定

| 項目 | 第一版設定 |
| --- | --- |
| 手機相機 | iPhone 13 |
| 使用鏡頭 | 後鏡頭，固定不切換鏡頭 |
| 筆電端 | MacBook |
| 影像來源 | iPhone 13 透過 USB/Continuity Camera 連線到 MacBook |
| 輸入模式 | 即時畫面與影片 replay 都需支援 |
| 相機擺放 | 手機固定於架上，下視拍攝 AprilTag |
| AprilTag family | `tag36h11` |
| AprilTag id | `0` |
| Tag 實體尺寸 | 黑色外框邊長實測 `100 mm` |
| 相機校正資料 | 目前沒有；第一版需可在無校正資料下 log 粗略估計，但不得預設視為可驅動狀態機的可靠 pose |
| 定位模式 | 有校正檔時使用相機校正參數與 tag 實體尺寸做 pose 估計；無校正檔時只做 uncalibrated estimate |
| 驗收距離 | 正常室內光線下 `30-80 cm` |
| 輸出對接目標 | 可直接映射到 `software/companion/docking_state_machine.py` 的 `SensorSnapshot` |

## 角色分工

專案經理 agent：

- 維護本規格與檔案位置。
- 確認本實驗維持 log-only，不開始飛控或狀態機實機整合。
- 在使用者確認本規格後，才允許進入程式開發。

控制/視覺工程師：

- 設計 AprilTag 偵測、pose 估計、座標轉換與有效性判斷流程。
- 定義相機校正需求、tag 尺寸參數、輸出欄位、低信心/失敗處理。
- 確保輸出可映射為 `SensorSnapshot`，但不直接控制無人機。

軟韌體 QA 工程師：

- 設計正常、邊界、負面與長時間穩定性測試。
- 驗證 false positive、錯 tag id、座標方向、stale pose、range invalid、延遲等風險。
- 保護現有 docking state machine 的 abort 與安全行為不被視覺整合繞過。

## AprilTag 定位流程

第一版 pipeline：

1. 從 USB/Continuity Camera 讀取影像 frame。
2. 轉灰階，執行 AprilTag 偵測。
3. 只接受 `tag36h11` 且 `id=0` 的偵測結果。
4. 若指定 tag 不存在，輸出 `target_visible=false`，所有 pose 欄位為空或 invalid。
5. 若偵測到指定 tag，讀取四角點、中心點與偵測品質資訊。
6. 若有相機內參與畸變參數，使用校正資料與 tag 邊長 `0.100 m` 估計 tag 相對相機 pose。
7. 若沒有相機校正資料，允許用假設 FOV 與 tag 像素尺寸輸出粗略估計，但狀態必須標示為 `UNCALIBRATED_ESTIMATE`，且預設不得映射為可驅動狀態機的有效 pose。
8. 轉換為 docking state machine 使用的水平偏移、yaw error 與 range。
9. 對每個 frame 輸出 timestamp、可見性、offset、yaw、range、validity 與品質狀態。
10. 寫入 log，供人工檢查與日後 replay，不直接送入飛控。

有效性規則：

- 只有指定 tag id 可見且 pose 求解成功時，才可輸出 `target_visible=true`。
- 若 range 無法可信估計，需輸出 `range_valid=false`，不得假裝可下降。
- 若 offset/yaw/range 出現 `NaN`、`inf`、空值或超出合理範圍，該 frame 必須標為 invalid。
- 若 tag 消失，不可沿用上一筆 pose 當作有效定位。
- 若偵測延遲過高或 frame timestamp 過舊，該 frame 應標為 stale，不可作為狀態機輸入。
- 無相機校正資料時，輸出可供畫面驗證與方向測試，但 adapter 預設必須拒絕將其視為可靠 `SensorSnapshot`。

## 座標與單位

第一版採固定下視相機座標，並定義 adapter 輸出時對齊未來機體 body frame：

| 欄位 | 單位 | 定義 |
| --- | --- | --- |
| `target_offset_x_m` | m | tag 中心相對相機光軸的水平 x 偏移，需在實作時用方向測試確認符號能讓狀態機修正方向正確 |
| `target_offset_y_m` | m | tag 中心相對相機光軸的水平 y 偏移，需在實作時用方向測試確認符號能讓狀態機修正方向正確 |
| `yaw_error_deg` | deg | docking 需要修正的 yaw 誤差，不是任意原始旋轉角 |
| `range_m` | m | 相機到 tag 平面的估計距離 |

座標方向不得只靠假設定案。第一版開發完成後，必須用「tag 往畫面左/右/上/下移動」的方向測試確認：

- offset 符號與 `DockingStateMachine._horizontal_velocity()` 的修正方向一致。
- yaw 符號與 `DockingStateMachine._yaw_rate()` 的修正方向一致。

## 輸出契約

視覺模組應先輸出一筆可 log 的資料結構，之後再由 adapter 映射到 `SensorSnapshot`。第一版欄位如下：

| 欄位 | 型別 | 必填 | 說明 |
| --- | --- | --- | --- |
| `timestamp_s` | float | yes | 影像 frame 或處理完成時間，需單調遞增 |
| `target_visible` | bool | yes | 指定 AprilTag 是否可見 |
| `tag_family` | string | yes | 第一版固定 `tag36h11` |
| `tag_id` | int/null | yes | 第一版只接受 `0` |
| `target_offset_x_m` | float/null | yes | 可映射到 `SensorSnapshot.target_offset_x_m` |
| `target_offset_y_m` | float/null | yes | 可映射到 `SensorSnapshot.target_offset_y_m` |
| `yaw_error_deg` | float/null | yes | 可映射到 `SensorSnapshot.yaw_error_deg` |
| `range_m` | float/null | yes | 可映射到 `SensorSnapshot.range_m` |
| `range_valid` | bool | yes | 可映射到 `SensorSnapshot.range_valid` |
| `pose_valid` | bool | yes | pose 是否可信 |
| `confidence` | float/null | no | 偵測品質或 margin，依使用的 AprilTag API 定義 |
| `latency_ms` | float/null | no | frame 擷取到輸出完成的估計延遲 |
| `status` | string | yes | `OK`、`TAG_NOT_FOUND`、`WRONG_TAG_ID`、`POSE_INVALID`、`RANGE_INVALID`、`STALE_FRAME` 等 |

映射到 `SensorSnapshot` 時：

- `time_s = timestamp_s`
- `target_visible = target_visible and pose_valid`
- `target_offset_x_m = target_offset_x_m`
- `target_offset_y_m = target_offset_y_m`
- `yaw_error_deg = yaw_error_deg`
- `range_m = range_m`
- `range_valid = range_valid and pose_valid`

視覺模組不得填寫 `contact`、`seated`、`lock_switch_closed`、`hall_locked`、`battery_voltage_v`、`current_a`、`pilot_override`。這些欄位仍由日後感測節點、飛控或人工測試資料提供。

## 驗收標準

在正常室內光線、固定下視手機、tag36h11 id=0、tag 邊長 100 mm、距離 30-80 cm 下：

- 指定 tag 偵測成功率需達 `>= 95%`。
- 無 tag 場景 false positive 必須為 `0`。
- 錯誤 tag id 不得被當作 docking target。
- 靜態水平 offset 最大誤差需 `<= 3 cm`。
- 靜態 yaw 最大誤差需 `<= 5 deg`。
- `range_m` 需隨 30/50/80 cm 距離變化呈單調合理趨勢。
- tag 消失後，需在小於 `target_loss_timeout_s=1.0` 的時間內停止回報有效目標。
- p95 處理延遲目標 `<= 150 ms`；若 `> 300 ms`，只能維持 log-only，不得進入飛控整合。
- pose invalid、range invalid、stale frame 不得被映射成可下降的有效 `SensorSnapshot`。

## QA 測試案例

| ID | 情境 | 條件 | 預期結果 |
| --- | --- | --- | --- |
| AT-01 | 基準辨識 | 30/50/80 cm，tag 正對相機 | `target_visible=true`，range valid，offset 接近 0 |
| AT-02 | 水平偏移 | x/y 偏移 0、±3、±5、±8、±10 cm | 偏移方向正確，誤差可量測 |
| AT-03 | yaw 偏差 | 0、±5、±10、±15 deg | yaw 符號與大小趨勢正確 |
| AT-04 | 距離變化 | 30、50、80 cm | range 單調合理，不跳變 |
| AT-05 | 光照變化 | 正常、偏暗、背光、陰影 | 正常光通過，低品質影像記錄 lost rate |
| AT-06 | 遮擋 | 遮擋 10%、25%、50% | 嚴重遮擋不可輸出可信 pose |
| AT-07 | 無 tag | 空白桌面、印刷圖案、反光表面 | `target_visible=false`，false positive 為 0 |
| AT-08 | 錯 tag id | 放入非 id=0 的 AprilTag | 不得觸發 docking target |
| AT-09 | 多 tag 干擾 | id=0 與其他 tag 同時出現 | 只追蹤 id=0 |
| AT-10 | 座標方向 | tag 往畫面左/右/上/下移動 | offset 符號與狀態機修正方向一致 |
| AT-11 | target lost | 對準中移開 tag 超過 1 秒 | 不沿用舊 pose，未來可導向 `TARGET_LOST` |
| AT-12 | range invalid | tag 可見但 pose/range 不可信 | `range_valid=false`，不得作為下降候選 |
| AT-13 | 長時間穩定 | 固定 50 cm 跑 3-5 分鐘 | 無明顯漂移，lost rate 可接受 |
| AT-14 | snapshot dry-run | 將輸出映射為 `SensorSnapshot` log | 可供狀態機 replay，但不控制硬體 |

## 第一版執行方式

掃描 MacBook 上可用的 OpenCV camera index：

```bash
python3 -m software.companion.vision.apriltag_phone_pose --scan-cameras
```

使用 iPhone 13 Continuity Camera 即時偵測，並寫出 CSV log：

```bash
python3 -m software.companion.vision.apriltag_phone_pose \
  --camera-index 0 \
  --output experiments/apriltag_phone_camera/live_log.csv
```

使用影片檔 replay：

```bash
python3 -m software.companion.vision.apriltag_phone_pose \
  --video path/to/apriltag_test_video.mov \
  --output experiments/apriltag_phone_camera/video_log.csv
```

目前沒有相機校正資料時，程式會輸出 `UNCALIBRATED_ESTIMATE`，用於方向測試與可見性驗證；adapter 預設不會把它當成可直接驅動 docking state machine 的有效 pose。

視覺化目前誤差與演算法建議調整方向：

```text
software/companion/vision/apriltag_alignment_dashboard.html
```

此頁面可手動輸入 offset/yaw/range，也可載入 `apriltag_phone_pose.py` 產生的 CSV log。建議修正值採用目前狀態機演算法：`vx = clamp(-0.6 * x, +/-0.20)`、`vy = clamp(-0.6 * y, +/-0.20)`、`yaw_rate = clamp(-0.5 * yaw, +/-15)`。

與 iPhone 13 Continuity Camera 即時串接時，啟動本地 live server：

```bash
python3 -m software.companion.vision.apriltag_live_server \
  --camera-index 0 \
  --port 8765
```

然後用瀏覽器開啟：

```text
http://127.0.0.1:8765/dashboard
```

`127.0.0.1` 只適用於 Mac 自己開 dashboard。若要用手機瀏覽器連到 Mac，live server 需綁定 `0.0.0.0`，並在手機開啟 live server 印出的 `Phone/LAN URL`，例如 `http://192.168.x.x:8765/dashboard`。手機與 Mac 必須在同一個 Wi-Fi，且 macOS 防火牆需允許連入。

若 `--camera-index 0` 不是 iPhone 13，可先用 `apriltag_phone_pose.py --scan-cameras` 找出正確 index。live server 會同時把觀測資料寫到 `experiments/apriltag_phone_camera/live_dashboard_log.csv`。

注意：若同一支 iPhone 13 正在當 Continuity Camera，建議 dashboard 先在 Mac 上開啟；用同一支 iPhone 同時當相機又開 dashboard，可能會讓 Continuity Camera 中斷或切換狀態。

若掃描相機時出現 `OpenCV: camera access has been denied`，需先到 macOS 的 Privacy & Security > Camera 允許目前執行 Python/Codex 的應用程式存取相機；必要時重開 Codex 或重新執行掃描。權限未開時 live dashboard 仍可開啟，但會顯示 `LIVE ERROR`，不會收到即時觀測資料。

若 Camera 權限清單中沒有 Python 或 Codex，請改從 Terminal 啟動 live server，讓 macOS 將相機權限授權給 Terminal：

```bash
software/companion/vision/run_apriltag_live_server.command
```

第一次執行時 macOS 應跳出相機權限提示；允許 Terminal 後，Mac 開啟 `http://127.0.0.1:8765/dashboard`，手機則開啟 Terminal 顯示的 `Phone/LAN URL`。

## 手機校正與 A4 四 Tag 測試

產生列印用 PDF：

```bash
python3 -m software.companion.vision.generate_a4_calibration_board
```

輸出檔：

```text
output/pdf/a4_calibration_and_apriltag_board.pdf
```

PDF 第 1 頁是 A4 棋盤格校正板，第 2 頁是四角 AprilTag 定位板。列印時必須使用 `Actual Size / 100%`，不要縮放。定位板紙面只印四個 AprilTag；中心紅點、綠線、每條線距離與手機到紙張高度都只顯示在手機網頁 overlay。

啟動手機校正與定位 HTTPS server：

```bash
software/companion/vision/run_phone_experiment_server.command
```

Terminal 會印出兩個 URL：

```text
https://<mac-ip>:9443/calibration
https://<mac-ip>:9443/phone
```

先用 iPhone Safari 開 `/calibration`，允許後鏡頭，依畫面提示拍攝棋盤格至少 15 張有效樣本，再按 Solve 產生：

```text
software/companion/vision/calibration/iphone13_rear_checkerboard.json
```

校正完成後開 `/phone`。手機會持續將相機 frame 傳到 Mac Python，Mac 回傳四 tag 偵測、A4 中心、手機到紙張垂直高度與修正提示，手機畫面會即時畫出紅點、綠線與距離標示。

若 iPhone Safari 拒絕相機，通常是 HTTPS 憑證尚未被信任。第一版 server 會自動產生本地 self-signed certificate，iPhone 需要接受或信任該憑證後才能使用網頁相機。

## 檔案位置規劃

目前只新增本規格文件。後續若使用者確認開發，建議檔案位置如下：

| 類型 | 位置 |
| --- | --- |
| 本規格 | `docs/08_apriltag_phone_camera_experiment.md` |
| A4 PDF generator | `software/companion/vision/generate_a4_calibration_board.py` |
| 手機校正/定位 server | `software/companion/vision/phone_experiment_server.py` |
| 手機校正頁 | `software/companion/vision/phone_calibration.html` |
| 手機定位頁 | `software/companion/vision/phone_tracking.html` |
| 視覺程式 | `software/companion/vision/apriltag_phone_pose.py` |
| Live server | `software/companion/vision/apriltag_live_server.py` |
| 視覺化頁面 | `software/companion/vision/apriltag_alignment_dashboard.html` |
| 狀態機 adapter | `software/companion/vision/vision_to_sensor_snapshot.py` |
| 校正資料 | `software/companion/vision/calibration/` |
| 契約與 adapter 測試 | `tests/test_apriltag_output_contract.py` |
| 實驗紀錄模板 | `templates/apriltag_phone_experiment_log.csv` |
| 實際測試證據 | `experiments/apriltag_phone_camera/` |

## 風險與限制

- Continuity Camera 與 USB camera 行為可能受作業系統、手機型號、解析度、曝光與對焦模式影響。
- 未完成相機校正前，range 與公尺級 offset 不可視為可靠量測。
- 手機自動曝光、自動對焦、滾動快門與串流延遲可能造成 pose 抖動或 stale frame。
- AprilTag 紙張翹曲、列印比例錯誤、tag 邊長量測錯誤會直接影響距離估計。
- 座標與 yaw 符號是整合高風險點，必須由方向測試鎖定。
- 本實驗只驗證視覺定位，不代表對接流程安全；接觸、seated、雙鎖定感測與飛手 override 仍需獨立驗證。

## 開發前確認門檻

開始寫程式前需確認：

- 使用者已接受本規格。
- 可取得固定手機的下視支架。
- 已列印並量測 tag36h11 id=0，黑框邊長為 100 mm。
- 已確認第一版使用 iPhone 13 後鏡頭與 MacBook，並完成 USB/Continuity Camera 實際影像輸入測試。
- 已知目前沒有相機校正資料；因此第一版即時與影片模式均需先以 log-only/uncalibrated estimate 驗證，不得直接放行飛控整合。
- 後續若要讓輸出成為正式 docking pose，需再準備相機校正流程或校正板。

未達上述門檻前，不進入飛控整合、不修改 `docking_state_machine.py` 的行為。
