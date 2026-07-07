# 驗收測試矩陣

## 桌上機構

| 測試 | 條件 | 通過標準 |
| --- | --- | --- |
| 導正容錯 | 偏移 0 cm、±5 cm、±10 cm | 可導入、不卡死、不刮傷 |
| yaw 容錯 | 0 度、±5 度、±10 度 | 可導入或明確失敗，不半卡住 |
| 緩衝 | 模擬下降接觸 | 無硬碰硬撞擊 |
| 鎖扣 | 連續開關 30 次 | 不卡死、不鬆脫 |
| 解鎖 | 連續解鎖 30 次 | 可釋放、不帶動模組翻倒 |

## 感測

| 測試 | 條件 | 通過標準 |
| --- | --- | --- |
| 未接觸 | 全部 false | 不鎖定 |
| 接觸未定位 | contact=true, seated=false | 不鎖定 |
| 定位未鎖定 | contact=true, seated=true, lock false | 允許進入鎖定，不允許起吊 |
| 雙感測鎖定 | lock_switch=true, hall_locked=true | 允許起吊測試 |
| 單感測故障 A | lock_switch=true, hall_locked=false | abort |
| 單感測故障 B | lock_switch=false, hall_locked=true | abort |

## 軟體狀態機

| 測試 | 條件 | 通過標準 |
| --- | --- | --- |
| 成功流程 | 目標可見、偏移收斂、接觸、seated、雙鎖定 | 進入 `COMPLETE` |
| 目標遺失 | 對準/下降期間超過 target timeout | 進入 `ABORT` |
| 下降逾時 | 未接觸且超過 descent timeout | 進入 `ABORT` |
| 姿態過斜 | roll/pitch 超過安全值 | 進入 `ABORT` |
| 飛手介入 | pilot_override=true | 進入 `ABORT` |
| 電源異常 | 電壓過低或電流過高 | 進入 `ABORT` |

## 低空繫留

| 測試 | 條件 | 通過標準 |
| --- | --- | --- |
| 無模組 hover | 20-50 cm | 姿態穩定、標記可見 |
| 假負載 hover | <=300 g | 無明顯震動或 brownout |
| 自動對準 | 不下降 | 位置修正方向正確 |
| 慢速下降 | 軟墊與繫留 | 接觸後停止下降 |
| 完整鎖定 | 假負載 | 10 次至少 8 次完成，無危險事件 |
