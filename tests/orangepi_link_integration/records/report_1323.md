# 深度相机 ↔ FPGA 链路联调（10-29 13:26）

- 生成时间：2026-10-29 13:26:18
- 数据来源：3 个 JSON

## preflight_1323.json

- 端口：`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5757024820-if00`

| 检查 | 结果 | 详情 |
| --- | --- | --- |
| P1_port_exists | ✅ | /dev/serial/by-id/usb-1a86_USB_Single_Serial_5757024820-if00 |
| P2_port_open | ✅ | uart:/dev/serial/by-id/usb-1a86_USB_Single_Serial_5757024820-if00@115200 |
| P3_telemetry_received | ✅ | n=151 / 3.0s（预热 0.5s 不计） |
| P4_rate_in_range | ✅ | 50.02 Hz (期望 45.0~60.0) |
| P5_no_parse_errors | ✅ | errors=0 |
| P6_heartbeat_fresh | ✅ | hb_age=100 ms, link_lost=False |
| P7_fault_byte_readable | ✅ | fault=0x20[CMD_TIMEOUT] |

**结论：PASS**

末帧快照：enc=[0, 0] yaw=5.59° pitch=-1.58° roll=-1.54° fault=0x20[CMD_TIMEOUT]


## link_1323.json

| 项 | 值 |
| --- | --- |
| 遥测帧数 | 1500 |
| 速率 | 平均 50.0 Hz（50.0~50.0）|
| 解析统计 | {'good': 1501, 'sync_err': 0, 'len_err': 0, 'crc_err': 0, 'tail_err': 0, 'timeout': 0, 'type_err': 0} |
| 故障位分布 | {'0x20': 1500} |

末帧：enc=[0, 0] yaw=5.89° pitch=-1.59° roll=-1.54° hb_age=92ms


## soak_cam_1323.json

| 项 | 值 |
| --- | --- |
| 链路遥测 | n=6001, 平均 50.0 Hz, 最低 49.0 Hz |
| 相机 | mode=camera, 963 帧, 平均 8.01 fps |
| CPU | 平均 30.8%, 峰值 32.1% |
| 解析统计 | {'good': 6002, 'sync_err': 0, 'len_err': 0, 'crc_err': 0, 'tail_err': 0, 'timeout': 0, 'type_err': 0} |
| 端口 | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5757024820-if00` @ 115200 |

| 判定项 | 结果 |
| --- | --- |
| link_ok | ✅ |
| link_rate_ok | ✅ |
| link_errors_ok | ✅ |
| link_no_loss | ✅ |
| link_hb_age_ok | ✅ |
| camera_opened | ✅ |
| camera_frames_ok | ✅ |
| camera_fps_ok | ✅ |
| cpu_ok | ✅ |

**结论：PASS**

