# 2026-10-29 涓婃澘杩炵画閬块殰杩愯鏃ュ織鍒嗘瀽锛堢敤鎴峰疄娴嬶級

- 鍥轰欢锛氫富宸ョ▼璇婃柇鏋勫缓 SHA256 `F9B573A648AFEDA94A59EED067BA84CA3182837355B9E6E47A0C0E237F7E85CD`
  锛堝甫 `route_reason`/`tof_reused` 璇婃柇瀛楁涓?PAUSED 闈㈡澘琛岋級
- 涓插彛锛欳H9102锛堟姄鍙栨湡闂寸敱 COM14 閲嶆柊鏋氫妇涓?**COM23**锛夛紝115200 8N1锛屽彧璇?- 鍘熷鎶撳彇锛歚board_run_demo.log`锛堣剼鏈洜绔彛琚叾浠栫▼搴忔姠鍗犳湭钀界洏锛夛紝
  瀹屾暣灏炬 `board_run_demo_full.log`锛?6:18:16鈥?6:20:40锛?357 鏉?`D` 琛岋級
- 16:16:56鈥?6:18:16 娈典粎瀛樹簬浼氳瘽杈撳嚭锛屽叧閿В鐮佽鎽樺綍瑙佷笅琛?
## debug_state 瑙ｇ爜锛坰ystem_motion.v锛?
```
[63:56] start_diag  [55:48] route_reason  [47:40] menu_command  [39:32] ack
[31:24] aux_type    [23:16] aux_code     [15:13] tof_reused    [12:8] route_state
[7] voice [6] ready [5] tof_all_valid [4] fault [3] busy [2] wait [1] auto [0] avoid
```

route_state锛? IDLE / 1 CRUISE_START / 2 CRUISE / 3 BRAKE_SELECT / 4 TURN_OUT /
5 WAIT_OUT / 6 SHIFT / 7 TURN_PARALLEL / 8 WAIT_PARALLEL / 9 PASS / 10 BRAKE_EDGE /
11 EXTRA / 12 TURN_BACK / 13 WAIT_BACK / 14 RETURN / 15 TURN_HOME / 16 WAIT_HOME /
17 BLOCKED / 20 WAIT_HOME 鍚庣殑宸¤埅鎭㈠ barrier / 21 BRAKE_SHIFT / 22 SHIFT_EXTRA / 23 SAMPLE_SHIFT

## 涓夋杩愯缁撴灉

| # | K2 鍚姩 | 缁撴灉 | 鍏抽敭璇佹嵁 |
|---|---|---|---|
| 1 | 16:17:10.45 | **PAUSED锛坮eason=0x85锛?* | 16:17:18.4 妫€娴嬮殰纰嶁啋BRAKE_SELECT锛?6:17:22.31 TURN_OUT 瀹屾垚杩涘叆 **WAIT_PARALLEL(8)**锛涙鍚?`tof_reused` 鐨?bit2锛圧IGHT 閫氶亾锛夋寔缁疆浣嶏紙0x88/0xA8/0xC8/0xE8锛夛紝RIGHT 涓€鐩村仠鍦ㄧ簿纭?20 mm 鐨勫鐢ㄧ姸鎬侊紱4 娆?1000 ms barrier 閲嶈瘯鍏ㄩ儴瓒呮椂 鈫?16:17:26.311 `D 0085160006019163`锛歴tate=17(BLOCKED)銆?*reason=0x85**锛涢殢鍚?`D 0000160006018061`锛歛uto_run=0 鈫?闈㈡澘 PAUSED |
| 2 | 16:17:59.85 | **瀹屾暣缁曢殰鎴愬姛** | 16:18:00.845 CRUISE 鈫?16:18:07.73 闅滅 鈫?16:18:11.686 WAIT_PARALLEL锛?84 ms 閫氳繃锛夆啋 23 SAMPLE_SHIFT 鈫?6 SHIFT 鈫?21 BRAKE_SHIFT 鈫?22 SHIFT_EXTRA 鈫?7 TURN_PARALLEL 鈫?13 WAIT_BACK 鈫?9 PASS 鈫?10 BRAKE_EDGE 鈫?11 EXTRA 鈫?12 TURN_BACK 鈫?16 WAIT_HOME 鈫?14 RETURN 鈫?15 TURN_HOME 鈫?20 鈫?1 CRUISE_START 鈫?**16:18:47.20 鎭㈠ CRUISE**锛屽叏绋嬬害 47 s |
| 3 | 16:19:19.3 | **瀹屾暣缁曢殰鎴愬姛** | 16:19:19.51 CRUISE 鈫?19:26.60 闅滅 鈫?19:30.589 WAIT_PARALLEL锛?5 ms 閫氳繃锛夆啋 23 鈫?6 鈫?21 鈫?22 鈫?7 鈫?13 鈫?9 鈫?10 鈫?11 鈫?12 鈫?16 鈫?14 鈫?20锛?6:20:07.52锛夆啋 1/2锛?*16:20:07.67 鎭㈠ CRUISE**锛岀害 48 s |

杩愯 2 鏈熼棿鏈?`S M 00 / S N 00 / S I 00` 浜嬩欢琛岋紙鍔ㄤ綔寮€濮?ASR 鐘舵€侊級锛屾棤 `E `锛堟晠闅滐級琛屻€?
## 缁撹

1. 鏃х増鈥滆浆 90掳 鍚?FAULT銆佸繀椤?K0 澶嶄綅鈥濈殑鏁呴殰宸叉秷闄わ細鏈 1 娆″け璐ヤ篃鍙槸
   `BLOCKED`/PAUSED锛坄auto_run` 鎺?0锛孠2 鍙仮澶嶏級锛? 娆¤繍琛?0 娆℃晠闅滈攣瀛樸€?2. 鍓╀綑闂瀹氫綅鏄庣‘锛?*杞悜鍚?barrier锛圵AIT_PARALLEL锛夌瓑涓嶅埌璇ヤ晶閫氶亾鐨勨€滄柊鏈夋晥鏍锋湰鈥?*銆?   RIGHT 閫氶亾杩炵画杩斿洖绮剧‘ 20 mm锛坢in-range 鎴柇/娴嬭窛澶辨晥锛夛紝鍥轰欢鎸夌瓥鐣ヤ涪寮冨苟澶嶇敤鏃у€硷紝
   鍥犳 `post_turn` 姘歌繙涓嶇疆浣嶏紱4脳1000 ms 閲嶈瘯鍚庤繘鍏?PAUSED銆備笁娆¤繍琛屼腑 1 娆″懡涓?   锛堟湰娆＄害 1/3锛屽睘闂存瓏鎬э級銆?3. 澶辫触涓庢垚鍔熸椂 WAIT_PARALLEL 鍋滅暀鏃堕棿宸窛鏋佸ぇ锛?.0 s 澶辫触 vs 0.085鈥?.184 s 鎴愬姛锛夛紝
   璇存槑淇鐐瑰簲鏀惧湪璇?barrier 瀵光€?0 mm 鍗′綇閫氶亾鈥濈殑澶勭悊绛栫暐涓婏紝鑰屼笉鏄彁楂橀噸璇曟鏁般€?
## 淇鏂瑰悜锛堟湰杞疄鏂斤級

- 浼犳劅鍣ㄥ眰锛氳繛缁?2 娆＄簿纭?20 mm 瑙嗕负鈥滄湰娆℃祴璺濆凡瀹屾垚銆佺洰鏍囦笉鍙祴/杩囪繎鈥濓紝
  鍙綔涓衡€滄柊椴滄祴璺濃€濅緵 barrier 浣跨敤锛堜絾缁濅笉鎶婂畠褰撲綔鍙敤鐨勫嚑浣曡窛绂伙級锛?  鍙栨秷鈥滆繛缁?4 娆″氨 XSHUT 閲嶅垵濮嬪寲鈥濈殑鏃犳晥鍔ㄤ綔锛堟湰娆℃棩蹇楄瘉鏄?4 s 鍐呭娆￠噸鍒濆鍖栧悗
  閫氶亾浠嶆寔缁?20 mm锛夈€?- 璺嚎灞傦細杞悜鍚庤嫢闅滅渚ч€氶亾鎸佺画 20 mm锛岀珛鍗宠繘鍏ュ浐瀹氶噷绋嬪厹搴曠粫琛?  锛堟í绉?+ 棰濆妯Щ + 鍥哄畾鐩磋閫氳繃 + 鍥炵嚎锛夛紝涓嶅啀鍋滃湪 WAIT_PARALLEL銆?- 浠跨湡锛氭柊澧炩€滆浆鍚戝悗渚у悜閫氶亾鎸佺画 20 mm鈥濆満鏅紝瑕佹眰璺嚎浠嶈兘鎭㈠鍒?CRUISE锛?  閲嶅缓鍚庨噸鏂颁笂鏉胯繛缁窇 3鈥? 娆＄‘璁ゃ€?

## 用户补充反馈与最终处理（2026-10-29 16:24–16:45）

- 用户反馈：“刚刚 IMU 地线掉了跑得很稳，现在插回去又不行了”。
  这与“地环路影响 VL53L0X 量程下限截断”的判断一致。本工程机械接线把 IMU 地接在
  P11-3 / LCD_GPIO0 / **T11**，而 PIN_ASSIGNMENT.md 早已注明 T11 是 FPGA 信号脚、
  “P11-2 才是专用 GND”。建议把 IMU 地改接 P11-2 或 P6-12/P6-30 专用 GND，
  三路 ToF 与 IMU 星形一点接地，不经 T11；改线后再跑 3–5 次确认。
- 用户随后反馈：“现在改了之后基本上都能跑完了，如果能让他更稳定就好了”。
  据此本轮完成第二轮固件修复（20 mm 截断容错 + 固定里程兜底，详见
  TECHNICAL_REPORT.md §3.6/§4.5 与 
ecords/fault_injection/REPORT.md §9）：
  连续 2 次精确 20 mm 视为“测距完成但不可用”，作为 barrier 的新鲜事件放行；障碍侧
  持续 20 mm 时路线直接改走固定里程兜底，不再 4 s 后 PAUSED；删除上板证明无效的
  XSHUT 重初始化。
- 新位流：impl/pnr/Embodied_RPACS.fs，SHA256
  B69DD9A04C3BCB6F06ED60B8311C14892A8FE185E804A341D0828F4423EA1B2C
  （Logic 13521/59%、CLS 8318/73%、Fmax 59.226 MHz、0 违例）。
  仿真：11 个故障注入场景 0 故障 0 PAUSED（场景 10 断言进入兜底）、7 套主工程仿真 PASS。
  待用户接上 Gowin USB 线与调试串口后烧写 SRAM，连续跑 3–5 次避障复测。
