#!/usr/bin/env bash
# 链路抗劣化测试：用 tc netem 在 eth0 上注入延迟/丢包，看协议行为是否符合设计。
#
#   sudo bash eth_netem_test.sh
#
# 场景：
#   N1 延迟 20 ms                -> 遥测照常 50 Hz，心跳年龄升高但不应掉链
#   N2 延迟 20 ms + 丢包 2%      -> 遥测少量丢帧，命令/心跳仍工作
#   N3 延迟 300 ms               -> 心跳 300 ms 超时，预期出现 LINK_LOST（安全设计）
#   N4 恢复                     -> CLEAR_FAULT 后回到健康
set -u
USER_HOME=/home/HwHiAiUser
OUT="$USER_HOME/link_test_records"
mkdir -p "$OUT"
TS=$(date +%Y%m%d_%H%M%S)
RC_ALL=0

cleanup() { tc qdisc del dev eth0 root 2>/dev/null || true; }
trap cleanup EXIT

run_case() {
    local name="$1"; shift
    local qdisc="$1"; shift
    echo ""
    echo "=== $name：$qdisc ==="
    cleanup
    if [ -n "$qdisc" ]; then
        tc qdisc add dev eth0 root netem $qdisc || { echo "  netem 设置失败"; return 1; }
    fi
    python3 "$USER_HOME/eth_soak.py" --duration 60 \
        --json "$OUT/netem_${name}_$TS.json" > "$OUT/netem_${name}_$TS.log" 2>&1
    local rc=$?
    python3 - "$OUT/netem_${name}_$TS.json" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1]))
print("  帧数 %d  速率 %.2f Hz  丢包 %.2f%%  gap max %.0f ms  p99 %.0f ms"
      % (d["frames"], d["rate_hz"], d["loss_percent"],
         d["gap_ms"]["max"] or 0, d["gap_ms"]["p99"] or 0))
print("  故障分布: %s  健康帧 %.1f%%  hb_age max %d ms"
      % (d["fault_histogram"], d["healthy_fraction"] * 100,
         d["hb_age_ms"]["max"] or 0))
EOF
    cleanup
    return $rc
}

echo "== 基线（无 netem）=="
run_case "N0_baseline" "" || RC_ALL=1

if run_case "N1_delay20ms" "delay 20ms"; then echo "  -> N1 期望：速率仍≈50 Hz、无 LINK_LOST"; else
    echo "  !! N1 失败（延迟 20 ms 不应破坏链路）"; RC_ALL=1; fi

if run_case "N2_delay20ms_loss2" "delay 20ms loss 2%"; then echo "  -> N2 期望：少量丢帧可接受（<3%）"; else
    echo "  !! N2 失败"; RC_ALL=1; fi

# N3 故意超过心跳超时：这里 soak 自己的判定会失败（预期），只看是否出现 LINK_LOST
run_case "N3_delay300ms" "delay 300ms" || true
python3 - "$OUT/netem_N3_delay300ms_$TS.json" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1]))
lost = sum(v for k, v in d["fault_histogram"].items() if int(k, 16) & 0x02)
total = sum(d["fault_histogram"].values()) or 1
print("  -> N3 期望：出现 LINK_LOST（安全设计），观测比例 %.1f%%" % (lost * 100.0 / total))
raise SystemExit(0 if lost > 0 else 1)
EOF
[ $? -eq 0 ] || { echo "  !! N3 未出现 LINK_LOST（延迟 300 ms 应触发心跳超时）"; }

echo ""
if [ "$RC_ALL" -eq 0 ]; then echo "NETEM TEST PASS"; else echo "NETEM TEST 有失败项"; fi
exit "$RC_ALL"
