#!/usr/bin/env bash
# Pi 侧一键跑昨晚验证过的全部"不需要动车"的测试（可重复执行）。
#
#   bash ~/run_link_tests.sh            # 默认：验收 + 故障语义 + 60 s 长稳
#   bash ~/run_link_tests.sh 600        # 额外跑 10 分钟长稳
#
# 依赖：~/eth_link_test.py、~/eth_fault_test.py、~/eth_soak.py、~/uart_tools/
# 前置：sudo bash ~/pi_setup_eth.sh（IP + 静态 ARP）
set -u
SOAK_SECONDS="${1:-60}"
OUT="$HOME/link_test_records"
mkdir -p "$OUT"
TS=$(date +%Y%m%d_%H%M%S)
FAIL=0

echo "=== 1) 协议验收（12 项）==="
python3 "$HOME/eth_link_test.py" --json "$OUT/accept_$TS.json" 2>&1 | tail -16
[ "${PIPESTATUS[0]}" -eq 0 ] || { echo "!! 验收失败"; FAIL=1; }

echo ""
echo "=== 2) 安全/故障语义 ==="
python3 "$HOME/eth_fault_test.py" --json "$OUT/fault_$TS.json" 2>&1 | tail -20
[ "${PIPESTATUS[0]}" -eq 0 ] || { echo "!! 故障语义有失败项"; FAIL=1; }

echo ""
echo "=== 3) 长稳 ${SOAK_SECONDS}s ==="
python3 "$HOME/eth_soak.py" --duration "$SOAK_SECONDS" \
    --json "$OUT/soak_$TS.json" --csv "$OUT/soak_$TS.csv" 2>&1 | tail -6
[ "${PIPESTATUS[0]}" -eq 0 ] || { echo "!! 长稳失败"; FAIL=1; }

echo ""
if [ "$FAIL" -eq 0 ]; then
    echo "ALL LINK TESTS PASS  (records: $OUT)"
else
    echo "SOME LINK TESTS FAILED (records: $OUT)"
fi
exit "$FAIL"
