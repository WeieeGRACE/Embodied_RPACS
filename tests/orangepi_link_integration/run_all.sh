#!/usr/bin/env bash
# 板上一条命令跑完整套联调：预检 → 链路采集 → 相机+链路并发 → 报告
#
#   ./run_all.sh                      # 默认 30s 链路 + 120s 并发
#   ./run_all.sh 300                  # 并发时长改成 300s
#   PORT=/dev/ttyS0 ./run_all.sh      # 用 Pi 的 40pin UART
#
# 前置：FPGA 已烧链路位流、串口线已接、用户已在 dialout 组。
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
export UART_BRIDGE_TOOLS="${UART_BRIDGE_TOOLS:-$HERE/uart_tools}"
# pyorbbecsdk 是编译产物，需要指向 install/lib
export PYTHONPATH="${PYTHONPATH:-}:$HOME/rpacs_phase0/pyorbbecsdk-main/install/lib"
PY="${PY:-$HOME/rpacs_phase0/pyorbbecsdk-main/.venv/bin/python}"
if [ ! -x "$PY" ]; then PY="$(command -v python3)"; fi
PORT="${PORT:-}"
SOAK_S="${1:-120}"
TS="$(date +%H%M)"
REC="$HERE/records"
mkdir -p "$REC"

echo "== 环境 =="
echo "python : $PY"
echo "tools  : $UART_BRIDGE_TOOLS"
echo "port   : ${PORT:-<config 默认>}"
ls -l /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null || echo "!! 没看到 USB 串口"
lsusb | grep -iE '1a86|0403|2bc5' || echo "!! 没看到 CH9102 / Orbbec"

fail=0
run_step() {
  name="$1"; shift
  echo ""
  echo "== $name =="
  if "$@"; then echo "-- $name OK"; else echo "-- $name FAILED"; fail=1; fi
}

PORTARG=""
[ -n "$PORT" ] && PORTARG="--port $PORT"

run_step "预检" "$PY" "$HERE/src/preflight.py" $PORTARG --json "$REC/preflight_$TS.json"
run_step "链路采集 30s" "$PY" "$HERE/src/link_client.py" $PORTARG --duration 30 \
    --csv "$REC/link_$TS.csv" --json "$REC/link_$TS.json"
run_step "相机+链路并发 ${SOAK_S}s" "$PY" "$HERE/src/camera_link_soak.py" $PORTARG \
    --duration "$SOAK_S" --json "$REC/soak_cam_$TS.json"
run_step "生成报告" "$PY" "$HERE/src/report.py" \
    "$REC/preflight_$TS.json" "$REC/link_$TS.json" "$REC/soak_cam_$TS.json" \
    --out "$REC/report_$TS.md" --title "深度相机 ↔ FPGA 链路联调（$(date '+%m-%d %H:%M')）"

echo ""
if [ "$fail" -eq 0 ]; then
  echo "==== ALL PASS ====  报告: $REC/report_$TS.md"
else
  echo "==== 有步骤失败，请看上面输出与 $REC/ ===="
fi
exit "$fail"
