#!/usr/bin/env bash
# 链路 v2（以太网/UDP）板上环境准备 —— 幂等，可在每次开机后跑一次。
#
#   sudo bash pi_setup_eth.sh
#
# 作用：
#   1) 建/更新 NetworkManager 连接 ethfpga = 192.168.0.1/24（never-default，不动 WiFi 默认路由）
#   2) 拉起 eth0（FPGA 断电时链路无载波，会提示"等待对端上电"）
#   3) 写静态 ARP：192.168.0.2 -> 00:0a:35:01:fe:c0（FPGA 收包需要单播，且它没有 ARP 响应器）
#
# FPGA 侧：192.168.0.2:5000（命令），回包广播到 192.168.0.1:5001。
set -u
FPGA_IP=192.168.0.2
FPGA_MAC=00:0a:35:01:fe:c0
HOST_IP=192.168.0.1
IF=eth0

if [ "$(id -u)" -ne 0 ]; then
    echo "请用 sudo 运行: sudo bash $0" >&2
    exit 1
fi

echo "== 1) NetworkManager 连接 ethfpga =="
if nmcli connection show ethfpga >/dev/null 2>&1; then
    nmcli connection modify ethfpga \
        ipv4.method manual ipv4.addresses ${HOST_IP}/24 ipv4.never-default yes \
        ipv6.method disabled connection.autoconnect yes
else
    nmcli connection add type ethernet ifname ${IF} con-name ethfpga \
        ipv4.method manual ipv4.addresses ${HOST_IP}/24 ipv4.never-default yes \
        ipv6.method disabled connection.autoconnect yes
fi
nmcli connection up ethfpga 2>&1 | tail -1

echo "== 2) 链路状态 =="
carrier=$(cat /sys/class/net/${IF}/carrier 2>/dev/null || echo 0)
speed=$(cat /sys/class/net/${IF}/speed 2>/dev/null || echo -1)
if [ "$carrier" = "1" ]; then
    echo "   ${IF}: 载波正常, ${speed} Mb/s"
else
    echo "   ${IF}: 无载波（FPGA 板未上电/网线未接）—— 上电后重跑本脚本即可"
fi
ip -4 addr show ${IF} | grep -q "inet ${HOST_IP}" && echo "   IP ${HOST_IP}/24 已配置" \
    || echo "   !! IP 未配置，检查 nmcli"

echo "== 3) 静态 ARP =="
ip neigh replace ${FPGA_IP} lladdr ${FPGA_MAC} dev ${IF} nud permanent
ip neigh show dev ${IF} | grep ${FPGA_IP} || echo "   !! ARP 写入失败"

echo ""
echo "完成。验证："
echo "  pgrep -f eth_link_test.py >/dev/null || python3 ~/eth_link_test.py"
