#!/usr/bin/env bash
# sowbot_usb_diag.sh — RealSense L515 + sowbot 采集异常一键排查/（可选）修复
# 用法：
#   ./sowbot_usb_diag.sh        仅排查并生成报告
#   ./sowbot_usb_diag.sh --fix  排查并执行温和修复（reload uvcvideo/设置电源/重启服务）

set -u

FIX=0
if [[ "${1:-}" == "--fix" ]]; then
  FIX=1
fi

TS="$(date +%Y%m%d-%H%M%S)"
OUTDIR="${HOME}/usb_diag/diag-${TS}"
mkdir -p "${OUTDIR}"

green(){ printf "\033[32m%s\033[0m\n" "$*"; }
yellow(){ printf "\033[33m%s\033[0m\n" "$*"; }
red(){ printf "\033[31m%s\033[0m\n" "$*"; }

green "==> 开始排查（输出目录：${OUTDIR})"

# 0) 基本系统信息
{
  echo "=== uname ==="; uname -a
  echo; echo "=== OS / Kernel ==="; cat /etc/os-release 2>/dev/null || true; uname -r
  echo; echo "=== Time ==="; date -R
} > "${OUTDIR}/system.txt"

# 1) USB & 设备绑定状态
{
  echo "=== lsusb ==="; lsusb
  echo; echo "=== lsusb -t ==="; lsusb -t
  echo; echo "=== dmesg | USB（本次启动） ==="; journalctl -k -b | grep -i usb || true
  echo; echo "=== 最近5分钟内核USB事件 ==="; journalctl -k --since "-5 min" | grep -i usb || true
} > "${OUTDIR}/usb.txt"

# 识别 L515 设备路径与端口（8086:0b64）
L515_BUSID=""
for d in /sys/bus/usb/devices/*; do
  if [[ -f "$d/idVendor" && -f "$d/idProduct" ]]; then
    vid="$(cat "$d/idVendor" 2>/dev/null || true)"
    pid="$(cat "$d/idProduct" 2>/dev/null || true)"
    if [[ "$vid" == "8086" && "$pid" == "0b64" ]]; then
      L515_BUSID="$(basename "$d")"   # 例如 2-1
      break
    fi
  fi
done

# 2)
