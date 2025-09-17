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
OUTDIR="/home/pi/Desktop/ROBOTSOFTWARE/arduino/check/diag-${TS}"
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

# 2) RealSense 枚举 / 流配置（如果有 librealsense 工具）
{
  echo "=== rs-enumerate-devices (若无则忽略) ==="
  if command -v rs-enumerate-devices >/dev/null 2>&1; then
    rs-enumerate-devices
  else
    echo "(rs-enumerate-devices 未安装)"
  fi
} > "${OUTDIR}/realsense.txt"

# 3) 电源/性能相关
{
  echo "=== vcgencmd get_throttled (若无则忽略) ==="
  if command -v vcgencmd >/dev/null 2>&1; then
    vcgencmd get_throttled
  else
    echo "(vcgencmd 未安装)"
  fi
  echo; echo "=== free -h ==="; free -h
  echo; echo "=== df -h ==="; df -h
  echo; echo "=== df -i ==="; df -i
} > "${OUTDIR}/power_mem_disk.txt"

# 4) sowbot 服务与日志
{
  echo "=== systemctl status sowbot.service ==="
  systemctl status sowbot.service --no-pager || true
  echo; echo "=== sowbot.service 最近300行日志 ==="
  journalctl -u sowbot.service -n 300 --no-pager || true
} > "${OUTDIR}/sowbot_service.txt"

# 5) uvcvideo / video 设备占用情况
{
  echo "=== uvcvideo 模块 ==="
  lsmod | grep -E '^uvcvideo|^videobuf|^videodev' || true
  echo; echo "=== /dev/video* ==="; ls -l /dev/video* 2>/dev/null || true
  echo; echo "=== 哪些进程打开了 /dev/video* (lsof，若无则忽略) ==="
  if command -v lsof >/dev/null 2>&1; then
    lsof /dev/video* 2>/dev/null || true
  else
    echo "(lsof 未安装)"
  fi
} > "${OUTDIR}/video_stack.txt"

# 6) L515 的 power/control 与 autosuspend
{
  if [[ -n "${L515_BUSID}" ]]; then
    echo "=== L515 sysfs: ${L515_BUSID} ==="
    DEV="/sys/bus/usb/devices/${L515_BUSID}"
    for f in power/control power/autosuspend usb_speed bConfigurationValue; do
      if [[ -f "${DEV}/${f}" ]]; then
        printf "%s: " "${f}"; cat "${DEV}/${f}"
      fi
    done
  else
    echo "未在 /sys/bus/usb/devices/* 中找到 8086:0b64"
  fi
} > "${OUTDIR}/l515_power.txt"

# 7) 快速判定 & 提示
BIND_OK=$(grep -E 'Driver=uvcvideo' "${OUTDIR}/usb.txt" | wc -l | awk '{print $1}')
if [[ "$BIND_OK" -gt 0 ]]; then
  green "✓ 检测到 L515 接口由 uvcvideo 绑定（正常）"
else
  red "✗ 未检测到 uvcvideo 绑定（可能处于 usbfs / 断连后未正确接管）"
fi

if grep -qi 'usb .*disconnect' "${OUTDIR}/usb.txt"; then
  yellow "！日志中发现 USB disconnect 记录，建议检查线材/供电/震动干扰"
fi

if grep -q 'get_throttled' "${OUTDIR}/power_mem_disk.txt" && \
   grep -Eq '0x[0-9a-fA-F]*[25]0000' "${OUTDIR}/power_mem_disk.txt"; then
  yellow "！vcgencmd 显示曾发生欠压/限频，建议更换更好的电源/使用有源USB3 Hub"
fi

green "==> 排查完成。报告文件在：${OUTDIR}"

# 8) 可选修复
if [[ "$FIX" -eq 1 ]]; then
  yellow "==> 执行温和修复：reload uvcvideo / 设置 power.control=on / 重启 sowbot.service"
  # reload uvcvideo
  if lsmod | grep -q '^uvcvideo'; then
    sudo modprobe -r uvcvideo && sleep 1
  fi
  sudo modprobe uvcvideo

  # 设置 L515 的 power/control=on
  if [[ -n "${L515_BUSID}" ]]; then
    DEV="/sys/bus/usb/devices/${L515_BUSID}"
    if [[ -w "${DEV}/power/control" ]]; then
      echo on | sudo tee "${DEV}/power/control" >/dev/null
      green "已将 ${L515_BUSID} 的 power/control 设为 on"
    fi
  fi

  # 重启 sowbot.service
  sudo systemctl restart sowbot.service
  sleep 1
  systemctl status sowbot.service --no-pager | sed -n '1,20p' > "${OUTDIR}/sowbot_service_after_fix.txt"

  green "==> 修复动作完成。新的状态写入：${OUTDIR}/sowbot_service_after_fix.txt"
fi

# 9) 最后给出简短汇总
echo
green "【快速查看要点】"
echo " 1) 设备绑定：grep 'Driver=' ${OUTDIR}/usb.txt"
echo " 2) 断连历史：grep -i 'disconnect' ${OUTDIR}/usb.txt"
echo " 3) 相机情况：查看 ${OUTDIR}/realsense.txt"
echo " 4) 供电/欠压：查看 ${OUTDIR}/power_mem_disk.txt"
echo " 5) 服务异常：查看 ${OUTDIR}/sowbot_service.txt"
