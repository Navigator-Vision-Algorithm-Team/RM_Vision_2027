#!/usr/bin/env bash
# =============================================================================
#  达妙 USB-CAN 运行环境配置脚本（Ubuntu 22.04，x86_64 / aarch64 通用）
#
#  解决本机两个运行期问题：
#    1) libdm_device.so 依赖 libusb >= 1.0.27（libusb_init_context 符号）；
#       Ubuntu 22.04 自带 1.0.25，直接运行报：
#         symbol lookup error: .../libdm_device.so: undefined symbol: libusb_init_context
#    2) 非 root 用户访问达妙 USB-CAN 需要 udev 规则（否则 open device 失败）
#
#  用法：
#    ./io/can/setup_damiao_env.sh                   # 检查 + 安装 libusb + 写 udev 规则
#    ./io/can/setup_damiao_env.sh --check           # 只检查，不改动系统
#    VID=1234 PID=5678 ./io/can/setup_damiao_env.sh # 自动识别不到设备时手动指定
#
#  新版 libusb 只装到 /usr/local（不覆盖系统库），/usr/local/lib 在 ld.so 搜索
#  顺序中优先于 /lib/x86_64-linux-gnu；配置完成后无需重新编译程序。
# =============================================================================
set -euo pipefail

LIBUSB_VERSION="1.0.27"
LIBUSB_SYMBOL="libusb_init_context"  # 1.0.27 起提供，旧版缺失
CHECK_ONLY=false
[[ "${1:-}" == "--check" ]] && CHECK_ONLY=true

log() { echo "[env] $*"; }
die() {
  echo "[env] 错误: $*" >&2
  exit 1
}

# 运行时实际生效的 libusb 是否提供 libusb_init_context
check_libusb() {
  command -v ldconfig >/dev/null || die "未找到 ldconfig，本脚本仅支持 Linux"
  local so
  so=$(ldconfig -p 2>/dev/null | awk '/libusb-1\.0\.so\.0/ {print $NF; exit}')
  [[ -n "${so}" ]] || {
    log "未找到 libusb-1.0.so.0"
    return 1
  }
  if command -v nm >/dev/null && nm -D "${so}" 2>/dev/null | grep -q "${LIBUSB_SYMBOL}"; then
    log "libusb OK: ${so} 提供 ${LIBUSB_SYMBOL}"
    return 0
  fi
  log "libusb 过旧: ${so} 缺少 ${LIBUSB_SYMBOL}（需要 >= ${LIBUSB_VERSION}）"
  return 1
}

# 源码编译安装 libusb 到 /usr/local（GitHub 不通时自动改用 gitee 镜像）
install_libusb() {
  log "安装编译依赖"
  sudo apt-get update
  sudo apt-get install -y build-essential autoconf automake libtool pkg-config \
    libudev-dev wget curl git

  local work
  work=$(mktemp -d)
  trap 'rm -rf "${work}"' EXIT
  cd "${work}"

  local url="https://github.com/libusb/libusb/releases/download/v${LIBUSB_VERSION}/libusb-${LIBUSB_VERSION}.tar.bz2"
  log "下载 libusb ${LIBUSB_VERSION}"
  if ! wget -q "${url}" 2>/dev/null && ! curl -fsSL -o "libusb-${LIBUSB_VERSION}.tar.bz2" "${url}" 2>/dev/null; then
    log "GitHub 不可达，改用 gitee 镜像"
    git clone --depth 1 --branch "v${LIBUSB_VERSION}" https://gitee.com/mirrors/libusb.git \
      "libusb-${LIBUSB_VERSION}"
    cd "libusb-${LIBUSB_VERSION}"
    ./autogen.sh --prefix=/usr/local --disable-static
  else
    tar xf "libusb-${LIBUSB_VERSION}.tar.bz2"
    cd "libusb-${LIBUSB_VERSION}"
    ./configure --prefix=/usr/local --disable-static
  fi

  log "编译并安装到 /usr/local"
  make -j"$(nproc)"
  sudo make install
  sudo ldconfig
}

# 为达妙 USB-CAN 写 udev 规则（当前用户免 sudo 访问）
setup_udev() {
  local vid="${VID:-}" pid="${PID:-}"
  if [[ -z "${vid}" || -z "${pid}" ]]; then
    local line
    line=$(lsusb 2>/dev/null | grep -iE 'damiao|dm-?usb|dm ?can' | head -1 || true)
    if [[ -n "${line}" ]]; then
      vid=$(sed -n 's/.*ID \([0-9a-fA-F]\{4\}\):[0-9a-fA-F]\{4\}.*/\1/p' <<<"${line}")
      pid=$(sed -n 's/.*ID [0-9a-fA-F]\{4\}:\([0-9a-fA-F]\{4\}\).*/\1/p' <<<"${line}")
    fi
  fi
  if [[ -z "${vid}" || -z "${pid}" ]]; then
    log "未识别到达妙 USB-CAN。插好设备后运行 lsusb 找到其 VID:PID，再执行："
    log "  VID=xxxx PID=xxxx $0"
    return 0
  fi

  local rule="/etc/udev/rules.d/99-damiao-usb-can.rules"
  log "写入 udev 规则 ${vid}:${pid} -> ${rule}"
  echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"${vid}\", ATTRS{idProduct}==\"${pid}\", MODE=\"0666\"" |
    sudo tee "${rule}" >/dev/null
  sudo udevadm control --reload
  sudo udevadm trigger
  log "请拔插一次 USB-CAN 使规则生效"
}

# 用 ld.so 检查 SDK 的 libdm_device.so 依赖符号是否全部可解析
verify_sdk() {
  local so
  for so in "${DMCAN_SDK_ROOT:-$HOME/dm-device-sdk}"/C\&C++/lib/*/linux/*/libdm_device.so \
    "$HOME"/dm-device-sdk*/C\&C++/lib/*/linux/*/libdm_device.so; do
    [[ -f "${so}" ]] || continue
    log "检查 ${so}"
    local missing
    missing=$(ldd -r "${so}" 2>&1 | grep "undefined symbol" | head -5 || true)
    if [[ -n "${missing}" ]]; then
      echo "${missing}"
    else
      log "  依赖符号全部解析成功"
    fi
  done
}

log "系统: $(. /etc/os-release 2>/dev/null && echo "${PRETTY_NAME}" || echo 未知)"

if ! check_libusb; then
  if ${CHECK_ONLY}; then
    die "libusb 过旧；去掉 --check 重新运行本脚本自动安装"
  fi
  install_libusb
  check_libusb || die "libusb 安装后仍缺少 ${LIBUSB_SYMBOL}"
fi

verify_sdk

if ${CHECK_ONLY}; then
  log "检查结束（--check 模式未改动系统）"
  exit 0
fi

setup_udev
log "完成。无需重新编译，直接运行："
log "  ./build/tests/can_com_debug"
