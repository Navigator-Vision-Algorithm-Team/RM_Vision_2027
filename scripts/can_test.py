#!/usr/bin/env python3
"""CAN 下位机响应测试脚本。
发送云台控制帧到 can0，验证下位机是否响应。

帧格式（与 cboard.cpp send() 一致）：
  Byte 0: ctrl  (0=不控, 1=控制)
  Byte 1: shoot (0=不开火, 1=开火)
  Byte 2-3: yaw   * 10000  (int16, big-endian, rad)
  Byte 4-5: pitch * 10000  (int16, big-endian, rad)
  Byte 6-7: 0 (horizon_distance)

用法:
  python3 can_test.py 10 -5        # yaw=10°, pitch=-5°
  python3 can_test.py 0 0 0        # 停止控制 (ctrl=0)
  python3 can_test.py --interactive # 交互模式
"""

import argparse
import math
import struct
import sys
import time

CAN_ID = 0xFF
CAN_IFACE = "can0"


def deg_to_raw(deg: float) -> int:
    """角度(度) → CAN 原始值 (rad * 10000)"""
    rad = math.radians(deg)
    return int(rad * 10000)


def send_can(ctrl: bool, shoot: bool, yaw_deg: float, pitch_deg: float):
    """发送一帧 CAN 指令"""
    try:
        import can
    except ImportError:
        print("python-can 未安装，尝试用 socketcan 原始方式...")
        _send_raw(ctrl, shoot, yaw_deg, pitch_deg)
        return

    yaw_raw = deg_to_raw(yaw_deg)
    pitch_raw = deg_to_raw(pitch_deg)

    # 限制在 int16 范围
    yaw_raw = max(-32768, min(32767, yaw_raw))
    pitch_raw = max(-32768, min(32767, pitch_raw))

    data = struct.pack(">BBhhH",
                      1 if ctrl else 0,
                      1 if shoot else 0,
                      yaw_raw,
                      pitch_raw,
                      0)  # horizon_distance

    msg = can.Message(arbitration_id=CAN_ID, data=data, is_extended_id=False)

    try:
        bus = can.interface.Bus(channel=CAN_IFACE, bustype="socketcan")
        bus.send(msg)
        print(f"已发送: ctrl={ctrl} shoot={shoot} yaw={yaw_deg:.1f}°(raw={yaw_raw}) "
              f"pitch={pitch_deg:.1f}°(raw={pitch_raw}) "
              f"data={data.hex()}")
        bus.shutdown()
    except OSError as e:
        print(f"CAN 发送失败: {e}")
        print("请确认 can0 已启动: sudo ip link set can0 up type can bitrate 1000000")


def _send_raw(ctrl: bool, shoot: bool, yaw_deg: float, pitch_deg: float):
    """原始 socket 方式发送 CAN 帧（python-can 不可用时的回退）"""
    import ctypes
    import socket

    # struct can_frame
    # https://github.com/torvalds/linux/blob/master/include/uapi/linux/can.h
    class CanFrame(ctypes.Structure):
        _fields_ = [
            ("can_id", ctypes.c_uint32),
            ("can_dlc", ctypes.c_uint8),
            ("__pad", ctypes.c_uint8),
            ("__res0", ctypes.c_uint8),
            ("__res1", ctypes.c_uint8),
            ("data", ctypes.c_uint8 * 8),
        ]

    yaw_raw = deg_to_raw(yaw_deg)
    pitch_raw = deg_to_raw(pitch_deg)
    yaw_raw = max(-32768, min(32767, yaw_raw))
    pitch_raw = max(-32768, min(32767, pitch_raw))

    frame = CanFrame()
    frame.can_id = CAN_ID
    frame.can_dlc = 8
    frame.data[0] = 1 if ctrl else 0
    frame.data[1] = 1 if shoot else 0
    # big-endian int16
    frame.data[2] = (yaw_raw >> 8) & 0xFF
    frame.data[3] = yaw_raw & 0xFF
    frame.data[4] = (pitch_raw >> 8) & 0xFF
    frame.data[5] = pitch_raw & 0xFF
    frame.data[6] = 0
    frame.data[7] = 0

    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((CAN_IFACE,))
        sock.send(ctypes.string_at(ctypes.addressof(frame), ctypes.sizeof(frame)))
        print(f"已发送: ctrl={ctrl} shoot={shoot} yaw={yaw_deg:.1f}°(raw={yaw_raw}) "
              f"pitch={pitch_deg:.1f}°(raw={pitch_raw})")
    except OSError as e:
        print(f"CAN 发送失败: {e}")
        print("请确认 can0 已启动: sudo ip link set can0 up type can bitrate 1000000")
    finally:
        sock.close()


def interactive():
    """交互模式"""
    print(f"CAN 云台控制测试 — ID=0x{CAN_ID:02X}, 接口={CAN_IFACE}")
    print("输入格式: <yaw_deg> <pitch_deg>  (如: 10 -5)")
    print("         stop / 0      → 停止控制")
    print("         fire <y> <p>   → 开火 + 控制")
    print("         q / quit      → 退出")
    print()

    ctrl = True
    shoot = False
    last_yaw, last_pitch = 0.0, 0.0

    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not cmd:
            continue
        parts = cmd.split()

        if parts[0] in ("q", "quit", "exit"):
            break
        elif parts[0] == "stop" or (parts[0] == "0" and len(parts) == 1):
            send_can(False, False, 0, 0)
            ctrl = False
        elif parts[0] == "fire" and len(parts) >= 3:
            yaw = float(parts[1])
            pitch = float(parts[2])
            send_can(True, True, yaw, pitch)
            last_yaw, last_pitch = yaw, pitch
            shoot = True
        elif len(parts) >= 2:
            try:
                yaw = float(parts[0])
                pitch = float(parts[1])
                send_can(ctrl, shoot, yaw, pitch)
                last_yaw, last_pitch = yaw, pitch
                ctrl = True
            except ValueError:
                print(f"无效输入: {cmd}")
        else:
            print("用法: <yaw> <pitch> 或 fire <yaw> <pitch> 或 stop")


def main():
    parser = argparse.ArgumentParser(description="CAN 云台控制测试")
    parser.add_argument("yaw", type=float, nargs="?", help="yaw 角度 (度)")
    parser.add_argument("pitch", type=float, nargs="?", help="pitch 角度 (度)")
    parser.add_argument("shoot", type=int, nargs="?", default=0, help="开火 (0/1), 默认 0")
    parser.add_argument("--ctrl", type=int, default=1, help="控制 (0/1), 默认 1")
    parser.add_argument("-i", "--interactive", action="store_true", help="交互模式")
    parser.add_argument("--id", type=str, default="0xFF", help="CAN ID (十六进制), 默认 0xFF")
    parser.add_argument("--iface", type=str, default="can0", help="CAN 接口, 默认 can0")
    args = parser.parse_args()

    global CAN_ID, CAN_IFACE
    CAN_ID = int(args.id, 16)
    CAN_IFACE = args.iface

    if args.interactive:
        interactive()
    elif args.yaw is not None and args.pitch is not None:
        send_can(bool(args.ctrl), bool(args.shoot), args.yaw, args.pitch)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
