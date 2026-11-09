#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
虚拟下位机（Python）：创建 pty 伪串口对，按真实 RM 协议与真实频率写入四元数等数据。

- 一端（slave, /dev/pts/N）给主程序用普通串口方式打开，主程序完全不知道对面不是真硬件；
- 另一端（master）由本脚本写入/读取，负责按协议打包。

支持：
  * 实时生成：云台姿态（0x0503）、底盘反馈（0x0502）、协议版本（0x0500）、
    机器人状态（0x0501）、以及下位机转发的裁判帧（0x0001 / 0x0201）
  * 文件回放：--replay 读取 C++ 侧记录的 serial.bin（只回放下位机→视觉方向）
  * 异常注入：丢包、延迟、跳变、校验错误、角度噪声、MCU 重启（uptime_ms 回落）

用法：
  python3 sim/virtual_mcu.py --port-file /tmp/vmcu.pts
  # 然后：在 yaml 中把 com_port 改为 /tmp/vmcu.pts 的内容，serial_mode 设为 real
  #       （或 serial_mode: fake + fake_serial_external: true）
  python3 sim/virtual_mcu.py --drop-rate 0.05 --spike-every 200 --corrupt-crc-rate 0.02
  python3 sim/virtual_mcu.py --replay logs/virtual_session/serial.bin
"""

import argparse
import math
import os
import random
import signal
import struct
import sys
import time
import tty

# ============================================================================
# RM 标准 CRC（与 io/serial/gimbal/protocol_crc.cpp 完全一致）
#   CRC8 : 反射多项式 0x8C（即标准 0x31 反射），初值 0xFF
#   CRC16: 反射多项式 0x8408（即标准 0x1021 反射），初值 0xFFFF
# ============================================================================


def _make_crc8_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if (crc & 1) else (crc >> 1)
        table.append(crc & 0xFF)
    return table


def _make_crc16_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if (crc & 1) else (crc >> 1)
        table.append(crc & 0xFFFF)
    # 与 io/serial/gimbal/protocol_crc.cpp 的 W_CRC_TABLE 严格对齐：固件实际使用的表
    # 在索引 47 处与标准生成表不同（0x0D974，非标准的 0x0D9F5）。
    # 必须保留该差异，否则主程序会拒绝我们生成的每一帧。
    table[47] = 0xD974
    return table


CRC8_TAB = _make_crc8_table()
CRC16_TAB = _make_crc16_table()


def crc8_init():
    return 0xFF


def crc8_update(crc, data):
    for b in data:
        crc = CRC8_TAB[(crc ^ b) & 0xFF]
    return crc


def crc16_update(crc, data):
    for b in data:
        crc = ((crc >> 8) ^ CRC16_TAB[(crc ^ b) & 0xFF]) & 0xFFFF
    return crc


# ============================================================================
# 协议打包（与下位机 host 协议 / 2027 赛季裁判协议逐字节对齐，小端序）
#   线格式：A5 | len(u16) | seq | crc8 | cmd_id(u16) | data | crc16
#   上行 host：0x0500~0x0504   裁判转发：0x0001 / 0x0201
# ============================================================================
HEADER_MAGIC = 0xA5


def build_header(cmd_id, data_length, seq):
    """5 字节帧头 + 2 字节 cmd_id：magic, data_length(u16), seq, crc8, cmd_id(u16)"""
    head = struct.pack("<BHB", HEADER_MAGIC, data_length, seq & 0xFF)
    crc = crc8_update(crc8_init(), head)
    return head + struct.pack("<BH", crc, cmd_id)


def build_packet(cmd_id, payload, seq=0):
    """帧头 + 载荷 + CRC16（小端）"""
    head = build_header(cmd_id, len(payload), seq)
    body = head + payload
    crc = crc16_update(0xFFFF, body)
    return body + struct.pack("<H", crc)


def wrap_pi(a):
    """INS 实际给 (-π, π]，这里按同样区间发，顺带验证上位机的归一化"""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def pack_host_version(uptime_ms, seq=0):
    # host_version_t: u8 host_version, u16 referee_season, u8 robot_type,
    #                 u8 chassis_type, u8 gimbal_type, u32 uptime_ms  (10)
    payload = struct.pack("<B H B B B I", 1, 2027, 7, 2, 2, uptime_ms & 0xFFFFFFFF)
    return build_packet(0x0500, payload, seq)


def pack_host_robot(seq=0):
    # host_robot_status_t: enable, controller_online, robot_mode,
    #                      fric_mode, shoot_mode, module_online  (6)
    payload = struct.pack("<B B B B B B", 0x0F, 1, 3, 2, 0, 0x3F)
    return build_packet(0x0501, payload, seq)


def pack_chassis(vx, vy, wz, heading=0.0, enabled=1, seq=0):
    # host_chassis_status_t: u8 enabled, float vx, vy, wz, feedback_angle  (17)
    payload = struct.pack("<Bffff", enabled & 0xFF, vx, vy, wz, heading)
    return build_packet(0x0502, payload, seq)


def pack_gimbal(pitch, yaw, roll, upper_joint, enable=1, autoaim=1, seq=0):
    # host_gimbal_status_t: u8 gimbal_enable, u8 autoaim_enabled,
    #                       float pitch, yaw, roll, upper_yaw_angle, fold_yaw_angle  (22)
    # yaw 是 IMU（相机/枪管）世界系角，upper_joint 是小yaw 相对大yaw 的关节角
    payload = struct.pack(
        "<BBfffff", enable & 0xFF, autoaim & 0xFF, pitch, yaw, roll, upper_joint, 0.0
    )
    return build_packet(0x0503, payload, seq)


def pack_game_status(game_type=1, game_progress=4, remain=300, seq=0):
    # game_status_t: u8 bitfield(game_type:4|game_progress:4), u16 remain, u64 sync  (11)
    game_info = ((game_progress & 0x0F) << 4) | (game_type & 0x0F)
    payload = struct.pack(
        "<BHQ", game_info, remain, int(time.monotonic() * 1e9) & 0xFFFFFFFFFFFFFFFF
    )
    return build_packet(0x0001, payload, seq)


def pack_robot_status(robot_id=7, seq=0):
    # robot_status_t (17)：u8 id, u8 level, u16 hp, u16 max_hp, u16 cooling, u16 heat_limit,
    #                     u16 power_limit, float bullet_speed_limit, u8 power_management 位域
    payload = struct.pack(
        "<BBHHHHHfB", robot_id & 0xFF, 3, 400, 400, 40, 120, 40, 30.0, 0x07
    )
    return build_packet(0x0201, payload, seq)


# ============================================================================
# 虚拟下位机
# ============================================================================
class VirtualMcu:
    def __init__(self, args):
        self.args = args
        self.rnd = random.Random(args.seed)
        self.master_fd = None
        self.slave_name = None
        self._stop = False
        self.t0 = time.monotonic()
        self.stats = {"emitted": 0, "dropped": 0, "spiked": 0, "corrupt": 0, "delayed": 0}

    # ---------------------------------------------------------------- pty
    def open_pty(self):
        import pty

        master_fd, slave_fd = pty.openpty()
        self.slave_name = os.ttyname(slave_fd)
        tty.setraw(slave_fd)          # 关闭行规程（等价于串口 raw 模式）
        os.set_blocking(master_fd, False)
        os.close(slave_fd)            # 只保留 master，slave 由主程序按路径打开
        self.master_fd = master_fd

        print(f"[VirtualMCU] pty 伪串口已创建: slave={self.slave_name}")
        print(f"[VirtualMCU] 将 yaml 中 com_port 设为 '{self.slave_name}'")
        print("[VirtualMCU] 并设 serial_mode: real，或 serial_mode: fake + fake_serial_external: true")

        if self.args.port_file:
            with open(self.args.port_file, "w") as f:
                f.write(self.slave_name + "\n")
            print(f"[VirtualMCU] slave 路径已写入 {self.args.port_file}")

        if self.args.link:
            try:
                if os.path.islink(self.args.link) or os.path.exists(self.args.link):
                    os.unlink(self.args.link)
                os.symlink(self.slave_name, self.args.link)
                print(f"[VirtualMCU] 软链接 {self.args.link} -> {self.slave_name}")
            except OSError as e:
                print(f"[VirtualMCU] 创建软链接失败: {e}", file=sys.stderr)

    # ---------------------------------------------------------------- 输出
    def write(self, data):
        try:
            os.write(self.master_fd, data)
            return True
        except BlockingIOError:
            # 没有订阅者或缓冲区满：丢弃（真实下位机也会丢弃）
            return False
        except OSError:
            return False

    def drain_input(self):
        """读取主程序下发的数据（0x0402 等），用于日志/校验"""
        try:
            data = os.read(self.master_fd, 4096)
        except (BlockingIOError, OSError):
            return b""
        if data and self.args.verbose and len(data) >= 7 and data[0] == HEADER_MAGIC:
            cmd_id = struct.unpack_from("<H", data, 5)[0]
            if cmd_id == 0x0402 and len(data) >= 22:
                pitch, yaw = struct.unpack_from("<ff", data, 9)
                print(f"[VirtualMCU] 收到 0x0402: yaw={yaw:.3f} pitch={pitch:.3f}")
        return data

    # ---------------------------------------------------------------- 生成
    def emit_gimbal(self, idx):
        a = self.args
        t = time.monotonic() - self.t0

        # 双yaw：大yaw 慢速摆动，小yaw 相对大yaw 有自己的小幅快摆（关节角）
        lower = math.pi * math.sin(2 * math.pi * 0.1 * t)
        # 加相位偏移，让关节角启动时就不为 0，避免"joint 恒 0"掩盖接线错误
        joint = 0.35 * math.sin(2 * math.pi * 0.45 * t + 0.6)
        pitch = 0.15 * math.sin(2 * math.pi * 0.5 * t)
        roll = 0.05 * math.sin(2 * math.pi * 0.3 * t)

        injected = False
        if a.noise_rad > 0:
            lower += self.rnd.gauss(0.0, a.noise_rad)
            pitch += self.rnd.gauss(0.0, a.noise_rad)
        if a.spike_every > 0 and idx > 0 and idx % a.spike_every == 0:
            lower += a.spike_rad
            injected = True
            self.stats["spiked"] += 1

        # 噪声/跳变只加在大yaw 上，关节角保持干净：上位机应能精确还原 lower/upper
        # 0x0503 只发「IMU(相机) 世界角 + 小yaw 关节角」，两轴由上位机自己复合
        yaw = wrap_pi(lower + joint)

        if a.drop_rate > 0 and self.rnd.random() < a.drop_rate:
            self.stats["dropped"] += 1
            return None

        if a.delay_ms > 0:
            time.sleep(a.delay_ms / 1000.0)
            self.stats["delayed"] += 1

        pkt = bytearray(pack_gimbal(pitch, yaw, roll, joint, seq=idx & 0xFF))

        if a.corrupt_crc_rate > 0 and self.rnd.random() < a.corrupt_crc_rate:
            pkt[-1] ^= 0xFF
            pkt[-2] ^= 0xFF
            injected = True
            self.stats["corrupt"] += 1

        self.stats["emitted"] += 1
        if injected:
            print(f"[VirtualMCU] 注入异常帧 idx={idx}")
        return bytes(pkt)

    # ---------------------------------------------------------------- 主循环
    def run(self):
        a = self.args
        if a.replay:
            return self.run_replay()

        self.open_pty()

        imu_period = 1.0 / a.imu_hz if a.imu_hz > 0 else 0.002
        chassis_period = 1.0 / a.chassis_hz if a.chassis_hz > 0 else 0.02
        status_period = 1.0 / a.status_hz if a.status_hz > 0 else 1.0
        slow_period = 1.0
        reset_period = a.reset_period if a.reset_period > 0 else float("inf")

        t0 = time.monotonic()
        next_imu = next_chassis = next_status = next_slow = 0.0
        next_reset = reset_period
        idx = 0
        last_log = t0

        print(
            f"[VirtualMCU] 开始供数: gimbal={a.imu_hz}Hz chassis={a.chassis_hz}Hz "
            f"status={a.status_hz}Hz drop={a.drop_rate} delay={a.delay_ms}ms "
            f"spike_every={a.spike_every} corrupt_crc={a.corrupt_crc_rate}"
        )

        while not self._stop:
            rel = time.monotonic() - t0
            uptime_ms = int(rel * 1000.0) & 0xFFFFFFFF

            if rel >= next_imu:
                pkt = self.emit_gimbal(idx)
                idx += 1
                if pkt:
                    self.write(pkt)
                next_imu += imu_period
                if next_imu < rel:
                    next_imu = rel + imu_period

            if rel >= next_chassis:
                self.write(pack_chassis(
                    1.0 * math.sin(2 * math.pi * 0.2 * rel),
                    0.3 * math.cos(2 * math.pi * 0.2 * rel),
                    0.5 * math.sin(2 * math.pi * 0.05 * rel),
                    heading=math.pi * math.sin(2 * math.pi * 0.02 * rel),
                    seq=(int(rel * a.chassis_hz) & 0xFF),
                ))
                next_chassis += chassis_period
                if next_chassis < rel:
                    next_chassis = rel + chassis_period

            if rel >= next_slow:
                self.write(pack_host_version(uptime_ms))
                self.write(pack_host_robot())
                next_slow += slow_period
                if next_slow < rel:
                    next_slow = rel + slow_period

            if rel >= next_status:
                self.write(pack_game_status())
                self.write(pack_robot_status())
                next_status += status_period
                if next_status < rel:
                    next_status = rel + status_period

            if rel >= next_reset:
                # 新协议没有 reset 位：MCU 重启（uptime_ms 回落）就是视觉侧的复位信号
                self.write(pack_host_version(0))
                print("[VirtualMCU] 注入 MCU 重启信号 0x0500.uptime_ms=0")
                next_reset += reset_period

            self.drain_input()

            if time.monotonic() - last_log >= 2.0:
                print(f"[VirtualMCU] 统计: {self.stats}")
                last_log = time.monotonic()

            if a.duration > 0 and rel >= a.duration:
                print("[VirtualMCU] 达到运行时长，退出")
                break

            nxt = min(next_imu, next_chassis, next_slow, next_status)
            wait = nxt - (time.monotonic() - t0)
            if wait > 0:
                time.sleep(min(wait, 0.002))

        self.close()

    # ---------------------------------------------------------------- 回放
    def run_replay(self):
        records = self.load_serial_bin(self.args.replay)
        if not records:
            print(f"[VirtualMCU] 回放文件无有效记录: {self.args.replay}", file=sys.stderr)
            return 1

        self.open_pty()
        base = records[0][0]
        print(f"[VirtualMCU] 回放 {self.args.replay}: {len(records)} 条(rx)，span={(records[-1][0] - base) / 1e9:.2f}s")

        loop = 0
        while not self._stop:
            loop += 1
            t0 = time.monotonic()
            for t_ns, data in records:
                if self._stop:
                    break
                target = t0 + (t_ns - base) / 1e9 / self.args.speed
                remain = target - time.monotonic()
                while remain > 0 and not self._stop:
                    time.sleep(min(remain, 0.002))
                    remain = target - time.monotonic()
                self.write(data)
            if not self.args.loop:
                break
            print(f"[VirtualMCU] 回放第 {loop} 轮结束，重新开始")

        self.close()
        return 0

    @staticmethod
    def load_serial_bin(path):
        """读取 C++ SerialRecorder 的 serial.bin，返回 [(t_ns, bytes)]，仅下位机→视觉方向"""
        with open(path, "rb") as f:
            magic = f.read(8)
            if magic != b"RMSERIAL":
                raise ValueError(f"{path} 不是 SerialRecorder 文件")
            f.read(8)   # start_unix_ns
            f.read(4)   # record_count
            out = []
            while True:
                head = f.read(8 + 1 + 4)
                if len(head) < 13:
                    break
                t_ns, direction, length = struct.unpack("<QBI", head)
                data = f.read(length)
                if len(data) < length:
                    break
                if direction == 0:  # rx
                    out.append((t_ns, data))
            return out

    # ---------------------------------------------------------------- 收尾
    def close(self):
        if self.master_fd is not None:
            try:
                os.close(self.master_fd)
            except OSError:
                pass
            self.master_fd = None
        print(f"[VirtualMCU] 退出，最终统计: {self.stats}")

    def stop(self, *_):
        self._stop = True


def main():
    p = argparse.ArgumentParser(description="RM 虚拟下位机（pty 伪串口）")
    p.add_argument("--port-file", help="把创建的 pts 路径写入该文件")
    p.add_argument("--link", help="为 pts 创建软链接（方便固定路径使用）")
    p.add_argument("--imu-hz", type=float, default=500.0, help="云台姿态(0x0503)频率")
    p.add_argument("--chassis-hz", type=float, default=50.0, help="底盘反馈(0x0502)频率")
    p.add_argument("--status-hz", type=float, default=1.0, help="裁判帧(0x0001/0x0201)频率")
    p.add_argument(
        "--reset-period", type=float, default=0.0, help="每 N 秒注入一次 MCU 重启(0=关闭)"
    )
    p.add_argument("--drop-rate", type=float, default=0.0, help="丢包概率")
    p.add_argument("--delay-ms", type=float, default=0.0, help="每帧额外延迟(ms)")
    p.add_argument("--spike-every", type=int, default=0, help="每 N 帧注入一次跳变(0=关闭)")
    p.add_argument("--spike-rad", type=float, default=1.5708, help="跳变幅度(rad)")
    p.add_argument("--corrupt-crc-rate", type=float, default=0.0, help="校验错误概率")
    p.add_argument("--noise-rad", type=float, default=0.0, help="角度噪声标准差(rad)")
    p.add_argument("--seed", type=int, default=20261015, help="随机种子（保证异常可复现）")
    p.add_argument("--duration", type=float, default=0.0, help="运行时长(秒)，0=一直运行")
    p.add_argument("--replay", help="回放 C++ 记录的 serial.bin")
    p.add_argument("--speed", type=float, default=1.0, help="回放倍速")
    p.add_argument("--loop", action="store_true", help="回放循环")
    p.add_argument("--verbose", action="store_true", help="打印主程序下发内容")
    args = p.parse_args()

    mcu = VirtualMcu(args)
    signal.signal(signal.SIGINT, mcu.stop)
    signal.signal(signal.SIGTERM, mcu.stop)
    return mcu.run() or 0


if __name__ == "__main__":
    sys.exit(main())
