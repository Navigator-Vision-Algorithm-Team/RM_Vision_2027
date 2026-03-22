import serial
import struct
import sys
import time
from typing import Tuple

class CRC:
    
    @staticmethod
    def get_crc8_check_sum(data: bytes, init_val: int = 0xFF) -> int:
        crc8_tab = [
            0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
            0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
            0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
            0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
            0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
            0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
            0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
            0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
            0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
            0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
            0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
            0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
            0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
            0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
            0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
            0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
        ]
        
        crc = init_val
        for byte in data:
            index = crc ^ byte
            crc = crc8_tab[index]
        return crc
    
    @staticmethod
    def get_crc16_check_sum(data: bytes, init_val: int = 0xFFFF) -> int:
        w_crc_table = [
            0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
            0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
            0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
            0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
            0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
            0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
            0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
            0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
            0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
            0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
            0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
            0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
            0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
            0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
            0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
            0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
            0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
            0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
            0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
            0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
            0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
            0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
        ]
        
        crc = init_val
        for byte in data:
            index = (crc ^ byte) & 0xFF
            crc = ((crc >> 8) ^ w_crc_table[index]) & 0xFFFF
        return crc


class GimbalController:
    
    # Header: sof(1) + data_length(2) + seq(1) + crc8(1) + cmd_id(2) = 7 bytes
    HEADER_SIZE = 7
    # SendPacket: Header(7) + id(1) + robo_id(1) + pitch(4) + yaw(4) + accuracy(1) + shoot(1) + crc16(2) = 21 bytes
    SEND_PACKET_SIZE = 21
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 921600, timeout: float = 1.0, simulate: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.seq = 0
        self.simulate = simulate  # 模拟模式：不需要真实串口
        
    def connect(self) -> bool:
        if self.simulate:
            print(f"✓ [模拟模式] 已连接到虚拟端口 {self.port}")
            return True
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            return True
        except serial.SerialException as e:
            print(f"✗ fail connect {self.port}: {e}")
            return False
    
    def disconnect(self):
        if self.simulate:
            print("✓ [模拟模式] 已断开连接")
            return
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ success disconnect")
    
    def _build_send_packet(self, pitch: float, yaw: float, shoot: int = 0, 
                          robo_id: int = 0, accuracy: int = 50) -> bytes:
        header_bytes = bytearray()
        header_bytes.append(0xA5)  # SOF (1 byte)
        
        # data_length = sizeof(packet) - sizeof(header) - 2
        # = 21 - 7 - 2 = 12
        data_length = self.SEND_PACKET_SIZE - self.HEADER_SIZE - 2
        header_bytes.extend(struct.pack('<H', data_length))  # data_length (2 bytes)
        header_bytes.append(self.seq)  # seq (1 byte)
        self.seq = (self.seq + 1) % 256
        
        # CRC8 计算前 4 个字节 (SOF + data_length + seq)，结果放在第 5 个字节
        crc8_val = CRC.get_crc8_check_sum(bytes(header_bytes[:4]))
        header_bytes.append(crc8_val)  # crc8 (1 byte)
        
        header_bytes.extend(struct.pack('<H', 0x0402))  # cmd_id (2 bytes)
        # 此时 header_bytes 共 7 字节
        
        packet_data = bytearray()
        packet_data.extend(header_bytes)
        
        packet_data.append(0)  # id (1 byte)
        packet_data.append(robo_id)  # robo_id (1 byte)
        packet_data.extend(struct.pack('<f', pitch))  # pitch (4 bytes)
        packet_data.extend(struct.pack('<f', yaw))  # yaw (4 bytes)
        packet_data.append(accuracy)  # accuracy (1 byte)
        packet_data.append(shoot)  # shoot (1 byte)
        # 此时 packet_data 共 19 字节
        
        # CRC16 计算前 19 个字节，结果追加到末尾
        crc16_val = CRC.get_crc16_check_sum(bytes(packet_data))
        packet_data.extend(struct.pack('<H', crc16_val))  # crc16 (2 bytes)
        # 最终 packet_data 共 21 字节
        
        return bytes(packet_data)
    
    def send_gimbal_angle(self, pitch: float, yaw: float, shoot: int = 0) -> bool:
        packet = self._build_send_packet(pitch, yaw, shoot)
        
        if self.simulate:
            # 模拟模式：显示数据包内容
            print(f"✓ [模拟模式] Pitch={pitch:.4f} rad ({pitch*180/3.14159:.2f}°), Yaw={yaw:.4f} rad ({yaw*180/3.14159:.2f}°), Shoot={shoot}")
            print(f"  数据包 ({len(packet)} bytes): {packet.hex(' ')}")
            return True
        
        if not self.ser or not self.ser.is_open:
            print("✗ no connect")
            return False
        
        try:
            self.ser.write(packet)
            print(f"✓ : Pitch={pitch:.4f} rad, Yaw={yaw:.4f} rad, Shoot={shoot}")
            return True
        except serial.SerialException as e:
            print(f"✗ fail: {e}")
            return False
    
    def send_gimbal_angle_degrees(self, pitch_deg: float, yaw_deg: float, shoot: int = 0) -> bool:

        import math
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)
        return self.send_gimbal_angle(pitch_rad, yaw_rad, shoot)


def print_help():
    """打印帮助信息"""
    print("""
================== 云台控制工具 ==================
使用方法:
  p <pitch>  - 设置Pitch角度 (单位: 弧度)
  y <yaw>    - 设置Yaw角度 (单位: 弧度)
  d <pitch> <yaw> - 以角度制设置 (单位: 度)
  s <pitch> <yaw> <shoot> - 发送完整数据 (pitch yaw单位:弧度)
  c <port>   - 切换串口 (例如: c /dev/ttyUSB0)
  b <baud>   - 设置波特率 (例如: b 115200)
  connect    - 连接串口
  disconnect - 断开串口
  help       - 显示帮助
  exit       - 退出程序

示例:
  d 0 45     - 发送Pitch=0°, Yaw=45°
  s 0.1 0.5 0 - 发送Pitch=0.1rad, Yaw=0.5rad, 不射击
  s 0.1 0.5 1 - 发送Pitch=0.1rad, Yaw=0.5rad, 射击

==================================================
    """)


def main():
    
    # Check command line arguments
    port = '/dev/ttyACM0'
    baudrate = 921600
    simulate = False
    
    # 检查是否有 --simulate 或 -s 参数
    if '--simulate' in sys.argv or '-s' in sys.argv:
        simulate = True
        # 移除这些参数以便后续处理
        sys.argv = [arg for arg in sys.argv if arg not in ('--simulate', '-s')]
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            baudrate = int(sys.argv[2])
        except ValueError:
            print(f"Warning: Invalid baudrate {sys.argv[2]}, using default 921600")
    
    controller = GimbalController(port, baudrate, simulate=simulate)
    
    if simulate:
        print("========== 模拟模式 ==========")
    print(f"Config: Port={port}, Baudrate={baudrate}")
    print()
    
    # Connect to serial port
    if not controller.connect():
        print("Exiting...")
        return
    
    pitch = 0.0
    yaw = 0.0
    
    try:
        while True:
            user_input = input("> ").strip()
            
            if not user_input:
                continue
            
            tokens = user_input.split()
            cmd = tokens[0].lower()
            
            if cmd == 'help':
                print_help()
            
            elif cmd == 'p':
                try:
                    pitch = float(tokens[1])
                    print(f"  Pitch set to: {pitch:.4f} rad ({pitch*180/3.14159:.2f}°)")
                except (IndexError, ValueError):
                    print("  Error: Usage p <pitch>")
            
            elif cmd == 'y':
                try:
                    yaw = float(tokens[1])
                    print(f"  Yaw set to: {yaw:.4f} rad ({yaw*180/3.14159:.2f}°)")
                except (IndexError, ValueError):
                    print("  Error: Usage y <yaw>")
            
            elif cmd == 'd':
                try:
                    import math
                    pitch_deg = float(tokens[1])
                    yaw_deg = float(tokens[2])
                    pitch = math.radians(pitch_deg)
                    yaw = math.radians(yaw_deg)
                    print(f"  Sent: Pitch={pitch_deg}°, Yaw={yaw_deg}°")
                    controller.send_gimbal_angle(pitch, yaw, 0)
                except (IndexError, ValueError):
                    print("  Error: Usage d <pitch_deg> <yaw_deg>")
            
            elif cmd == 's':
                try:
                    print(f"  [Debug] raw input: '{user_input}'")
                    print(f"  [Debug] tokens: {tokens}, len={len(tokens)}")
                    print(f"  [Debug] tokens[1]: '{tokens[1]}', tokens[2]: '{tokens[2]}'")
                    pitch = float(tokens[1])
                    yaw = float(tokens[2])
                    shoot = int(tokens[3]) if len(tokens) > 3 else 0
                    controller.send_gimbal_angle(pitch, yaw, shoot)
                except (IndexError, ValueError) as e:
                    print(f"  Error: Usage s <pitch> <yaw> [shoot] (reason: {e})")
            
            elif cmd == 'c':
                try:
                    port = tokens[1]
                    controller.disconnect()
                    controller.port = port
                    if controller.connect():
                        pass
                except IndexError:
                    print("  Error: Usage c <port>")
            
            elif cmd == 'b':
                try:
                    baudrate = int(tokens[1])
                    controller.disconnect()
                    controller.baudrate = baudrate
                    if controller.connect():
                        pass
                except (IndexError, ValueError):
                    print("  Error: Usage b <baudrate>")
            
            elif cmd == 'connect':
                if not controller.ser or not controller.ser.is_open:
                    controller.connect()
                else:
                    print("  ✓ Connected to serial port")
            
            elif cmd == 'disconnect':
                controller.disconnect()
            
            elif cmd == 'exit' or cmd == 'quit':
                print("Exiting...")
                break
            
            else:
                print(f"Unknown command: {cmd}, type 'help' for assistance")
    
    except KeyboardInterrupt:
        print("\nexit...")
    finally:
        controller.disconnect()


if __name__ == '__main__':
    main()
