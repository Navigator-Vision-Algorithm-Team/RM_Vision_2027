#ifndef IO__GIMBAL__PROTOCOL_CRC_HPP
#define IO__GIMBAL__PROTOCOL_CRC_HPP

#include <cstdint>

namespace crc8
{
/*
 * 帧头 CRC8（RoboMaster 标准，init 0xFF，查表实现）。
 *
 * dwLength 包含末尾 1 字节校验位：Verify 计算前 dwLength-1 字节并与最后 1 字节比较，
 * 返回 1 表示通过；Append 把校验值写入 pchMessage[dwLength-1]。
 */
uint8_t Verify_CRC8_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);
void Append_CRC8_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);
}  // namespace crc8

namespace crc16
{
/*
 * 整帧 CRC16（RoboMaster 标准，init 0xFFFF，查表实现）。
 *
 * dwLength 包含末尾 2 字节校验位，CRC 以小端写入：Verify 返回非 0 表示通过，
 * Append 把校验值写入最后 2 字节。
 *
 * 查表值必须与固件逐项一致，不能用「标准」0x1021 前向表替代。
 */
uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);
}  // namespace crc16

#endif  // IO__GIMBAL__PROTOCOL_CRC_HPP
