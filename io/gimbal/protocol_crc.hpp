#ifndef IO__GIMBAL__PROTOCOL_CRC_HPP
#define IO__GIMBAL__PROTOCOL_CRC_HPP

#include <cstdint>

namespace crc8
{
uint8_t Verify_CRC8_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);
void Append_CRC8_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);
}  // namespace crc8

namespace crc16
{
uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);
}  // namespace crc16

#endif  // IO__GIMBAL__PROTOCOL_CRC_HPP
