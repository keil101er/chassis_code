#ifndef TEST_TASK_H
#define TEST_TASK_H
#include "struct_typedef.h"
 void Buletooth_debug_task(void);

// STP-23激光测距传感器相关定义 / STP-23 Laser Ranging Sensor Definitions
#define POINT_PER_PACK 12       // 每帧测量点数 / Points per frame
#define HEADER 0x54             // 起始符 / Header
#define VERLEN 0x2C             // 版本长度 / Version Length

// 单个测量点数据结构 / Single Measurement Point Structure
typedef struct __attribute__((packed)) {
    uint16_t distance;      // 距离数据(mm) / Distance (mm)
    uint8_t  intensity;     // 信号强度 / Signal Intensity
} LidarPointStructDef;

// 激光雷达数据帧结构 / LiDAR Frame Structure
typedef struct __attribute__((packed)) {
    uint8_t  header;                        // 起始符 0x54 / Header
    uint8_t  ver_len;                       // 版本长度 0x2C / Version Length
    uint16_t temperature;                   // 温度值(ADC原始值0-4095) / Temperature
    uint16_t start_angle;                   // 起始角度(0.01度单位) / Start Angle
    LidarPointStructDef point[POINT_PER_PACK];  // 12个测量点 / 12 Points
    uint16_t end_angle;                     // 结束角度(0.01度单位) / End Angle
    uint16_t timestamp;                     // 时间戳(ms) / Timestamp
    uint8_t  crc8;                          // CRC8校验 / CRC8 Checksum
} LiDARFrameTypeDef;

// 函数声明 / Function Declarations
void STP23_Data_Process(void);  // 数据处理函数 / Data Processing Function
uint8_t CalCRC8(uint8_t *p, uint8_t len);  // CRC8校验函数 / CRC8 Checksum Function

#endif
