#ifndef ADXL345_H
#define ADXL345_H

#if 0
//ioctl()获取数据范围的命令
#define ADXL345_GET_RANGE 101
//ioctl()设置数据范围的命令
#define ADXL345_SET_RANGE 102
//ioctl()获取采样率的命令
#define ADXL345_GET_DATARATE 103
//ioctl()设置采样率的命令
#define ADXL345_SET_DATARATE 104
#endif

//可选数据范围
typedef enum
{
    ADXL345_RANGE_16_G          = 0b11,   // +/- 16g
    ADXL345_RANGE_8_G           = 0b10,   // +/- 8g
    ADXL345_RANGE_4_G           = 0b01,   // +/- 4g
    ADXL345_RANGE_2_G           = 0b00    // +/- 2g
}
ADXL345_range_t;

//可选采样率
typedef enum
{
    ADXL345_DATARATE_3200_HZ    = 0b1111, // 1600Hz
    ADXL345_DATARATE_1600_HZ    = 0b1110, //  800Hz
    ADXL345_DATARATE_800_HZ     = 0b1101, //  400Hz
    ADXL345_DATARATE_400_HZ     = 0b1100, //  200Hz
    ADXL345_DATARATE_200_HZ     = 0b1011, //  100Hz
    ADXL345_DATARATE_100_HZ     = 0b1010, //   50Hz
    ADXL345_DATARATE_50_HZ      = 0b1001, //   25Hz
    ADXL345_DATARATE_25_HZ      = 0b1000, // 12.5Hz
    ADXL345_DATARATE_12_5_HZ    = 0b0111, // 6.25Hz
    ADXL345_DATARATE_6_25HZ     = 0b0110, // 3.13Hz
    ADXL345_DATARATE_3_13_HZ    = 0b0101, // 1.56Hz
    ADXL345_DATARATE_1_56_HZ    = 0b0100, // 0.78Hz
    ADXL345_DATARATE_0_78_HZ    = 0b0011, // 0.39Hz
    ADXL345_DATARATE_0_39_HZ    = 0b0010, // 0.20Hz
    ADXL345_DATARATE_0_20_HZ    = 0b0001, // 0.10Hz
    ADXL345_DATARATE_0_10_HZ    = 0b0000  // 0.05Hz
}
ADXL345_dataRate_t;
#define ADXL345_DEVICE_ADDR     0x53    //adxl345 i2c address
#define ADXL345_DEVICE_ID       0xE5    //Device ID
#define ADXL345_REG_DEVID       0x00    // Device ID
#define ADXL345_REG_BW_RATE     0x2C    // Data rate and power mode control
#define ADXL345_REG_POWER_CTL   0x2D    // Power-saving features control
#define ADXL345_REG_INT_ENABLE  0x2E    // Interrupt enable switch
#define ADXL345_REG_DATA_FORMAT 0x31    // Data format control
#define ADXL345_REG_DATAX0      0x32    // X-axis data 0
#define ADXL345_REG_DATAX1      0x33    // X-axis data 1
#define ADXL345_REG_DATAY0      0x34    // Y-axis data 0
#define ADXL345_REG_DATAY1      0x35    // Y-axis data 1
#define ADXL345_REG_DATAZ0      0x36    // Z-axis data 0
#define ADXL345_REG_DATAZ1      0x37    // Z-axis data 1

#endif
