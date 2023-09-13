#ifndef HARDWIRE_IIC_INIT_H_
#define HARDWIRE_IIC_INIT_H_
#include "ch32v20x_conf.h"
/*BMP280的寄存器定义*/
#define _BMP280_LIB_H_
#define BMP280_addr 0xED
#define BMP280_REG_CONTROL 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_RESULT_PRESSURE 0xF7         // 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data.
#define BMP280_REG_RESULT_TEMPRERATURE 0xFA     // 0xFA(msb) , 0xFB(lsb) , 0xFC(xlsb) : stores the temperature data.
#define BMP280_OVERSAMPLING_T1      0x20
#define BMP280_OVERSAMPLING_T2      0x40
#define BMP280_OVERSAMPLING_T4      0x60
#define BMP280_OVERSAMPLING_T8      0x80
#define BMP280_OVERSAMPLING_T16     0xA0
#define BMP280_OVERSAMPLING_P1      0x04
#define BMP280_OVERSAMPLING_P2      0x08
#define BMP280_OVERSAMPLING_P4      0x0C
#define BMP280_OVERSAMPLING_P8      0x10
#define BMP280_OVERSAMPLING_P16     0x14
#define BMP280_MODE_SLEEP           0x00
#define BMP280_MODE_FORCED          0x01
#define BMP280_MODE_NORMAL          0x03
#define BMP280_TSB_0_5              0x00
#define BMP280_TSB_62_5             0x20
#define BMP280_TSB_125              0x40
#define BMP280_TSB_250              0x60
#define BMP280_TSB_500              0x80
#define BMP280_TSB_1000             0xA0
#define BMP280_TSB_2000             0xC0
#define BMP280_TSB_4000             0xE0
#define BMP280_FILTER_OFF           0x00
#define BMP280_FILTER_COEFFICIENT2  0x04
#define BMP280_FILTER_COEFFICIENT4  0x08
#define BMP280_FILTER_COEFFICIENT8  0x0C
#define BMP280_FILTER_COEFFICIENT16 0x10
#define BMP280_SPI_OFF  0x00
#define BMP280_SPI_ON   0x01
#define BMP280_MEAS         (BMP280_OVERSAMPLING_T16 | BMP280_OVERSAMPLING_P16 | BMP280_MODE_NORMAL)
#define BMP280_CONFIG       (BMP280_TSB_0_5 | BMP280_FILTER_COEFFICIENT16 | BMP280_SPI_OFF)

/*MPU6065的寄存器定义*/
#define SMPLRT_DIV      0x19    //陀螺仪采样率，典型值：0x07(125Hz)
#define CONFIG          0x1A    //低通滤波频率，典型值：0x06(5Hz)
#define GYRO_CONFIG     0x1B    //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define ACCEL_CONFIG    0x1C    //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48
#define PWR_MGMT_1      0x6B    //电源管理，典型值：0x00(正常启用)
#define WHO_AM_I        0x75    //IIC地址寄存器(默认数值0x68，只读)
#define SlaveAddress    0xD0

/*QMC5883的寄存器定义*/
#define XL 0x00
#define XM 0x01
#define YL 0x02
#define YM 0x03
#define ZL 0x04
#define ZM 0x05
#define status 0x06
#define control 0x09
#define set_reset 0x0B
#define qmc_address 0x1a

unsigned char IICReadbyte(I2C_TypeDef *I2Cx,unsigned char chip_address,unsigned char address);
short bmp280ReadShort(I2C_TypeDef *I2Cx,unsigned char address);
unsigned long bmp280ReadLong(I2C_TypeDef *I2Cx,unsigned char address);
void IIC_WriteByte(I2C_TypeDef *I2Cx,unsigned char chip_address,unsigned char address, unsigned char data);
int bmp280Convert();
double Highlight();
void BMP280_Init(void);
void IIC2_Init(u32 bound, u16 address);
void IIC1_Init(u32 bound, u16 address);
void MPU6050_Iint(void);
double MPU6050_Accel_GetData(unsigned char REG_Address);
double MPU6050_Gyro_GetData(unsigned char REG_Address);
void QMC5883_Init(void);
double Mpu_roll();
double Mpu_pitch();
double Get_Yaw();
double Convert_Gyro_x(double roll,double picth);
double Convert_Gyro_y(double roll,double picth);
double Convert_Gyro_z(double roll,double picth);
double QMC5883_Getdata(unsigned char REG_Address);

#endif /* HARDWIRE_IIC_INIT_H_ */
