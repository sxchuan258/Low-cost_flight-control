#include "math.h"
#include "iic_init.h"

unsigned short dig_T1;
short dig_T2;
short dig_T3;
unsigned short dig_P1;
short dig_P2;
short dig_P3;
short dig_P4;
short dig_P5;
short dig_P6;
short dig_P7;
short dig_P8;
short dig_P9;


unsigned char IICReadbyte(I2C_TypeDef *I2Cx,unsigned char chip_address,unsigned char address)
{
    unsigned char lsb=0;
    I2C_AcknowledgeConfig(I2Cx,ENABLE);
    I2C_GenerateSTART(I2Cx,ENABLE);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2Cx, chip_address, I2C_Direction_Transmitter);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2Cx,address);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_GenerateSTART(I2Cx,ENABLE);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2Cx, chip_address, I2C_Direction_Receiver);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
    lsb = I2C_ReceiveData(I2Cx);
    I2C_AcknowledgeConfig(I2Cx,DISABLE);
    I2C_GenerateSTOP(I2Cx,ENABLE);

    return lsb;
}

short bmp280ReadShort(I2C_TypeDef *I2Cx,unsigned char address)
{
  short msb=0;
  short lsb=0;
  I2C_AcknowledgeConfig(I2Cx,ENABLE);
  I2C_GenerateSTART(I2Cx,ENABLE);

  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2Cx, BMP280_addr, I2C_Direction_Transmitter);

  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2Cx,address);
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTART(I2Cx,ENABLE);
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2Cx, BMP280_addr, I2C_Direction_Receiver);
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
  lsb = I2C_ReceiveData(I2Cx);

  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
  msb = I2C_ReceiveData(I2Cx);

  I2C_AcknowledgeConfig(I2Cx,DISABLE);
  I2C_GenerateSTOP(I2Cx,ENABLE);

  return (msb << 8) | lsb;
}

unsigned long bmp280ReadLong(I2C_TypeDef *I2Cx,unsigned char address)
{
  unsigned long result=0;

  unsigned long msb=0;
  unsigned long lsb=0;
  unsigned long xsb=0;

  I2C_AcknowledgeConfig(I2Cx,ENABLE);
  I2C_GenerateSTART(I2Cx,ENABLE);
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2Cx, BMP280_addr, I2C_Direction_Transmitter);
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2Cx,address);
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTART(I2Cx,ENABLE);
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2Cx, BMP280_addr, I2C_Direction_Receiver);
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
  msb = I2C_ReceiveData(I2Cx);

  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
  lsb = I2C_ReceiveData(I2Cx);

  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
  xsb = I2C_ReceiveData(I2Cx);

  I2C_AcknowledgeConfig(I2Cx,DISABLE);
  I2C_GenerateSTOP(I2Cx,ENABLE);

  result = (msb << 16) | (lsb << 8) | xsb;

  return (result >> 4);
}

void IIC_WriteByte(I2C_TypeDef *I2Cx,unsigned char chip_address,unsigned char address, unsigned char data)
{
    I2C_GenerateSTART(I2Cx,ENABLE);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2Cx, chip_address, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2Cx,address);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_SendData(I2Cx,data);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTOP(I2Cx,ENABLE);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

}

int bmp280Convert()
{
  unsigned long adc_T;
  unsigned long adc_P;
  adc_T = bmp280ReadLong(I2C2,BMP280_REG_RESULT_TEMPRERATURE);
  adc_P = bmp280ReadLong(I2C2,BMP280_REG_RESULT_PRESSURE);

  double var1, var2, p, t_fine;
  var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
  var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) * (((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
  t_fine = (var1 + var2);

  var1 = ((double)t_fine/2.0) - 64000.0;
  var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)dig_P5) * 2.0;
  var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
  var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);

  p = 1048576.0 - (double)adc_P;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double)dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double)dig_P8) / 32768.0;
  p = (p + (var1 + var2 + ((double)dig_P7)) / 16.0);

  return p;
}

double Highlight()
{
    unsigned long P;
    double H;
    P=bmp280Convert();
    H=(101352-P)/12.7;
    return H;
}

void BMP280_Init(void)
{
  dig_T1 = bmp280ReadShort(I2C2,0x88);//dig_T1
  dig_T2 = bmp280ReadShort(I2C2,0x8A);//dig_T2
  dig_T3 = bmp280ReadShort(I2C2,0x8C);//dig_T3
  dig_P1 = bmp280ReadShort(I2C2,0x8E);//dig_P1
  dig_P2 = bmp280ReadShort(I2C2,0x90);//dig_P2
  dig_P3 = bmp280ReadShort(I2C2,0x92);//dig_P3
  dig_P4 = bmp280ReadShort(I2C2,0x94);//dig_P4
  dig_P5 = bmp280ReadShort(I2C2,0x96);//dig_P5
  dig_P6 = bmp280ReadShort(I2C2,0x98);//dig_P6
  dig_P7 = bmp280ReadShort(I2C2,0x9A);//dig_P7
  dig_P8 = bmp280ReadShort(I2C2,0x9C);//dig_P8
  dig_P9 = bmp280ReadShort(I2C2,0x9E);//dig_P9

  IIC_WriteByte(I2C2,BMP280_addr,BMP280_REG_CONFIG, BMP280_CONFIG);
  IIC_WriteByte(I2C2,BMP280_addr,BMP280_REG_CONTROL, BMP280_MEAS);
}

void IIC1_Init(u32 bound, u16 address)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef I2C_InitTSturcture={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitTSturcture );
    I2C_Cmd( I2C1, ENABLE );
    I2C_AcknowledgeConfig(I2C1,ENABLE);
}

void IIC2_Init(u32 bound, u16 address)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef I2C_InitTSturcture={0};
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C2, &I2C_InitTSturcture );
    I2C_Cmd( I2C2, ENABLE );
    I2C_AcknowledgeConfig(I2C2,ENABLE);
}

void MPU6050_Iint(void)
{
    IIC_WriteByte(I2C1,SlaveAddress,PWR_MGMT_1, 0x00);
    IIC_WriteByte(I2C1,SlaveAddress,SMPLRT_DIV, 0x07);
    IIC_WriteByte(I2C1,SlaveAddress,CONFIG, 0x06);
    IIC_WriteByte(I2C1,SlaveAddress,GYRO_CONFIG, 0x08);
    IIC_WriteByte(I2C1,SlaveAddress,ACCEL_CONFIG, 0x01);
}

double MPU6050_Accel_GetData(unsigned char REG_Address)
{
    unsigned char H,L;
    u_int16_t a;
    double accel=0;
    H=IICReadbyte(I2C1,SlaveAddress,REG_Address);
    L=IICReadbyte(I2C1,SlaveAddress,REG_Address+1);
    a=(H<<8)+L;
    if (a>32767) {
        accel=-((~a)&0x7fff)*9.8/16384;
    }else {
        accel=a*9.8/16384;
    }
    return accel;   //合成数据
}

double MPU6050_Gyro_GetData(unsigned char REG_Address)
{
    unsigned char H,L;
    u_int16_t a;
    double gyro=0;
    H=IICReadbyte(I2C1,SlaveAddress,REG_Address);
    L=IICReadbyte(I2C1,SlaveAddress,REG_Address+1);
    a=(H<<8)+L;
    if (a>32767) {
        gyro=-((~a)&0x7fff)*250.0/16384;
    }else {
        gyro=a*250.0/16384;
    }
    return gyro;
}

void QMC5883_Init(void)
{
    IIC_WriteByte(I2C1,qmc_address, control, 0x0d);
    IIC_WriteByte(I2C1,qmc_address, 0x0B, 0x01);
    /*IIC_WriteByte(I2C1,qmc_address, 0x20, 0x40);
    IIC_WriteByte(I2C1,qmc_address, 0x21, 0x01);*/
}

double QMC5883_Getdata(unsigned char REG_Address)
{
    unsigned char M,L;
    double M_S;
    L=IICReadbyte(I2C1, qmc_address, REG_Address);
    M=IICReadbyte(I2C1, qmc_address, REG_Address+1);
    M_S=((M<<8)+L);
    if (M_S>0x7fff)M_S-=0xffff;

    return M_S;
}

double Mpu_roll()
{
    double y;
    double z;
    double roll;
    y=MPU6050_Accel_GetData(ACCEL_YOUT_H);
    z=MPU6050_Accel_GetData(ACCEL_ZOUT_H);
    roll=atan(y*1.0/z)*57.295780;
    return roll;
}

double Mpu_pitch()
{
    double x;
    double y;
    double z;
    double picth;
    x=MPU6050_Accel_GetData(ACCEL_XOUT_H);
    y=MPU6050_Accel_GetData(ACCEL_YOUT_H);
    z=MPU6050_Accel_GetData(ACCEL_ZOUT_H);
    picth=-atan(x/pow((y*y+z*z),0.5))*57.295780;
    return picth;
}

double Get_Yaw()
{
    double a,b;   /*中间变量*/
    double M2x,M2y,M2z;
    double roll,picth,yaw;
    roll=Mpu_roll()/57.29578;
    picth=Mpu_pitch()/57.29578;
    M2x=QMC5883_Getdata(XL);
    M2y=QMC5883_Getdata(YL);
    M2z=QMC5883_Getdata(ZL);
    b=picth;
    picth=roll;
    roll=-b;
    a=M2z*cos(picth)*sin(roll)-M2x*sin(picth)*sin(roll)-M2y*cos(roll);
    a=a/(M2x*cos(picth)+M2z*sin(picth));
    yaw=atan(a)*57.29578;
    return yaw ;
}

double Convert_Gyro_x(double roll,double picth)
{
    double gyro_x;
    double gyro_y;
    double gyro_z;

    roll=roll/57.29578;
    picth=picth/57.29578;

    gyro_x=MPU6050_Gyro_GetData(GYRO_XOUT_H);
    gyro_y=MPU6050_Gyro_GetData(GYRO_YOUT_H);
    gyro_z=MPU6050_Gyro_GetData(GYRO_ZOUT_H);

    gyro_x=gyro_x+gyro_y*sin(roll)*tan(picth)+gyro_z*cos(roll)*tan(picth);

    return gyro_x;
}

double Convert_Gyro_y(double roll,double picth)
{
    double gyro_y;
    double gyro_z;

    roll=roll/57.29578;
    picth=picth/57.29578;

    gyro_y=MPU6050_Gyro_GetData(GYRO_YOUT_H);
    gyro_z=MPU6050_Gyro_GetData(GYRO_ZOUT_H);

    gyro_y=gyro_y*cos(roll)-gyro_z*sin(roll);

    return gyro_y;
}

double Convert_Gyro_z(double roll,double picth)
{
    double gyro_y;
    double gyro_z;

    roll=roll/57.29578;
    picth=picth/57.29578;

    gyro_y=MPU6050_Gyro_GetData(GYRO_YOUT_H);
    gyro_z=MPU6050_Gyro_GetData(GYRO_ZOUT_H);

    gyro_z=gyro_y*sin(roll)/cos(picth)+gyro_z*cos(roll)/cos(picth);

    return gyro_z;
}

