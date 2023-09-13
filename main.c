/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 USART Print debugging routine:
 USART1_Tx(PA9).
 This example demonstrates using USART1(PA9) as a print debug port output.

*/

#include "debug.h"
#include "iic_init.h"
#include "w25q16.h"

void TIM3_conter_init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period=arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM3, &TIM_TimeBaseInitStructure);

    //TIM_ARRPreloadConfig( TIM2, ENABLE );
}

void GPIO_PA15_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    u8 i=0;
    double H;
    double roll,picth,yaw=0;
    double roll_2=0,picth_2=0;
    double gyro_x,gyro_y,gyro_z;
    double sample_time=0.1;
    double time=0;
    double p[2][2]={{1,0},{0,1}};
    double p_1[2][2]={{0,0},{0,0}};
    double K[2][2]={{0,0},{0,0}};
    const double Q[2][2]={{0.04,0},{0,0.04}};
    const double R[2][2]={{0.0001,0},{0,0.0001}};

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(9600);
    printf("SystemClk:%d\r\n", SystemCoreClock);

    TIM3_conter_init(65535-1,48-1);
    GPIO_PA15_INIT();

    IIC1_Init(100000, 0x38);
    IIC2_Init(100000, 0x38);
    MPU6050_Iint();
    BMP280_Init();
    QMC5883_Init();

    while(1)
    {
        TIM_Cmd(TIM3, ENABLE);
        TIM3->CNT=0;
        H=Highlight();
        roll=Mpu_roll();
        picth=Mpu_pitch();
        gyro_x=Convert_Gyro_x(roll, picth)+0.9;
        gyro_y=Convert_Gyro_y(roll, picth);
        gyro_z=Convert_Gyro_z(roll, picth)+0.575;
        roll_2=roll_2+gyro_x*sample_time;
        picth_2=picth_2+gyro_y*sample_time;
        /*yaw=yaw+gyro_z*sample_time;*/
        yaw=Get_Yaw();
        p_1[0][0]=p[0][0]+Q[0][0];
        p_1[1][1]=p[1][1]+Q[1][1];
        K[0][0]=p_1[0][0]/(p_1[0][0]+R[0][0]);
        K[1][1]=p_1[1][1]/(p_1[1][1]+R[1][1]);
        roll=roll_2+K[0][0]*(roll-roll_2);
        picth=picth_2+K[1][1]*(picth-picth_2);
        p[0][0]=(1-K[0][0])*p_1[0][0];
        p[1][1]=(1-K[1][1])*p_1[1][1];
        printf("time and information is:%f,%f,%f,%f,%f\r\n",time,H,roll,picth,yaw);

        if (i>10) {
            GPIO_ResetBits(GPIOA, GPIO_Pin_15);
            if (i>20) {
                i=0;
            }
        }else {
            GPIO_SetBits(GPIOA, GPIO_Pin_15);
        }
        i++;
        time=TIM3->CNT/2000000.0;
        TIM_Cmd(TIM3, DISABLE);
        Delay_Ms((sample_time-time)*1000);
    }
}
