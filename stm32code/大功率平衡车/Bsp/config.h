#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f10x.h"
#include "millisecondtimer.h"

#define PI      					3.1415926
#define WHEEL_DIAMETER		0.065    //���ӵ�ֱ�� ��λ�� 
#define WHEEL_PERIMETER  	0.2042   //wheel's diameter in meters  ����תһ�� ǰ���ľ��� ��
#define COUNTS_PER_REV  	1560      //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev) ���� ��תһȦ ��������ֵ
//#define CAR_WIDTHS				0.142     //С���Ŀ��
//#define MAXLINESPEED			0.25			//�������ٶ� ���� ��λ ��/��
#define MAX_DISPLACE			25000				//���̼�¼λ������

#define MAGNETOMETER_ISIN 0						//�Ƿ�װ�˴�����  1 ��װ��  0 δ��װ

#define CAR_IMU_RATE 200 //hz					//��ȡ������ Ƶ��
#define CAR_CTL_RATE 200 //hz
#define BAT_PUBLISH_RATE 0.2 //hz
#define DEBUG_RATE 1

#define CAR_ZERO_ANGLE -6.5				//��е��ƫ ��λ ��

//�˲�Ȩ��ֵ
#define K1  0.05			//һ���˲�����  //���ٶ�ȡֵ��Ȩ��
#define K2  0.02			//�����˲�����  //���ٶ�ȡֵ��Ȩ��

enum Side{
	LEFT = 0,
	RIGHT = 1
}; 

//�����������  
#define LEFTBLIND 5
#define RIGHTBLIND 80

//С����ǹ�����ϵͳͣ��
#define FAILANGLE   60			//��λ ��  

#define 	USE_SERIAL1
#define 	USE_SERIAL2
#define 	USE_SERIAL3
/** --------���������������-------- **/
typedef enum {
	SERIAL1 = 0,
	SERIAL2 = 1,
	SERIAL3 = 2,
	SERIAL_END = 3
}Serial_TypeDef; 

#define SERIALn							3

#define ROS_SERIAL1									USART1
#define ROS_SERIAL1_IRQ							USART1_IRQn
#define ROS_SERIAL1_CLK             RCC_APB2Periph_USART1
#define ROS_SERIAL1_GPIO_CLK        RCC_APB2Periph_GPIOA
#define ROS_SERIAL1_GPIO_PORT       GPIOA
#define ROS_SERIAL1_TX_PIN          GPIO_Pin_9
#define ROS_SERIAL1_RX_PIN          GPIO_Pin_10
#define ROS_SERIAL1_NVIC						1

#define ROS_SERIAL2									USART2
#define ROS_SERIAL2_IRQ							USART2_IRQn
#define ROS_SERIAL2_CLK             RCC_APB1Periph_USART2
#define ROS_SERIAL2_GPIO_CLK        RCC_APB2Periph_GPIOA
#define ROS_SERIAL2_GPIO_PORT      	GPIOA
#define ROS_SERIAL2_TX_PIN          GPIO_Pin_2
#define ROS_SERIAL2_RX_PIN          GPIO_Pin_3
#define ROS_SERIAL2_NVIC						2

#define ROS_SERIAL3									USART3
#define ROS_SERIAL3_IRQ							USART3_IRQn
#define ROS_SERIAL3_CLK           	RCC_APB1Periph_USART3
#define ROS_SERIAL3_GPIO_CLK       	RCC_APB2Periph_GPIOB
#define ROS_SERIAL3_GPIO_PORT      	GPIOB
#define ROS_SERIAL3_TX_PIN         	GPIO_Pin_10
#define ROS_SERIAL3_RX_PIN          GPIO_Pin_11
#define ROS_SERIAL3_NVIC						3
///////////////////////////////////////////////////////
#endif // _CONFIG_H_