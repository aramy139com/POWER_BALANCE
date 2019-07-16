#include "stm32f10x.h"
#include <stdio.h>
#include "hardwareserial.h"
#include <ros.h>
#include <riki_msgs/Velocities.h>
#include <geometry_msgs/Twist.h>
#include <riki_msgs/PID.h>
#include <riki_msgs/Imu.h>
#include <riki_msgs/Battery.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include "balance.h"

BalanceCar balcar;
HardwareSerial serial;
int main() {
    uint32_t carImuTime=0,carCtlTime = 0,publishBatTime=0,debugTime=0;
    SystemInit();
    initialise();
    serial.begin(115200);
    delay(100);
    balcar.initialize();
    while(1) {
        //定时读取 imu信息，用以控制姿态  同时读取码盘信息
        if((millis()-carImuTime)>=(1000/CAR_IMU_RATE)) {
            carImuTime=millis();
            balcar.flushImuInfo();						//读取mpu6050 信息
            balcar.countCarPos();							//通过传感器计算小车角度
						balcar.flushEncodeInfo();					//读取码盘信息
        }
				if((millis()-carCtlTime)>=(1000/CAR_CTL_RATE)) {           
            carCtlTime=millis();						
						balcar.controlCarStand();
        }

        //定时读取电池电量
        if((millis()-publishBatTime)>=(1000/BAT_PUBLISH_RATE)) {
            //serial.print("%.2f \n",bat);
            publishBatTime=millis();
            balcar.getBatPower();
        }

    }
}
