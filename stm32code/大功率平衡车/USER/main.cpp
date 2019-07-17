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
    delay(10);
    balcar.initialize();		
    while(1) {
        //��ʱ��ȡ imu��Ϣ�����Կ�����̬  ͬʱ��ȡ������Ϣ
        if((millis()-carImuTime)>=(1000/CAR_IMU_RATE)) {
            carImuTime=millis();
            balcar.flushImuInfo();						//��ȡmpu6050 ��Ϣ
            balcar.countCarPos();							//ͨ������������С���Ƕ�
						balcar.flushEncodeInfo();					//��ȡ������Ϣ
        }
				if((millis()-carCtlTime)>=(1000/CAR_CTL_RATE)) {           
            carCtlTime=millis();						
						balcar.controlCarStand();
        }

        //��ʱ��ȡ��ص���
        if((millis()-publishBatTime)>=(1000/BAT_PUBLISH_RATE)) {
            //serial.print("%.2f \n",bat);
            publishBatTime=millis();
            //if(balcar.getBatPower()<11.5){		//��ѹ���ͣ��澯
							//balcar.beep.invert();
						//}
        }

    }
}
