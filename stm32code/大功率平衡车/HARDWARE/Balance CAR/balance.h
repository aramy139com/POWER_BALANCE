#ifndef _BALANCE_H_
#define _BALANCE_H_
#include "config.h"
#include "Gpio.h"
#include "mpu6050.h"
#include "encoder.h"
#include "battery.h"
#include "wheel.h"
#include "PID.h"
#include <riki_msgs/Imu.h>
#include <riki_msgs/Velocities.h>

union SerTran{  //���ڴ�����������
    char s[4];
    float f;
};  
 
//����ص���
class BalanceCar{
	private:
		//״̬��Ϣ
		Mpu6050 mpu;		//������
		riki_msgs::Imu raw_imu_msg;								//�����ǡ����ٶȡ�������
		//ƽ����Ϣ
		float ori_angle;			//С���Ӵ�������������� �Ƕ� ��ˮƽ��н� ��λ ��
		float flit_angle;			//�˲���ĽǶ�
		bool stat; //״̬   	false  ϵͳͣ��   true ϵͳ���� 
		
		Gpio ledwarn;		
		//���
		Battery batpower;
		
		//����
		Wheel leftwheel;
		Wheel rightwheel;		
		
		uint32_t prev_update_time;
		double actualLineSpeed;				//ʵ�ʵ����ٶ�  ��/��
		double expectLineSpeed;				//Ԥ�ڵ����ٶ�  ��/��  ��� +-0.25��/��
		double actualAngleSpeed;			//ʵ�ʵĽ��ٶ�	��/��		����Ľ��ٶ� ��ָ ˮƽ����Ľ��ٶȣ�����С��ת��ġ�
		double expectAngleSpeed;			//Ԥ�ڵĽ��ٶ�	��/��
		double headingRadians;				//С����ָ��	
		float displace;								//С����λ��  ��λ ��
		
		void yijieHubuFilt();		//һ�׻����˲�  
		void erjieHubuFilt();		//���׻����˲�  		
		void Kalman_Filter();		//�������˲�
	public:
		uint32_t commandTime;		//����ʱ��
		void initialize();			//��ʼ����������
		void flushImuInfo();		//�Ӵ������� ��� mpu6050 �����Ƶĵ�ǰֵ
		void countCarPos();			//����С����ǰ״̬ ���������� С������ǣ��ٶ�
		void checkMag();				//У��������
		//�ƹ�
		Gpio ledred;
		Gpio ledgreen;
		Gpio beep;		
		//������
		Encoder leftencode;
		Encoder rightencode;
		void flushEncodeInfo();	//���� ��������Ϣ
	
		//�����ֵ�PID
		//PID leftpid;
		//PID rightpid;
	
		void controlCarStand();		//����С��
		float getBatPower();	//��õ�ǰ����
		void dispDebugInfo(char *buf);			//��ʾС����ǰ״̬��Ϣ
		
	
		void batWarn(){				//��������
			ledwarn.invert();
		}
		void cmdWarn(){				//������ʾ
			ledred.invert();
		}
		double getActualLineSpeed(){			//��õ�ǰʵ�ʵ����ٶ�
			return actualLineSpeed;
		}
		double getExpectLineSpeed(){			//��õ�ǰ���������ٶ�
			return expectLineSpeed;
		}
		double getActualAngleSpeed(){
			return actualAngleSpeed;
		}
		double getExpectAngleSpeed(){
			return expectAngleSpeed;
		}
		void setExpectLineSpeed(double linespeed){
			expectLineSpeed=linespeed;
		}
		void setExpectAngleSpeed(double anglespeed){
			expectAngleSpeed=anglespeed;
		}
		riki_msgs::Imu getRaw_imu_msg(){
			return raw_imu_msg;
		}
		float getOri_angle(){
			return ori_angle;
		}
		float getFlit_angle(){
			return flit_angle;
		}
		bool getStat(){
		return stat;
		}
		void setStat(bool _stat){
			stat=_stat;
		}
//		PID getLeftpid(){return leftpid;}
//		PID getRightpid(){return rightpid;}
};

#endif // _ROSLCAR_H_