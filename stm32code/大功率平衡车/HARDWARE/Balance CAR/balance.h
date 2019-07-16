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

union SerTran{  //串口传送数据是用
    char s[4];
    float f;
};  
 
//车相关的类
class BalanceCar{
	private:
		//状态信息
		Mpu6050 mpu;		//传感器
		riki_msgs::Imu raw_imu_msg;								//陀螺仪、加速度、磁力计
		//平衡信息
		float ori_angle;			//小车从传感器计算出来的 角度 与水平面夹角 单位 度
		float flit_angle;			//滤波后的角度
		bool stat; //状态   	false  系统停机   true 系统正常 
		
		Gpio ledwarn;
		Gpio beep;		
		
		//电池
		Battery batpower;
		
		//轮子
		Wheel leftwheel;
		Wheel rightwheel;		
		
		uint32_t prev_update_time;
		double actualLineSpeed;				//实际的线速度  米/秒
		double expectLineSpeed;				//预期的线速度  米/秒  最大 +-0.25米/秒
		double actualAngleSpeed;			//实际的角速度	度/秒		这里的角速度 是指 水平地面的角速度，控制小车转向的。
		double expectAngleSpeed;			//预期的角速度	度/秒
		double headingRadians;				//小车的指向	
		float displace;								//小车的位移  单位 米
		
		void yijieHubuFilt();		//一阶互补滤波  
		void erjieHubuFilt();		//二阶互补滤波  		
		void Kalman_Filter();		//卡夫曼滤波
	public:
		uint32_t commandTime;		//命令时长
		void initialize();			//初始化各个参数
		void flushImuInfo();		//从传感器中 获得 mpu6050 磁力计的当前值
		void countCarPos();			//计算小车当前状态 计算内容有 小车的倾角，速度
		void checkMag();				//校正磁力计
		//灯光
		Gpio ledred;
		Gpio ledgreen;
		//编码器
		Encoder leftencode;
		Encoder rightencode;
		void flushEncodeInfo();	//更新 编码器信息
	
		//左右轮的PID
		//PID leftpid;
		//PID rightpid;
	
		void controlCarStand();		//控制小车
		float getBatPower();	//获得当前电量
		void dispDebugInfo(char *buf);			//显示小车当前状态信息
		
	
		void batWarn(){				//电量警告
			ledwarn.invert();
		}
		void cmdWarn(){				//命令提示
			ledred.invert();
		}
		double getActualLineSpeed(){			//获得当前实际的线速度
			return actualLineSpeed;
		}
		double getExpectLineSpeed(){			//获得当前期望的线速度
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