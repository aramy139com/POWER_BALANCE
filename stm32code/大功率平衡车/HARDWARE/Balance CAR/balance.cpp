//filename balance.cpp
#include "balance.h"
#include "config.h"
#include "millisecondtimer.h"
#include "hardwareserial.h"
extern HardwareSerial serial;

double y1;				//二阶滤波
float angle_dot; 	
float Q_angle=0.001;// 过程噪声的协方差
float Q_gyro=0.003;//0.03 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle=0.5;// 测量噪声的协方差 既测量偏差              
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

//float standpid[5]={71.88,8.05,-565.5,-740.0,2.25};						//站立用的环 前两个：角度环 P,D, 后两个 速度环 P,D
float standpid[5]={75.88,9.2,0,0,0};	
void BalanceCar::initialize(){
	//设置mpu6050的偏移量，实测得来的值
	short offset[6]={-343,-54,-33,0,0,0};	
	//灯
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	
	ledred= Gpio(PC,13);
	ledgreen= Gpio(PC,14);
	ledwarn=Gpio(PA,15);
	beep=Gpio(PB,3);
	ledred.high();
	ledgreen.high();
	ledwarn.high();
	beep.high();								//鸣笛
	delay(50);
	
	//初始化传感器
	mpu.setAcc_fsr(ACC16G);			//加速度传感器 量程
	mpu.setGyrofsr(GYRO2000);		//陀螺仪传感器 量程  角速度
	//设置mpu6050的偏移量，实测得来的值
	mpu.setOffset(offset);
	mpu.mpu6050_init();
	mpu.setOrientation(1,-1,1);
	
	if( MAGNETOMETER_ISIN ){			//磁力计的初始化
		mpu.hmc5883l_init();
	}
	
	delay(50);
	//编码器
	leftencode.initialize(LEFT);
	rightencode.initialize(RIGHT);	
	//轮子控制
	leftwheel.initialize(LEFT,499,3);
	rightwheel.initialize(RIGHT);
	
	//电池初始化
	batpower.initialize(12.0,11.6,14.0);
	
	//pid初始化
	//anglepid.initialize(-999, 999, ANGLE_P, ANGLE_I, ANGLE_D);
	//placepid.initialize(-999, 999, PLACE_P, PLACE_I, PLACE_D);	
	
	actualLineSpeed=0.0;				//实际的线速度  米/秒
	expectLineSpeed=0.0;				//预期的线速度  米/秒
	actualAngleSpeed=0.0;				//实际的角速度	弧度/秒
	expectAngleSpeed=0.0;				//预期的角速度	弧度/秒
	headingRadians=0.0;					//小车的指向  弧度角  0~2pi
	stat=true;
	prev_update_time=millis();
	beep.low();									//关闭蜂鸣器
}

//获得加速度、陀螺仪传感器的值
void BalanceCar::flushImuInfo(){
	float fbuf[3];
	uint8_t* buf;
	//处理加速度
	mpu.getAccAllVal(fbuf,G);	
	raw_imu_msg.linear_acceleration.x=fbuf[0];
	raw_imu_msg.linear_acceleration.y=fbuf[1];
	raw_imu_msg.linear_acceleration.z=fbuf[2];
	//处理角速度
	mpu.getGyroAllVal(fbuf,ANGLE);
	raw_imu_msg.angular_velocity.x=fbuf[0];
	raw_imu_msg.angular_velocity.y=fbuf[1];
	raw_imu_msg.angular_velocity.z=fbuf[2];
	//处理磁力计
	if( MAGNETOMETER_ISIN ){
		mpu.getAllMagnetometer(fbuf,1);
		raw_imu_msg.magnetic_field.x=fbuf[0];
		raw_imu_msg.magnetic_field.z=fbuf[1];
		raw_imu_msg.magnetic_field.y=fbuf[2];		
		//计算小车的指向  假定小车与地面平行 只计算  X Y
		headingRadians=atan2(fbuf[2],fbuf[0]);
		if(headingRadians<0) headingRadians+=2*PI;
	}
}
//更新编码器信息 
void BalanceCar::flushEncodeInfo(){
	double offtime,leftspeed,rightspeed;								//流逝的时间  车轮的速度 米/秒
	//获得码盘信息
	leftencode.setEncoder();
	rightencode.setEncoder();	
	//这里使用的是 IMU的间隔读取码盘信息，直接使用IMU间隔时间
	offtime=(double)1.0/CAR_IMU_RATE;			//秒	
	//计算当前的小车的速度
	leftspeed =WHEEL_PERIMETER*(double)leftencode.getEnValue()/COUNTS_PER_REV/offtime;			//单位：米/秒
	rightspeed=WHEEL_PERIMETER*(double)rightencode.getEnValue()/COUNTS_PER_REV/offtime;
	leftwheel.setSpeed(leftspeed);						//将计算出的轮子速度，写回轮子
	rightwheel.setSpeed(rightspeed);
	actualLineSpeed=actualLineSpeed*0.3+0.7*((leftspeed+rightspeed)/2.0);									//小车当前的 线速度  增加一个 低通滤波功能
	displace=WHEEL_PERIMETER*(float)(leftencode.getTotleValue()+rightencode.getTotleValue())/2.0/COUNTS_PER_REV;
}

//获得电池电量信息
float BalanceCar::getBatPower(){
	return batpower.get_volt();
}

//显示小车当前状态信息
void BalanceCar::dispDebugInfo(char *buf){
	//sprintf(buf,"\nIMUZ:%.4fexp:[%.4f,%.4f],act:[%.4f,%.4f]\tbat=%.1f",raw_imu_msg.angular_velocity.z,expectLineSpeed,expectAngleSpeed,actualLineSpeed,actualAngleSpeed,getBatPower());
	//sprintf(buf,"\nIMUZ:%.3f, exp:[%.4f,%.4f],act:[%.4f,%.4f]\tbat=%.1f",raw_imu_msg.angular_velocity.z,expectLineSpeed,expectAngleSpeed,actualLineSpeed,actualAngleSpeed,getBatPower());
	//显示 imu相关信息   9轴信息
	
	//sprintf(buf,"IMU:[%.4f,%.4f,%.4f],[%.4f,%.4f,%.4f],[%.4f,%.4f,%.4f]\n",raw_imu_msg.linear_acceleration.x,raw_imu_msg.linear_acceleration.y,raw_imu_msg.linear_acceleration.z,raw_imu_msg.angular_velocity.x,raw_imu_msg.angular_velocity.y,raw_imu_msg.angular_velocity.z,raw_imu_msg.magnetic_field.x,raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.z);
	//sprintf(buf,"%ld:[%.4f,%.4f,%.4f]\t%.2f\n",millis(),raw_imu_msg.magnetic_field.x,raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.z,atan2(raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.x)*(180/PI)+180);
	sprintf(buf,"%ld:[%.4f,%.4f,%.4f]\t%.2f\n",millis(),raw_imu_msg.magnetic_field.x,raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.z,headingRadians*(180/PI));
	//显示速度相关信息
	//sprintf(buf,"%ld:  %.4f\t[%.4f,%.4f]    [%.2f,%.2f]\n",millis(),expectLineSpeed,leftwheel.getSpeed(),rightwheel.getSpeed(),raw_imu_msg.angular_velocity.z,atan2(raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.x)*(180/PI)+180);
}

//通过imu获得的信息，计算小车的姿态信息
void BalanceCar::countCarPos(){
	float gyro;		//角度  依据加速度 与重力加速度的夹角 计算出来的角度      角速度
	//-------加速度--------------------------
  //角度较小时，x=sinx得到角度（弧度）, deg = rad*180/3.14
  //angle=g_fAccel_y*57.29578/16384.00;	 //去除零点偏移,计算得到角度（弧度转换为度,）
  ori_angle=atan2(raw_imu_msg.linear_acceleration.x,raw_imu_msg.linear_acceleration.z)*57.29578+CAR_ZERO_ANGLE;		//通过加速度 计算小车姿态角度
	//滤波计算角度
	//yijieHubuFilt();
	erjieHubuFilt();
	//Kalman_Filter();
	//计算角速度
	actualAngleSpeed=raw_imu_msg.angular_velocity.y;			//角速度  度/秒
	//如果当前倾角过大，则 停止小车的运动
	if(flit_angle>FAILANGLE || (-1)*flit_angle>FAILANGLE){
		ledwarn.low();
		setStat(false);
	}
}

//滤波函数 
void BalanceCar::yijieHubuFilt(){		//一阶互补滤波
	float dt=1.00/CAR_IMU_RATE;			//每次读取时间间隔
	//serial.print("%.3f , %.3f , %.3f , %f\n",ori_angle,flit_angle,raw_imu_msg.angular_velocity.y,dt);
	flit_angle = K1 * ori_angle + (1-K1) * (flit_angle  +	raw_imu_msg.angular_velocity.y*dt);	//置信角速度与时间的积	
}

void BalanceCar::erjieHubuFilt(){		//二阶互补滤波
	 double x1,x2,dt=1.00/CAR_IMU_RATE;
	 x1=(ori_angle- flit_angle)*(1-K2)*(1-K2);
   y1=y1+x1*dt;
   x2=y1+2*(1-K2)*(ori_angle-flit_angle)+raw_imu_msg.angular_velocity.y;
   flit_angle=flit_angle+x2*dt;
	 //serial.print("%f\t%f\t%f\n",x1,y1,x2,flit_angle);
}
/**************************************************************************
函数功能：简易卡尔曼滤波
入口参数：加速度、角速度
返回  值：无
Accel 与地面的夹角  Gyro角速度        ori_angle 与地面夹角  currangelspeed
void Kalman_Filter(float Accel,float Gyro)	
**************************************************************************/
void BalanceCar::Kalman_Filter(){
	float dt=1.00/CAR_IMU_RATE;
	flit_angle+=(raw_imu_msg.angular_velocity.y - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = ori_angle - flit_angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	flit_angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = raw_imu_msg.angular_velocity.y - Q_bias;	 //输出值(后验估计)的微分=角速度
}
//小车姿态控制，控制小车 站立
void BalanceCar::controlCarStand(){
	short pwm=0,pwm_turn=0;
	if(stat==false) {
		leftwheel.setPWM(0);
		rightwheel.setPWM(0);
		return ;				//小车状态不正常 
	}
	//小车为正常状态了  计算需要输出的 pwm值
	ledwarn.high();                  //关闭警告灯	
	pwm=flit_angle*standpid[0]+raw_imu_msg.angular_velocity.y*standpid[1];		//角度环 角度*P+角速度*D
	pwm+=actualLineSpeed*standpid[2]+displace*standpid[3];				//速度环 和位移环 速度*P+位移*D
	//serial.print("%f , %d\n",displace,pwm);
	leftwheel.setPWM(pwm-actualAngleSpeed*standpid[4]);
	rightwheel.setPWM(pwm+actualAngleSpeed*standpid[4]);
}
