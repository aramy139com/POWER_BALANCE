//filename balance.cpp
#include "balance.h"
#include "config.h"
#include "millisecondtimer.h"
#include "hardwareserial.h"
extern HardwareSerial serial;

double y1;				//�����˲�
float angle_dot; 	
float Q_angle=0.001;// ����������Э����
float Q_gyro=0.003;//0.03 ����������Э���� ����������Э����Ϊһ��һ�����о���
float R_angle=0.5;// ����������Э���� �Ȳ���ƫ��              
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

float standpid[4]={70.0,9.23,-725.5,0.0};						//վ���õĻ� ǰ�������ǶȻ� P,D, ������ �ٶȻ� P,D
void BalanceCar::initialize(){
	//����mpu6050��ƫ������ʵ�������ֵ
	short offset[6]={-343,-54,-33,0,0,0};	
	//��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	
	ledred= Gpio(PC,13);
	ledgreen= Gpio(PC,14);
	ledwarn=Gpio(PA,15);
	beep=Gpio(PB,3);
	ledred.high();
	ledgreen.high();
	ledwarn.high();
	beep.high();								//����
	delay(50);
	
	//��ʼ��������
	mpu.setAcc_fsr(ACC16G);			//���ٶȴ����� ����
	mpu.setGyrofsr(GYRO2000);		//�����Ǵ����� ����  ���ٶ�
	//����mpu6050��ƫ������ʵ�������ֵ
	mpu.setOffset(offset);
	mpu.mpu6050_init();
	mpu.setOrientation(1,-1,1);
	
	if( MAGNETOMETER_ISIN ){			//�����Ƶĳ�ʼ��
		mpu.hmc5883l_init();
	}
	
	delay(50);
	//������
	leftencode.initialize(LEFT);
	rightencode.initialize(RIGHT);	
	//���ӿ���
	leftwheel.initialize(LEFT,499,3);
	rightwheel.initialize(RIGHT);
	
	//��س�ʼ��
	batpower.initialize(12.0,11.6,14.0);
	
	//pid��ʼ��
	//anglepid.initialize(-999, 999, ANGLE_P, ANGLE_I, ANGLE_D);
	//placepid.initialize(-999, 999, PLACE_P, PLACE_I, PLACE_D);	
	
	actualLineSpeed=0.0;				//ʵ�ʵ����ٶ�  ��/��
	expectLineSpeed=0.0;				//Ԥ�ڵ����ٶ�  ��/��
	actualAngleSpeed=0.0;				//ʵ�ʵĽ��ٶ�	����/��
	expectAngleSpeed=0.0;				//Ԥ�ڵĽ��ٶ�	����/��
	headingRadians=0.0;					//С����ָ��  ���Ƚ�  0~2pi
	stat=true;
	prev_update_time=millis();
	beep.low();									//�رշ�����
}

//��ü��ٶȡ������Ǵ�������ֵ
void BalanceCar::flushImuInfo(){
	float fbuf[3];
	uint8_t* buf;
	//������ٶ�
	mpu.getAccAllVal(fbuf,G);	
	raw_imu_msg.linear_acceleration.x=fbuf[0];
	raw_imu_msg.linear_acceleration.y=fbuf[1];
	raw_imu_msg.linear_acceleration.z=fbuf[2];
	//������ٶ�
	mpu.getGyroAllVal(fbuf,ANGLE);
	raw_imu_msg.angular_velocity.x=fbuf[0];
	raw_imu_msg.angular_velocity.y=fbuf[1];
	raw_imu_msg.angular_velocity.z=fbuf[2];
	//���������
	if( MAGNETOMETER_ISIN ){
		mpu.getAllMagnetometer(fbuf,1);
		raw_imu_msg.magnetic_field.x=fbuf[0];
		raw_imu_msg.magnetic_field.z=fbuf[1];
		raw_imu_msg.magnetic_field.y=fbuf[2];		
		//����С����ָ��  �ٶ�С�������ƽ�� ֻ����  X Y
		headingRadians=atan2(fbuf[2],fbuf[0]);
		if(headingRadians<0) headingRadians+=2*PI;
	}
}
//���±�������Ϣ 
void BalanceCar::flushEncodeInfo(){
	double offtime,leftspeed,rightspeed;								//���ŵ�ʱ��  ���ֵ��ٶ� ��/��
	//���������Ϣ
	leftencode.setEncoder();
	rightencode.setEncoder();	
	//����ʹ�õ��� IMU�ļ����ȡ������Ϣ��ֱ��ʹ��IMU���ʱ��
	offtime=(double)1.0/CAR_IMU_RATE;			//��	
	//���㵱ǰ��С�����ٶ�
	leftspeed =WHEEL_PERIMETER*(double)leftencode.getEnValue()/COUNTS_PER_REV/offtime;			//��λ����/��
	rightspeed=WHEEL_PERIMETER*(double)rightencode.getEnValue()/COUNTS_PER_REV/offtime;
	leftwheel.setSpeed(leftspeed);						//��������������ٶȣ�д������
	rightwheel.setSpeed(rightspeed);
	actualLineSpeed=(leftspeed+rightspeed)/2.0;									//С����ǰ�� ���ٶ�
	displace=WHEEL_PERIMETER*(float)(leftencode.getTotleValue()+rightencode.getTotleValue())/2.0/COUNTS_PER_REV;
}

//��õ�ص�����Ϣ
float BalanceCar::getBatPower(){
	return batpower.get_volt();
}

//��ʾС����ǰ״̬��Ϣ
void BalanceCar::dispDebugInfo(char *buf){
	//sprintf(buf,"\nIMUZ:%.4fexp:[%.4f,%.4f],act:[%.4f,%.4f]\tbat=%.1f",raw_imu_msg.angular_velocity.z,expectLineSpeed,expectAngleSpeed,actualLineSpeed,actualAngleSpeed,getBatPower());
	//sprintf(buf,"\nIMUZ:%.3f, exp:[%.4f,%.4f],act:[%.4f,%.4f]\tbat=%.1f",raw_imu_msg.angular_velocity.z,expectLineSpeed,expectAngleSpeed,actualLineSpeed,actualAngleSpeed,getBatPower());
	//��ʾ imu�����Ϣ   9����Ϣ
	
	//sprintf(buf,"IMU:[%.4f,%.4f,%.4f],[%.4f,%.4f,%.4f],[%.4f,%.4f,%.4f]\n",raw_imu_msg.linear_acceleration.x,raw_imu_msg.linear_acceleration.y,raw_imu_msg.linear_acceleration.z,raw_imu_msg.angular_velocity.x,raw_imu_msg.angular_velocity.y,raw_imu_msg.angular_velocity.z,raw_imu_msg.magnetic_field.x,raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.z);
	//sprintf(buf,"%ld:[%.4f,%.4f,%.4f]\t%.2f\n",millis(),raw_imu_msg.magnetic_field.x,raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.z,atan2(raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.x)*(180/PI)+180);
	sprintf(buf,"%ld:[%.4f,%.4f,%.4f]\t%.2f\n",millis(),raw_imu_msg.magnetic_field.x,raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.z,headingRadians*(180/PI));
	//��ʾ�ٶ������Ϣ
	//sprintf(buf,"%ld:  %.4f\t[%.4f,%.4f]    [%.2f,%.2f]\n",millis(),expectLineSpeed,leftwheel.getSpeed(),rightwheel.getSpeed(),raw_imu_msg.angular_velocity.z,atan2(raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.x)*(180/PI)+180);
}

//ͨ��imu��õ���Ϣ������С������̬��Ϣ
void BalanceCar::countCarPos(){
	float gyro;		//�Ƕ�  ���ݼ��ٶ� ���������ٶȵļн� ��������ĽǶ�      ���ٶ�
	//-------���ٶ�--------------------------
  //�ǶȽ�Сʱ��x=sinx�õ��Ƕȣ����ȣ�, deg = rad*180/3.14
  //angle=g_fAccel_y*57.29578/16384.00;	 //ȥ�����ƫ��,����õ��Ƕȣ�����ת��Ϊ��,��
  ori_angle=atan2(raw_imu_msg.linear_acceleration.x,raw_imu_msg.linear_acceleration.z)*57.29578+CAR_ZERO_ANGLE;		//ͨ�����ٶ� ����С����̬�Ƕ�
	//�˲�����Ƕ�
	//yijieHubuFilt();
	erjieHubuFilt();
	//Kalman_Filter();
	//������ٶ�
	actualAngleSpeed=raw_imu_msg.angular_velocity.y;			//���ٶ�  ��/��
	//�����ǰ��ǹ����� ֹͣС�����˶�
	if(flit_angle>FAILANGLE || (-1)*flit_angle>FAILANGLE){
		ledwarn.low();
		setStat(false);
	}
}

//�˲����� 
void BalanceCar::yijieHubuFilt(){		//һ�׻����˲�
	float dt=1.00/CAR_IMU_RATE;			//ÿ�ζ�ȡʱ����
	//serial.print("%.3f , %.3f , %.3f , %f\n",ori_angle,flit_angle,raw_imu_msg.angular_velocity.y,dt);
	flit_angle = K1 * ori_angle + (1-K1) * (flit_angle  +	raw_imu_msg.angular_velocity.y*dt);	//���Ž��ٶ���ʱ��Ļ�	
}

void BalanceCar::erjieHubuFilt(){		//���׻����˲�
	 double x1,x2,dt=1.00/CAR_IMU_RATE;
	 x1=(ori_angle- flit_angle)*(1-K2)*(1-K2);
   y1=y1+x1*dt;
   x2=y1+2*(1-K2)*(ori_angle-flit_angle)+raw_imu_msg.angular_velocity.y;
   flit_angle=flit_angle+x2*dt;
	 //serial.print("%f\t%f\t%f\n",x1,y1,x2,flit_angle);
}
/**************************************************************************
�������ܣ����׿������˲�
��ڲ��������ٶȡ����ٶ�
����  ֵ����
Accel �����ļн�  Gyro���ٶ�        ori_angle �����н�  currangelspeed
void Kalman_Filter(float Accel,float Gyro)	
**************************************************************************/
void BalanceCar::Kalman_Filter(){
	float dt=1.00/CAR_IMU_RATE;
	flit_angle+=(raw_imu_msg.angular_velocity.y - Q_bias) * dt; //�������
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = ori_angle - flit_angle;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	flit_angle	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	angle_dot   = raw_imu_msg.angular_velocity.y - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}
//С����̬���ƣ�����С�� վ��
void BalanceCar::controlCarStand(){
	short pwm=0,pwm_turn=0;
	if(stat==false) {
		leftwheel.setPWM(0);
		rightwheel.setPWM(0);
		return ;				//С��״̬������ 
	}
	//С��Ϊ����״̬��  ������Ҫ����� pwmֵ
	ledwarn.high();                  //�رվ����	
	pwm=flit_angle*standpid[0]+raw_imu_msg.angular_velocity.y*standpid[1];		//�ǶȻ� �Ƕ�*P+���ٶ�*D
	pwm+=actualLineSpeed*standpid[2]+displace*standpid[3];				//�ٶȻ� ��λ�ƻ� �ٶ�*P+λ��*D
	//serial.print("%d , %d , %d\n",leftencode.getEnValue(),rightencode.getEnValue(),pwm);
	leftwheel.setPWM(pwm);
	rightwheel.setPWM(pwm);
}
