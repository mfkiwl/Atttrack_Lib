#pragma once
#define Kp 0.4
#define Ki 0.0001
struct IMUdata
{
	double imu_time;	//IMU��������  s
	double acc[3];		//���ٶ� g
	double gyo[3];		//���ٶ� deg/s
	int acc_flag;		//���ٶȳ�����״̬λ
};
typedef struct IMUdata IMUdata_t;
struct MAGdata
{
	double mag_data[3]; //�����������
	int mag_flag;		//���������� 1.ƽ��������� 2.��ά��������
};
typedef struct MAGdata MAGdata_t;
struct result
{
	double pitch;		//��������� deg
	double roll;		//�������� deg
	double heading;		//�������� deg
	double normacc;
};
typedef struct result result_t;
int Mahony_Process(IMUdata_t* imu, MAGdata_t* magn, result_t* res);
