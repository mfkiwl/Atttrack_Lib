#pragma once
#define Kp 0.4			//比例系数
#define Ki 0.0001		//积分系数
struct IMUdata
{
	double imu_time;	//IMU采样周期  s
	double acc[3];		//加速度 g
	double gyo[3];		//角速度 deg/s
	int acc_flag;		//加速度超量程状态位   0.未超量程  1.超量程
};
typedef struct IMUdata IMUdata_t;
struct MAGdata
{
	double mag_data[3]; //三轴磁力分量
	double acc[3];		//加速度分量
	int mag_flag;		//磁力计类型 1.平面电子罗盘 2.三维电子罗盘
};
typedef struct MAGdata MAGdata_t;
struct result
{
	double pitch;		//输出俯仰角 deg
	double roll;		//输出横滚角 deg
	double heading;		//输出航向角 deg
	double normacc;
};
typedef struct result result_t;
int Mahony_Process(IMUdata_t* imu, MAGdata_t* magn, result_t* res);
