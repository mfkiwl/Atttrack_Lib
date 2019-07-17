#pragma once
#include "stdio.h"
#define  FILE_SAVE_PATH "E:\\result\\"
#define  ATTITUDE_TRACK_CONFIG_PATH  "E:\\result\\AttitudeTrack.cfg"
#define nox 2  //状态矩阵行数
#define noz 1  //状态矩阵列数
#ifdef __cplusplus
extern "C"
{
#endif
// 惯性观测数据
struct IMUdata
{
	double imutimetarget;   //IMU输出的时间戳，秒/毫秒，100Hz，计算采样时间间隔
	double accx;            //加速度计X轴输出   g
	double accy;            //        Y         g
	double accz;            //        Z         g
	double gyox;            //陀螺X轴输出，     deg/s
	double gyoy;            //    Y             deg/s
	double gyoz;            //    Z             deg/s
	int state_acc;          //加速度计超量程状态位
};
typedef struct IMUdata IMUdata_t;
// 电子罗盘观测数据
struct Compassdata
{
	int comass_type;        //1 平面电子罗盘  2 三维电子罗盘
	double accx;            //加速度计X轴输出   g
	double accy;            //        Y         g
	double accz;            //        Z         g
	double mx;              //磁力计X轴输出，     
	double my;              //      Y             
	double mz;              //      Z          
};
typedef struct Compassdata Compassdata_t;
//GNSS观测数据
struct GNSSdata
{
	double gnsstimetarget;   //GNSS观测的时间戳，周内秒，20Hz
	double lat;              //纬度, WGS84       deg
	double lon;              //经度，WGS84       deg
	double alt;              //高程，WGS84        m
	double gnssyaw;          //双天线航向，北偏东(-180 - 180)      deg
	double gnss_v[3];        //gnss速度，ecef/enu   m/s
	double speed;            //地速              m/s
	double heading;          //双天线航向        rad
	double speed_ver;        //高程速度          m/s
	int state_pos;           //GNSS位置解状态
	int state_yaw;           //GNSS航向解状态
	int heading_flag;        //gnss航向类型  0.双天线 1.单天线
};
typedef struct GNSSdata GNSSdata_t;
//姿态跟踪观测数据
struct ATdata
{
	IMUdata_t imu;           //IMU观测数据
	GNSSdata_t gnss;         //GNSS观测数据
	Compassdata_t compass;	 //compass观测数据
	int Angle_flag;          //1.单轴角度跟踪 2.双轴角度跟踪 3.三维角度跟踪
	int bgnss_updata;        //GNSS观测数据更新标志： 0-未更新 1-更新
	int bcompass_updata;     //compass观测数据更新标志 ： 0-未更新 1-更新
};
typedef struct ATdata ATdata_t;

struct Result
{
	double roll;
	double pitch;
	double heading;
};
typedef struct Result result_t;

/******************************EKF角度跟踪通用接口************************************/
int  AngleTrack_Process(ATdata_t* atdata, result_t* res);//通用接口，输出角度可选,需要配置



/******************************单角度跟踪接口************************************/
int  AngleTrack_Process_UEniaxialAngle(IMUdata_t* imu,double ExterAngle, double* result);//陀螺仪与外部角度融合，输出单轴角度
int  AngleTrack_Process_UniaxialAngle(IMUdata_t* imu,  double* result);//陀螺仪与加速度计融合，输出单轴角度
/******************************双轴角度跟踪接口************************************/
int  AngleTrack_Process_DualAngle(IMUdata_t* imu, double result[2]);//IMU姿态跟踪，输出俯仰、横滚角
/******************************三维角度跟踪接口************************************/
int  AngleTrack_Process_3DAngle(IMUdata_t* imu,double ExterHeading,double result[3]);//IMU与外部航向角融合，输出俯仰、横滚、航向角
int  GnssHeading_Process(GNSSdata_t* gnss, double *result);//单天线计算航向角
int  CompassHeading_Process(Compassdata_t* compass, double *result);//电子罗盘计算航向角
#ifdef __cplusplus
}
#endif