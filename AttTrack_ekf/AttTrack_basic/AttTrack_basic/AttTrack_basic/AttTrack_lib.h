#pragma once
#include "stdio.h"
#define  FILE_SAVE_PATH "E:\\result\\"  //数据存储路径
#define  ATTITUDE_TRACK_CONFIG_PATH  "E:\\result\\AttitudeTrack.cfg"  //配置文件存储路径
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
	int state_acc;          //IMU标志位，例如加速度计超量程状态位
};
typedef struct IMUdata IMUdata_t;
// 电子罗盘观测数据
struct Compassdata
{
	int comass_type;        //1 平面电子罗盘  2 三维电子罗盘
	double accx;            //加速度计X轴输出   g
	double accy;            //        Y         g
	double accz;            //        Z         g
	double mx;              //磁力计X轴输出，     高斯
	double my;              //      Y             高斯
	double mz;              //      Z             高斯
};
typedef struct Compassdata Compassdata_t;
//GNSS观测数据
struct GNSSdata
{
	double gnsstimetarget;   //GNSS观测的时间戳，周内秒，20Hz
	double lat;              //纬度, WGS84       deg
	double lon;              //经度，WGS84       deg
	double alt;              //高程，WGS84        m
	double gnss_v[3];        //gnss速度，ecef/enu   m/s
	double heading;          //双天线航向        deg
	int state_pos;           //GNSS位置解算状态
	int state_yaw;           //GNSS航向解状态
	int heading_flag;        //gnss航向类型  0.双天线 1.单天线
};
typedef struct GNSSdata GNSSdata_t;
//配置文件信息
struct Config
{
	double abiasx;           //加速度计x轴静态零偏  g
	double abiasy;           //加速度计y轴静态零偏  g
	double abiasz;           //加速度计z轴静态零偏  g
	double gstdxthr;         //陀螺仪静态阈值       deg/s
	double gstdythr;         //陀螺仪静态阈值       deg/s
	double gstdzthr;         //陀螺仪静态阈值       deg/s
	double astdxthr;         //加速度计静态阈值       g
	double astdythr;         //加速度计静态阈值       g
	double astdzthr;         //加速度计静态阈值       g
	double gnoisex;          //x轴角速度噪声         rad
	double gnoisey;          //y轴角速度噪声         rad
	double gnoisez;          //z轴角速度噪声         rad
	double gstabilityx;     //x轴陀螺仪零偏稳定性    rad/h
	double gstabilityy;     //x轴陀螺仪零偏稳定性    rad/h
	double gstabilityz;     //x轴陀螺仪零偏稳定性    rad/h
	double angle_err_init;      //角度误差初始值         rad
	double bias_err_init;       //零偏误差初始值         rad
	double gnoisestd;       //恒定量测噪声std        rad
	double gnssyawstd_thr;  //双天线航向角阈值       rad
	double gnssyawvar;      //双天线航向角量测噪声   rad
	double install_roll;    //横滚角安装误差         deg
	double install_pitch;   //俯仰角安装误差         deg
	double install_heading; //航向角安装误差         deg
	double acc_norm;        //干扰加速度
	int ekf_strategy;       //ekf量测噪声策略 1.加速度平滑滤波   2.动静态区分  3.其他策略
	int angle_flag;         //输出角度类型  1、IMU单轴角度跟踪 2、单轴陀螺+外部角度
};
typedef struct  Config Config_t;
//姿态跟踪观测数据
struct ATdata
{
	IMUdata_t imu;           //IMU观测数据
	GNSSdata_t gnss;         //GNSS观测数据
	Compassdata_t compass;	 //compass观测数据
	Config_t config;         //配置文件结构体
	int bimu_updta;          //IMU观测数据更新标志：0-未更新  1-更新
	int bgnss_updata;        //GNSS观测数据更新标志： 0-未更新 1-更新
	int bcompass_updata;     //compass观测数据更新标志 ： 0-未更新 1-更新
};
typedef struct ATdata ATdata_t;

int  AngleTrack_Process(Config_t* conf, double gyo,double IMU_time,double ExterAngle, double* result);//陀螺仪与外部角度融合，输出单轴角度



int AngleTrack_Process_Demo(ATdata_t *atd, Config_t* conf, double* result);
int  IMU_AccAngle_Process(IMUdata_t *imu, Config_t* conf, double *result);
int  GnssHeading_Process(GNSSdata_t* gnss, double *result);//单天线计算航向角
int  CompassHeading_Process(Compassdata_t* compass, double *result);//电子罗盘计算航向角
#ifdef __cplusplus
}
#endif