
#pragma once
//#include "ComFunc.h"
#include "AttTrack_lib.h"
#include "config.h"
//定义导航系为NED，载体系为FRD
//单轴角度跟踪
//class AngleTrackData
//{
//public:
//	double imutime;            //时间戳，秒
//	double gyo3[3];            //XYZ陀螺角速度，deg/s，前右下
//	double acc3[3];            //XYZ加计线加速度，m/s2,前右下
//public:
//	void Init();
//	void Rest();
//	AngleTrackData& operator=(const AngleTrackData& atdata);
//};
struct AngleTrackData
{
	double imutime;            //时间戳，秒
	double gyo3[3];            //XYZ陀螺角速度，deg/s，前右下
	double acc3[3];            //XYZ加计线加速度，m/s2,前右下
};
void AngleTrackData_Init(struct AngleTrackData* atd);
void AngleTrackData_Rest(struct AngleTrackData* atd);

double dataFilter(double acc_win[], int count);
double getstd(double acc_win[], int count);
void decode_gasensor(IMUdata_t* atd, struct AngleTrackData* atdata);
void acc_bias(AttTrackCfg_t* cfg, IMUdata_t* atd, double accbias[]);