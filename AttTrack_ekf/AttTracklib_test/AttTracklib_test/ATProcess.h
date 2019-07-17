#pragma once
#pragma once
#include "AttInit.h"
#include "DataStruct.h"
#include "ComFunc.h"
#include "kalmanfilter.h"
#include "config.h"
//#include "ElevationEstimate.h"
//#include "StaticDetect.h"

struct ATProcessSingleAngle
{
	//bool battinit;
	//bool bkfinit;
	//bool bprocessinit;
	int battinit;
	int bkfinit;
	int bprocessinit;
	double dt;             //IMU采样间隔
	double tpre;
	//NED
	double pitch;
	double roll;
	double integ_pitch;
	double heading;
	double gyobias[3];
	double accpre[3];
	double gyopre[3];
	int num_accnorm;        //加计模值小于阈值计数
	int num_gyonorm;        //陀螺模值小于阈值计数
	double qua[4];
	double Cb2n[9];
}ATProcessSingleAngle_t;

void ProcessSingleAngle(struct ATProcessSingleAngle* atp);
void ATPinit(struct ATProcessSingleAngle* atp);
int process_gasensor_D(struct AngleTrackData* iatd, struct AttTrackCfg* cfg,struct ATProcessSingleAngle* atp); //车体姿态处理
int process_gasensor_U(struct AngleTrackData* iatd, struct AttTrackCfg* cfg, struct ATProcessSingleAngle* atp); //大臂姿态处理
int process_gasensor_UE(struct AngleTrackData* iatd, struct AttTrackCfg* cfg, double ExterAngle, struct ATProcessSingleAngle* atp); //小臂姿态处理
int process_gasensor_3D(struct AngleTrackData* iatd, struct AttTrackCfg* cfg, double ExterHeading, struct ATProcessSingleAngle* atp); //挖斗姿态处理
