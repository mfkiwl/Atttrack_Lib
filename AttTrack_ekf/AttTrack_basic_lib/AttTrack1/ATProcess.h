#pragma once
#pragma once
#include "AttInit.h"
#include "DataStruct.h"
#include "ComFunc.h"
#include "kalmanfilter.h"
#include "config.h"
#include "AttTrack_lib.h"
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
};
typedef struct ATProcessSingleAngle ATProcessSingleAngle_t;

void ProcessSingleAngle(struct ATProcessSingleAngle* atp);
void ATPinit(struct ATProcessSingleAngle* atp);
int process_gasensor(struct AngleTrackData* iatd, struct Config* cfg, struct ATProcessSingleAngle* atp, double ExterAngle);
int process_AttTrack(struct ATProcessSingleAngle* atp, struct Config* cfg,double gyo, double IMU_time, double ExterAngle);
