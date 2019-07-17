#pragma once
#pragma once
/***********阈值需要实际确定********************/
//#include <algorithm>
#include "ComFunc.h"
#include "config.h"
//using namespace std;
#define Staticbias_Wlen 200
struct AttInit
{
	double bias_gx , bias_gy, bias_gz;
	double std_gx, std_gy, std_gz;
	double std_ax, std_ay, std_az, std_az2;
	double mean_gpsyaw, std_gpsyaw;
	double att[3];
	double tilt[2];  //水平偏转角，竖直偏转
	int bfinshinit;  //初始化成功标志，以及错误状态标志

};
//typedef AttInit AttInit_t;
int process_singleangle(struct AttInit* aint, AttTrackCfg_t*cfg, double gyo[3], double acc1[3]);
int process_singleangle_D(struct AttInit* aint, double gyo[3], double acc1[3]);
int process_singleangle_U(struct AttInit* aint, double gyo[3], double acc1[3]);
int process_singleangle_UE(struct AttInit* aint, double gyo[3]);
int process_singleangle_3D(struct AttInit* aint, double gyo[3], double acc1[3]);
void attInit(struct AttInit* aint);

