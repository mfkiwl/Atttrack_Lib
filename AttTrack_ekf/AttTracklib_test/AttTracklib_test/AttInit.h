#pragma once
#pragma once
/***********��ֵ��Ҫʵ��ȷ��********************/
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
	double tilt[2];  //ˮƽƫת�ǣ���ֱƫת
	int bfinshinit;  //��ʼ���ɹ���־���Լ�����״̬��־

};
//typedef AttInit AttInit_t;
int process_singleangle(struct AttInit* aint, AttTrackCfg_t*cfg, double gyo[3], double acc1[3]);
int process_singleangle_D(struct AttInit* aint, double gyo[3], double acc1[3]);
int process_singleangle_U(struct AttInit* aint, double gyo[3], double acc1[3]);
int process_singleangle_UE(struct AttInit* aint, double gyo[3]);
int process_singleangle_3D(struct AttInit* aint, double gyo[3], double acc1[3]);
void attInit(struct AttInit* aint);

