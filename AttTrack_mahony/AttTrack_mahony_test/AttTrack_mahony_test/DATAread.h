#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef __cplusplus
extern "C"
{
#endif

struct chardata
{
	int init_flag;
	double gyro_data[3];
	double acc_data[3] ;
	double bdf_data[3];
	double gnss_v[3];//vn ve vd  m/s
	double car_d_v[3];
	double imu_Ts;
	double gnss_pos[3];//lat lon alt rad
	double gnss_pitch;
	int gpsUpdate;
	double ga_time;
	double two_heading;
};
char imuchar[10];
char test[256];

char gv[128];
char gyro[218];
char ti[128];
int GAsensordataread1(struct chardata *charreaddata, FILE *file_fp);
void GAdataread(struct chardata *charreaddata, FILE *file_fp);
void GAdataread1(struct chardata *charreaddata, FILE *file_fp);
void IS203dataread(struct chardata *charreaddata, FILE *file_fp);
void IS203dataread1(struct chardata *charreaddata, FILE *file_fp);
void IMUstructinit(struct chardata *charreaddata);
double getmean(double inputdata[], int NUM);
void GAsensordataread(struct chardata *charreaddata, FILE *file_fp);
void BDFdataread(struct chardata *charreaddata, FILE *file_fp);
#ifdef __cplusplus
}
#endif