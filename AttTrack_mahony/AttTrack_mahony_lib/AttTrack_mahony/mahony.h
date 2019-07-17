#pragma once
#include "AttTrack_lib.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define Staticbias_Wlen 200
#define acc_Wlen 200
#define PI		    3.14159265358979
#define D2R         (PI/180.0)          /* deg to rad */
#define R2D         (180.0/PI)          /* rad to deg */
#define GN          9.80665
struct MahonyProcess
{
	int mpinit;
	int accinit;
	int maginit;
	double pitch;
	double roll;
	double heading;
	double q[4];
	double sume[3];
	double Cb2n[9];
	double gyo_offset[3];
	double gyo_bias[3];
	double acc_err[3];
	double mag_err[3];
	double acc_norm[3];
	double mag_norm[3];
	double att[3];
	double am_err[3];
	double normacc;
	double temp_Ki;
	double temp_Kp;
};
typedef struct MahonyProcess mp_t;
void Accnorm(double acc[3], double accnorm[3]);
void mp_init(mp_t *mp);
void AccCn2b(double q[4], double acc_b[3]);
void Accerr(double acc[3], double acc_b[3],double accerr[3]);
void Magnorm(double mag[3], double magnorm[3]);
void MagCb2n(double mag[3], double q[4], double mag_n[3]);
void Magerr(double mag_n[3],double mag[3], double q[4],double magerr[3]);

void AccMag_err(double accerr[3], double magerr[3], double amerr[3]);
int PIcontrol(double gyo[3], double amerr[3], mp_t* mp);
int init_process(IMUdata_t* imu, MAGdata_t* mag, mp_t* mp);
int grobias(IMUdata_t* imu, mp_t* mp);
double acc_norm(double acc[]);