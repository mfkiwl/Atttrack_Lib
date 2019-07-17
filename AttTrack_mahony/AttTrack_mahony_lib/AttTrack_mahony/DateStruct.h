#pragma once
#include "AttTrack_lib.h"
#include "mahony.h"
#define WinLen      50                // 加速度开窗平滑的队列大小
void a2mat_ned(double att[], double m[]);
void m2qua_ned(double m[], double q[]);
void qua2att(double q[], double att[]);
void Mmul(double *a, int m, int n, double b);
void Mtn(double *a, int m, int n, double *b);
void Mmulnm(double *a, double *b, int m, int n, int k, double *c);
void Mequalm(double *M, int m, int n, double *N);
void gyo2qua(double gyo[], double imu_time, double q[]);
void angle_change(double acc[], double att[]);
void acc_filter(double acc[], double smooth_acc[]);
double dataFilter(double acc_win[], int count);