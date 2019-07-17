#pragma once
#pragma once
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
//#include <vector>
//#include <algorithm.h> 

//using namespace std;


#define RE_WGS84    6378137.0           // earth semimajor axis (WGS84) (m)
#define FE_WGS84    (1.0/298.257223563) // earth flattening (WGS84)
#define GN          9.80665
#define PI		    3.14159265358979
#define D2R         (PI/180.0)          /* deg to rad */
#define R2D         (180.0/PI)          /* rad to deg */
#define MAXVAL		30                  // max value for spilting
#define MAXLEN		1024
#define WinLen      5                // ���ٶȿ���ƽ���Ķ��д�С
#define WinLen_pitch 50             //���ٶȼ��㸩���Ǽ���std���ڴ�С

/*************************log******************************/
extern void fileopen(const char *file); //·��
extern void fileclose(void);
extern void outfile(int option, const char *format, ...);

/*************************�ַ�������***********************/
extern void Str2Array(const char *str, const char *sep, double *val);
extern int checkstr(const char* str, const char* substr);
//extern double GetAveStd(vector<double> a, int opt);
//extern double Getsum(vector<double> a);

/*************************����ת��*************************/
extern void ecef2pos(const double *r, double *pos); //xyz2blh
extern void pos2ecef(const double *pos, double *r); //blh2xyz
extern void xyz2enu(const double *pos, double *E);  //blh2 Cxyz2enu
extern void ecef2enu(const double *pos, const double *r, double *e); //xyz����ϵ����rת����enu
extern void enu2ecef(const double *pos, const double *e, double *r); //enu����ϵ����rת����xyz
																	 /*************************��������*************************/
extern void matmul(const char *tr, int n, int k, int m, double alpha,
	const double *A, const double *B, double beta, double *C);
extern double dot(const double *a, const double *b, int n);
//c[m][n]=a[m][n]+b[m][n]
void Maddn(double *a, double *b, double *c, int m, int n);
void Mminn(double *a, double *b, double *c, int m, int n);
//c[m][k]=a[m][n]*b[n][k]
void Mmulnm(double *a, double *b, int m, int n, int k, double *c);
//a[m][n]=a[m][k]*b
void Mmul(double *a, int m, int n, double b);
//a=aT
void Mtn(double *a, int m, int n, double *b);
//N[m][n]=M[m][n]
void Mequalm(double *M, int m, int n, double *N);

/*************************��̬ת��*************************/
void askew(double v[], double m[]);
//�������Ҿ���ת��Ԫ��ned
void m2qua_ned(double m[], double q[]);
//��Ԫ��ת�������Ҿ���ned
void q2mat_ned(double qua[], double m[]);
//�������Ҿ���תŷ����ned
void m2att_ned(double m[], double a[]);
//void m2att_ned(double m[], double* roll, double* pitch, double* yaw);
//ŷ����ת�������Ҿ���ned
void a2mat_ned(double att[], double m[]);
//void a2mat_ned(double roll, double pitch, double yaw, double m[]);
void qupdt(double qnb0[], double rv_nb[]);
//ŷ����תˮƽ��б��
void att2tilt(double att[], double* roll, double* pitch);

/*************************ˮƽ��װ���*********************/
//int Cal_InstallErr_Acc(vector<double> vaccx, vector<double> vaccy, vector<double> vaccz, double Install_Acc[2]);
//void Comp_InstallErr_Acc(double acc[3], double Install_Acc[2]);

/*************************�۸˴���*************************/
/* calcuate the arm lever in NED*/
/* argin: att: attitude-roll,pitch,yaw, rad
lever_b: arm lever in b, m
argout: lever_enu: arm lever in NED, m*/
/* -----------by dhf,2017.08.11----------*/
extern void cal_lever_ned(double att[3], double lever_b[3], double lever_ned[3]);

/* lever compensation from blh_ant to blh_ground*/
/* argin: blh_ant: the BLH of antenna, rad/rad/m
lever_enu: arm lever in enu, m
argout: blh_target: the BLH of target point, rad/rad/m*/
/* -----------by dhf,2017.08.11----------*/
extern void lever_compen(double blh_ant[3], double lever_ned[3], double blh_target[3], int option );

/* tilt measuring��б��������*/
/* argin: acc: m/s2
mag: Gauss
blh_ant: the BLH of antenna, rad/rad/m
lever: the length of arm lever , m
argout: blh_ground: the BLH of ground point, rad/rad/m*/
/* -----------by dhf,2017.08.11----------*/
extern void titl_compen(double att[3], double blh_ant[3], double lever_b[3], double blh_target[3]);

/********************����ƽ��********************/
//double FilterQueue(double NEW_DATA, double *QUEUE, char n);
//ȥ�����Сֵ
//double FilterVector(double NEW_DATA, vector<double>& QUEUE, int& count, char n);
//��ȥ�����Сֵ
//double FilterVector2(double NEW_DATA, vector<double>& QUEUE, int& count, char n);