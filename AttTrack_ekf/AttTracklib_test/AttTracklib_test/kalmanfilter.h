#pragma once
//#include "ComFunc.h"
//#include<stdlib.h>
//#include <armadillo>
#include <math.h>
#include "Matrix.h"

struct kalmanfilter
{
	//int nox;
	//int noz;
	//double xk[2][1];
	double **xk;
	//double Pxk[2][2];
	double **Pxk;
	//double Phi[2][2];
	double **Phi;
	//double Qk[2][2];
	double **Qk;
	//double Hk[1][2];
	double **Hk;
	//double Rk[1][1];
	double **Rk;
	//double zk[1][1];
	double **zk;
}kalmanfilter_t;

void kalmanfilterinit(struct kalmanfilter kft, int nrow, int ncol);
int setxk(struct kalmanfilter *kft, double xk0[]);
int setPxk(struct kalmanfilter* kft, double dpxk0[]);
int setPhi(struct kalmanfilter* kft, double dt);
int setQk(struct kalmanfilter* kft, double Qnoise[]);
int setHk(struct kalmanfilter* kft, int num, int option);
int setRk(struct kalmanfilter* kft, double Rnoise[],int num);
int setzk(struct kalmanfilter* kft, double zki[]);
int TUpdate(struct kalmanfilter* kft, double dt);
int MUpdate(struct kalmanfilter* kft, double Rnoise[], double zki[]);             
																				  
