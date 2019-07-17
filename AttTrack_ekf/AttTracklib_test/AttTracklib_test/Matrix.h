#pragma once
#include "AttTrack_lib.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "kalmanfilter.h"
void InitialMatrix_xk(struct kalmanfilter* kft);
void InitialMatrix_Pxk(struct kalmanfilter* kft);
void InitialMatrix_Phi(struct kalmanfilter* kft);
void InitialMatrix_Qk(struct kalmanfilter* kft);
void InitialMatrix_Hk(struct kalmanfilter* kft);
void InitialMatrix_Rk(struct kalmanfilter* kft);
void InitialMatrix_zk(struct kalmanfilter* kft);
//void FreeMatrix(Matrix *T);
