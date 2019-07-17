#include "Matrix.h"
void InitialMatrix_xk(struct kalmanfilter* kft)
{
	int m =nox;
	int n =noz;
	int i,j;
	kft->xk = (double**)malloc(m*sizeof(double*));//行分配内存
	for (i = 0;i < m;i++)
	{
		kft->xk[i] = (double*)malloc(n*sizeof(double));//列分配内存
	}

		for (i = 0;i<m;i++)
		{
			for (j = 0;j <n;j++)
			{
				kft->xk[i][0] =0;
			}
		}
}
void InitialMatrix_Pxk(struct kalmanfilter* kft)
{
	int m =nox;
	int n =noz;
	int i,j;
	kft->Pxk = (double**)malloc(m*sizeof(double*));
	for (i = 0;i<m;i++)
		kft->Pxk[i] = (double*)malloc(m*sizeof(double));
	for (i = 0;i<m;i++)
	{
		for (j = 0;j < m;j++)
		{
			kft->Pxk[i][j] = 0;
		}
	}
}
void InitialMatrix_Phi(struct kalmanfilter* kft)
{
	int m = nox;
	int n = noz;
	int i,j;
	kft->Phi = (double**)malloc(m*sizeof(double*));
	for (i = 0;i<2;i++)
		kft->Phi[i] = (double*)malloc(m*sizeof(double));
	for (i = 0;i<m;i++)
	{
		for (j = 0;j < m;j++)
		{
			kft->Phi[i][j] = 0;
		}
	}
}
void InitialMatrix_Qk(struct kalmanfilter* kft)
{
	int m = nox;
	int n = noz;
	int i,j;
	kft->Qk = (double**)malloc(m*sizeof(double*));
	for (i = 0;i<m;i++)
		kft->Qk[i] = (double*)malloc(m*sizeof(double));
	for (i = 0;i<m;i++)
	{
		for (j = 0;j < m;j++)
		{
			kft->Qk[i][j] = 0;
		}
	}
}
void InitialMatrix_Hk(struct kalmanfilter* kft)
{
	int m =nox;
	int n = noz;
	int i,j;
	kft->Hk = (double**)malloc(m*sizeof(double*));
	for (i = 0;i<m;i++)
		kft->Hk[i] = (double*)malloc(n*sizeof(double));
	for (i = 0;i<n;i++)
	{
		for (j = 0;j < m;j++)
		{
			kft->Hk[i][j] = 0;
		}
	}
}
void InitialMatrix_Rk(struct kalmanfilter* kft)
{
	int m = nox;
	int n = noz;
	int i,j;
	kft->Rk = (double**)malloc(n*sizeof(double*));
	for (i = 0;i<n;i++)
		kft->Rk[i] = (double*)malloc(n*sizeof(double));
	for (i = 0;i<n;i++)
	{
		for (j = 0;j < n;j++)
		{
			kft->Rk[i][j] = 0;
		}
	}
}
void InitialMatrix_zk(struct kalmanfilter* kft)
{
	int m = nox;
	int n = noz;
	int i;
	kft->zk = (double**)malloc(m*sizeof(double*));
	for (i = 0;i<m;i++)
		kft->zk[i] = (double*)malloc(n*sizeof(double));
	for (i = 0;i<n;i++)
	{
			kft->zk[i][0] = 0;
	}
}

//void PrintMatrix(Matrix *T)
//{
//	int i, j;
//	for (i = 0;i<(T->m);i++)
//	{
//		for (j = 0;j<(T->n);j++)
//			printf(" %3.0f", T->mat[i][j]);
//		printf("\n");
//	}
//}


