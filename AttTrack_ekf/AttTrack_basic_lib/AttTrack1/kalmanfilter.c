#include "kalmanfilter.h"

void kalmanfilterinit(struct kalmanfilter kft, int nrow, int ncol)
{
	//kft.nox = nrow;
	//kft.noz = ncol;
	//for (int i = 0;i<2;i++)
	//{
	//	kft->xk[i][0] = 0;
	//	kft->Pxk[i][0] = 0;			kft->Pxk[i][1] = 0;

	//	kft->Phi[i][0] = 0;			kft->Phi[i][1] = 0;

	//	kft->Qk[i][0] = 0;		  kft->Qk[i][1] = 0;
	//}
	//kft->Hk[0][0] = 0;			kft->Hk[0][1] = 0;
	//kft->Rk[0][0]= 0;
	//kft->zk[0][0] = 0;
	//InitialMatrix_xk(&kft);
	//InitialMatrix_Pxk(&kft);
	//InitialMatrix_Phi(&kft);
	//InitialMatrix_Qk(&kft);
	//InitialMatrix_Hk(&kft);
	//InitialMatrix_Rk(&kft);
	//InitialMatrix_zk(&kft);
	//printf("data=%f,%f\n", kft.xk[0][0], kft.xk[1][0]);
}



int setxk(struct kalmanfilter* kft,double xk0[])
{
	for (int i = 0; i <nox; i++)
	{
		kft->xk[i][0] = xk0[i];
	}
	return 0;
}

int setPxk(struct kalmanfilter* kft,double dpxk0[])
{	
	for (int i = 0; i <nox; i++)
	{
		kft->Pxk[i][i]= dpxk0[i] * dpxk0[i];
	}
	return 0;
}

int setPhi(struct kalmanfilter* kft,double dt)
{
	kft->Phi[0][0] = 1;
	kft->Phi[0][1] = dt;
	kft->Phi[1][0] = 0;
	kft->Phi[1][1] = 1;
	return 0;
}


int setQk(struct kalmanfilter* kft, double Qnoise[])
{
	for (int i = 0; i <nox; i++)
	{
		kft->Qk[i][i] = Qnoise[i] * Qnoise[i];
	}
	return 0;
}

int setHk(struct kalmanfilter* kft, int num, int option)
{

			kft->Hk[0][0] = 1;
			kft->Hk[0][1] = 0;

	return 0;
}

int setRk(struct kalmanfilter* kft, double Rnoise[], int num)
{
	for (int i = 0; i < noz; i++)
	{
		kft->Rk[i][i] = Rnoise[i] * Rnoise[i];
	}
	return 0;
}

int setzk(struct kalmanfilter* kft, double zki[])
{
	for (int i = 0; i < noz; i++)
	{
		kft->zk[i][0] = zki[i];
	}
	return 0;
}

int TUpdate(struct kalmanfilter* kft, double dt)
{
	//xk = Phi*xk;
		double  xk_temp[2] = { 0 };
		double  Pxk_temp[2][2] = { 0 };
	for (int i = 0;i < nox;i++)
	{
		xk_temp[i] = kft->Phi[i][0] * kft->xk[0][0] + kft->Phi[i][1] * kft->xk[1][0];
	}

	for (int i = 0;i < nox;i++)
	{
		kft->xk[i][0] = xk_temp[i];
	}
	//Pxk = Phi*Pxk*(Phi.t()) + Qk*dt;
	//printf("xk_temp:%f,%f\n", xk_temp[0], xk_temp[1]);

	for (int i = 0;i <  nox;i++)
	{
			Pxk_temp[i][0] = kft->Phi[i][0] * kft->Pxk[0][0]+ kft->Phi[i][1] * kft->Pxk[1][0];
			Pxk_temp[i][1] = kft->Phi[i][0] * kft->Pxk[0][1] + kft->Phi[i][1] * kft->Pxk[1][1];
	}



	for (int i = 0;i < nox;i++)
	{
		kft->Pxk[i][0] =  Pxk_temp[i][0]* kft->Phi[0][0] +  Pxk_temp[i][1]* kft->Phi[0][1]+ kft->Qk[i][0]*dt;
		kft->Pxk[i][1] = Pxk_temp[i][0] * kft->Phi[1][0] + Pxk_temp[i][1] * kft->Phi[1][1]+ kft->Qk[i][1] * dt;
	}
	return 0;
}

int MUpdate(struct kalmanfilter* kft, double Rnoise[], double zki[])
{
	//mat xk_1 = xk;
	double xk_1[2] = {0};
	for (int i = 0;i <  nox;i++)
	{
		xk_1[i] = kft->xk[i][0];
	}
		
	//mat Pxk_1 = Pxk;
	double Pxk_1[2][2] = {0};
	for (int i = 0;i < nox;i++)
	{
		Pxk_1[i][0] = kft->Pxk[i][0];
		Pxk_1[i][1] = kft->Pxk[i][1];
	}
	//mat PHt = Pxk_1*(Hk.t());
	double PHt[2][1] = {0};
	for (int i = 0;i < nox;i++)
	{
		PHt[i][0] = Pxk_1[i][0]* kft->Hk[0][0]+ Pxk_1[i][1] * kft->Hk[0][1];
	}
	//mat HPHtR = Hk*PHt + Rk;
	 double HPHtR=0;
		HPHtR= kft->Hk[0][0] * PHt[0][0] + kft->Hk[0][1] * PHt[1][0] + kft->Rk[0][0];

	
	//mat Kk = PHt*inv(HPHtR);
		double Kk[2] = {0};
		Kk[0] = PHt[0][0] *(1/ HPHtR);//∂‡Œ¨æÿ’Û«ÛƒÊ£ø
		Kk[1] = PHt[1][0] * (1 / HPHtR);

	//// test
	//mat kxk = Kk*(zk - Hk*xk_1);

	//xk = xk_1 + Kk*(zk - Hk*xk_1);
		for (int i = 0;i <  nox;i++)
		{
			kft->xk[i][0] = xk_1[i] + Kk[i]*(kft->zk[0][0] - (kft->Hk[0][0] * xk_1[0] + kft->Hk[0][1] * xk_1[1]));
		}


	//Pxk = Pxk_1 - Kk*HPHtR*(Kk.t());
		for (int i = 0;i < nox;i++)
		{
			kft->Pxk[i][0] = Pxk_1[i][0] - Kk[i] * HPHtR*Kk[0];
			kft->Pxk[i][1] = Pxk_1[i][1] - Kk[i] * HPHtR*Kk[1];

		}
	//Pxk = (Pxk + Pxk.t())*0.5;

		double Pxk_2[2][2] = {0};
		for (int i = 0;i <  nox;i++)
		{
			Pxk_2[i][0] = kft->Pxk[i][0];
			Pxk_2[i][1] = kft->Pxk[i][1];
		}
		for (int i = 0;i <  nox;i++)
		{
			kft->Pxk[i][0] = (Pxk_2[i][0] + Pxk_2[0][i]) / 2;
			kft->Pxk[i][1] = (Pxk_2[i][1] + Pxk_2[1][i]) / 2;
		}
	return 0;
}

