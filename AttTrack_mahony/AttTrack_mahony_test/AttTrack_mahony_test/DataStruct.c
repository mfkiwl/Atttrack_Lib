#include "DateStruct.h"
static double acc_x_win1[50] = { 0 };
static double acc_y_win1[50] = { 0 };
static double acc_z_win1[50] = { 0 };
static int  NUM_acc1 = 0;
static int chang_num = 0;
void a2mat_ned(double att[], double m[])
{
	double phi = att[0];
	double theta = att[1];
	double psi = att[2];
	double cpsi = cos(psi); double spsi = sin(psi);
	double cthe = cos(theta); double sthe = sin(theta);
	double cphi = cos(phi); double sphi = sin(phi);
	double C1[9] = { 0.0 };
	double C2[9] = { 0.0 };
	double C3[9] = { 0.0 };
	double m1[9] = { 0.0 };
	double m2[9] = { 0.0 };
	C1[0 * 3 + 0] = cpsi;  C1[0 * 3 + 1] = spsi;
	C1[1 * 3 + 0] = -spsi; C1[1 * 3 + 1] = cpsi;
	C1[2 * 3 + 2] = 1.0;
	C2[0 * 3 + 0] = cthe;                  C2[0 * 3 + 2] = -sthe;
	C2[1 * 3 + 1] = 1.0;
	C2[2 * 3 + 0] = sthe;                  C2[2 * 3 + 2] = cthe;
	C3[0 * 3 + 0] = 1.0;
	C3[1 * 3 + 1] = cphi; C3[1 * 3 + 2] = sphi;
	C3[2 * 3 + 1] = -sphi; C3[2 * 3 + 2] = cphi;
	Mmulnm(C3, C2, 3, 3, 3, m1);
	Mmulnm(m1, C1, 3, 3, 3, m2);
	Mtn(m2, 3, 3, m);
}
void m2qua_ned(double m[], double q[])
{
	double s[5] = { 0.0 };
	s[4] = m[0 * 3 + 0] + m[1 * 3 + 1] + m[2 * 3 + 2];
	s[0] = 1.0 + s[4];
	s[1] = 1.0 + 2.0*m[0 * 3 + 0] - s[4];
	s[2] = 1.0 + 2.0*m[1 * 3 + 1] - s[4];
	s[3] = 1.0 + 2.0*m[2 * 3 + 2] - s[4];
	int index = 0;
	double max = s[0];
	for (int k = 1; k<4; k++)
	{
		if (s[k]>max)
		{
			index = k;
			max = s[k];
		}
	}
	switch (index)
	{
	case 0:
		q[0] = 0.5*sqrt(s[0]);
		q[1] = 0.25*(m[2 * 3 + 1] - m[1 * 3 + 2]) / q[0];
		q[2] = 0.25*(m[0 * 3 + 2] - m[2 * 3 + 0]) / q[0];
		q[3] = 0.25*(m[1 * 3 + 0] - m[0 * 3 + 1]) / q[0];
		break;
	case 1:
		q[1] = 0.5*sqrt(s[1]);
		q[2] = 0.25*(m[1 * 3 + 0] + m[0 * 3 + 1]) / q[1];
		q[3] = 0.25*(m[0 * 3 + 2] + m[2 * 3 + 0]) / q[1];
		q[0] = 0.25*(m[2 * 3 + 1] - m[1 * 3 + 2]) / q[1];
		break;
	case 2:
		q[2] = 0.5*sqrt(s[2]);
		q[3] = 0.25*(m[2 * 3 + 1] + m[1 * 3 + 2]) / q[2];
		q[0] = 0.25*(m[0 * 3 + 2] - m[2 * 3 + 0]) / q[2];
		q[1] = 0.25*(m[1 * 3 + 0] + m[0 * 3 + 1]) / q[2];
		break;
	case 3:
		q[3] = 0.5*sqrt(s[3]);
		q[0] = 0.25*(m[1 * 3 + 0] - m[0 * 3 + 1]) / q[3];
		q[1] = 0.25*(m[0 * 3 + 2] + m[2 * 3 + 0]) / q[3];
		q[2] = 0.25*(m[2 * 3 + 1] + m[1 * 3 + 2]) / q[3];
		break;
	}
}

void Mmulnm(double *a, double *b, int m, int n, int k, double *c)
{
	int i, j, l, u;
	for (i = 0; i <= m - 1; i++)
		for (j = 0; j <= k - 1; j++)
		{
			u = i*k + j;
			c[u] = 0.0;
			for (l = 0; l <= n - 1; l++)
				c[u] = c[u] + a[i*n + l] * b[l*k + j];
		}
}

void Mmul(double *a, int m, int n, double b)
{
	for (int i = 0; i<m; i++)
	{
		for (int j = 0; j<n; j++)
		{
			*(a + i*n + j) = *(a + i*n + j)*b;
		}
	}
}

void Mtn(double *a, int m, int n, double *b)
{
	for (int l = 0; l<n; l++)
		for (int k = 0; k<m; k++)
		{
			b[l*m + k] = a[k*n + l];
		}

}

void Mequalm(double *M, int m, int n, double *N)
{
	for (int i = 0; i<m; i++)
	{
		for (int j = 0; j<n; j++)
		{
			*(N + i*n + j) = *(M + i*n + j);
		}
	}
}

void gyo2qua(double gyo[],double imu_time, double q[])
{
	double hft = imu_time / 2;
	q[0] = q[0] + (-q[1] * gyo[0] - q[2] * gyo[1] - q[3] * gyo[2])*hft;
	q[1] = q[1] + (q[0] * gyo[0] + q[2] * gyo[2] - q[3] * gyo[1])*hft;
	q[2] = q[2] + (q[0] * gyo[1] - q[1] * gyo[2] + q[3] * gyo[0])*hft;
	q[3] = q[3] + (q[0] * gyo[2] + q[1] * gyo[1] - q[2] * gyo[0])*hft;
	double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	for (int i = 0;i < 4;i++)
	{
		q[i] = q[i] / norm;
	}

}

void qua2att(double q[], double att[]) //NED
{
	att[1] = -asin((-2 * q[1] * q[3] + 2 * q[0] * q[2]));
	att[0] = atan((2 * q[2] * q[3] + 2 * q[0] * q[1]) / (1 - 2 * (q[1] * q[1] + q[2] * q[2])));
	att[2] = atan((2 * q[1] * q[2] + 2 * q[0] * q[3]) / (1 - (2 * (q[2] * q[2] + q[3] * q[3]))));
}

void angle_change(double acc[], double att[])
{
	if (acc[2] <= 0)
	{
		chang_num++;
	}
	else
	{
		chang_num = 0;
	}
	if(chang_num>0)
	{
		if (att[1] < 0)
		{
			att[1] = -PI - att[1];
		}
		if (att[1] > 0)
		{
			att[1] = PI - att[1];
		}
	}
}

void acc_filter(double acc[], double smooth_acc[])
{
	double smooth_ax = 0, smooth_ay = 0, smooth_az = 0, smooth_gz = 0;
	if (NUM_acc1 <WinLen)
	{
		acc_x_win1[NUM_acc1] = acc[0];
		acc_y_win1[NUM_acc1] = acc[1];
		acc_z_win1[NUM_acc1] = acc[2];
	}
	if (NUM_acc1 >= WinLen)
	{
		for (int i = 0;i < WinLen;i++)
		{
			acc_x_win1[i] = acc_x_win1[i + 1];
			acc_y_win1[i] = acc_y_win1[i + 1];
			acc_z_win1[i] = acc_z_win1[i + 1];
		}
		acc_x_win1[WinLen - 1] = acc[0];
		acc_y_win1[WinLen - 1] = acc[1];
		acc_z_win1[WinLen - 1] = acc[2];
		smooth_ax = dataFilter(acc_x_win1, WinLen);
		smooth_ay = dataFilter(acc_y_win1, WinLen);
		smooth_az = dataFilter(acc_z_win1, WinLen);
		smooth_acc[0] = smooth_ax;
		smooth_acc[1] = smooth_ay;
		smooth_acc[2] = smooth_az;
	}
	else
	{
		smooth_acc[0] = acc[0];
		smooth_acc[1] = acc[1];
		smooth_acc[2] = acc[2];
	}
	NUM_acc1++;
}


	double dataFilter(double acc_win[], int count)
	{
		double MAX = 0, MIN = 0;
		double sum = 0;
		double sum1 = 0;
		MAX = acc_win[0];
		MIN = acc_win[0];
		sum = acc_win[0];
		sum1 = acc_win[0];
		for (int i = 1;i<count; i++)
		{
			if (acc_win[i] > MAX)
			{
				MAX = acc_win[i];
			}
			if (acc_win[i] < MIN)
			{
				MIN = acc_win[i];
			}
			sum += acc_win[i];
			sum1 += acc_win[i];
		}
		double equal_data = (sum - MAX - MIN) / (count - 2);
		sum1 = sum1 / count;
		//printf("data:%f,%f,%f,%f,%f,MAX=%f,MIN=%f,%f,%f\n",acc_win[0], acc_win[1], acc_win[2], acc_win[3], acc_win[4],MAX, MIN,equal_data,sum1);
		return equal_data;
	}
