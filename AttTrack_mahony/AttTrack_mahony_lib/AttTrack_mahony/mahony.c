#include "mahony.h"
#include "DateStruct.h"

static double gyro_x_win1[Staticbias_Wlen] = { 0 };
static double gyro_y_win1[Staticbias_Wlen] = { 0 };
static double gyro_z_win1[Staticbias_Wlen] = { 0 };
static double acc_x_win1[acc_Wlen] = { 0 };
static double acc_y_win1[acc_Wlen] = { 0 };
static double acc_z_win1[acc_Wlen] = { 0 };
static int NUM1 = 0;
static int NUM2 = 0;
void Accnorm(double acc[3], double accnorm[3])
{
	double norm = sqrt(acc[0] * acc[0] + acc[1] * acc[1]+ acc[2] * acc[2]);
	for (int i = 0;i < 3;i++)
	{
		accnorm[i] = acc[i] / norm;
	}
}

void Magnorm(double mag[3], double magnorm[3])
{
	double norm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	for (int i = 0;i < 3;i++)
	{
		magnorm[i] = mag[i] / norm;
	}
}
void AccCn2b(double q[4], double acc_b[3])
{
	double q0q0 = q[0] * q[0];
	double q0q1 = q[0] * q[1];
	double q0q2 = q[0] * q[2];
	double q0q3 = q[0] * q[3];
	double q1q1 = q[1] * q[1];
	double q1q2 = q[1] * q[2];
	double q1q3 = q[1] * q[3];
	double q2q1 = q[2] * q[1];
	double q2q2 = q[2] * q[2];
	double q2q3 = q[2] * q[3];
	double q3q1 = q[3] * q[1];
	double q3q2 = q[3] * q[2];
	double q3q3 = q[3] * q[3];
	acc_b[0] = 2 * (q1q3 - q0q2);
	acc_b[1] = 2 * (q0q1 + q2q3);
	acc_b[2] = (q0q0 - q1q1 - q2q2 + q3q3);

}
void Accerr(double acc[3], double acc_b[3],double accerr[3])
{
	accerr[0] = acc[1] * acc_b[2] - acc[2] * acc_b[1];
	accerr[1] = acc[2] * acc_b[0] - acc[0] * acc_b[2];
	accerr[2] = acc[0] * acc_b[1] - acc[1] * acc_b[0];
}

void Magerr(double mag_n[3],double mag[3],double q[4], double magerr[3])
{
	double wx[3] = { 0 };
	double q0q1 = q[0] * q[1];
	double q0q2 = q[0] * q[2];
	double q0q3 = q[0] * q[3];
	double q1q1 = q[1] * q[1];
	double q1q2 = q[1] * q[2];
	double q1q3 = q[1] * q[3];
	double q2q1 = q[2] * q[1];
	double q2q2 = q[2] * q[2];
	double q2q3 = q[2] * q[3];
	double q3q1 = q[3] * q[1];
	double q3q2 = q[3] * q[2];
	double q3q3 = q[3] * q[3];
	wx[0] = 2 * mag_n[0] * (0.5 - q2q2 - q3q3) + 2 * mag_n[2] * (q1q3 - q0q2);
	wx[1] = 2 * mag_n[0] * (q1q2 - q0q3) + 2 * mag_n[2] * (q0q1 - q2q3);
	wx[2] = 2 * mag_n[0] * (q0q2 + q1q3) + 2 * mag_n[2] * (0.5 - q1q1 - q2q2);
	magerr[0] = mag[1] * wx[2] - mag[2] * wx[1];
	magerr[1] = mag[2] * wx[2] - mag[0] * wx[2];
	magerr[2] = mag[0] * wx[2] - mag[1] * wx[0];
}

void MagCb2n(double mag[3], double q[4], double mag_n[3])
{
	double q0q1 = q[0] * q[1];
	double q0q2 = q[0] * q[2];
	double q0q3 = q[0] * q[3];
	double q1q1 = q[1] * q[1];
	double q1q2 = q[1] * q[2];
	double q1q3 = q[1] * q[3];
	double q2q1 = q[2] * q[1];
	double q2q2 = q[2] * q[2];
	double q2q3 = q[2] * q[3];
	double q3q1 = q[3] * q[1];
	double q3q2 = q[3] * q[2];
	double q3q3 = q[3] * q[3];
	double hx[3] = { 0 };
	hx[0] = 2 * mag[0] * (0.5 - q2q2 - q3q3) + 2 * mag[1] * (q1q2 - q0q3) + 2 * mag[2] * (q1q3 + q0q1);
	hx[1] = 2 * mag[0] * (q1q2 - q0q3) + 2 * mag[1] * (0.5-q1q1 - q3q3) + 2 * mag[2] * (q2q3 - q0q1);
	hx[2] = 2 * mag[0] * (q1q3 - q0q2) + 2 * mag[1] * (q2q3 - q0q1) + 2 * mag[2] * (0.5-q1q1 + q2q2);
	mag_n[0] = sqrt(hx[0] * hx[0] + hx[1] * hx[1]);//nœµœ¬¥≈¡¶º∆¿ÌœÎ∑÷¡ø
	mag_n[1] = 0;
	mag_n[2] = hx[2];
}


void AccMag_err(double accerr[3], double magerr[3], double amerr[3])
{
	for (int i = 0;i < 3;i++)
	{
		amerr[i] = accerr[i] + magerr[i];
	}
}

int PIcontrol(double gyo[3], double amerr[3], mp_t* mp)
{
	if (mp->normacc > 0.4)
	{
		mp->temp_Ki = 0.0001;
		mp->temp_Kp = 0.8;
		for (int i = 0;i < 3;i++)
		{
			gyo[i] -= mp->gyo_bias[i];//Õ”¬›»•µÙ¡„∆´
			mp->sume[i] += amerr[i] * mp->temp_Ki;
		}

		for (int i = 0;i < 3;i++)
		{
			mp->gyo_offset[i] = mp->temp_Kp*amerr[i] + mp->sume[i] + gyo[i];
			//mp->gyo_offset[i] = gyo[i]+ Kp*amerr[i];
		}
	}
	else
	{
		for (int i = 0;i < 3;i++)
		{
			gyo[i] -= mp->gyo_bias[i];//Õ”¬›»•µÙ¡„∆´
			mp->sume[i] += amerr[i] * Ki;
		}

		for (int i = 0;i < 3;i++)
		{
			mp->gyo_offset[i] = Kp*amerr[i] + mp->sume[i] + gyo[i];
			//mp->gyo_offset[i] = gyo[i]+ Kp*amerr[i];
		}
	}
	//printf("DATA2:%f,%f,%f\n", mp->gyo_offset[0], mp->gyo_offset[1], mp->gyo_offset[2]);
	return 1;
}


int init_process(IMUdata_t* imu, MAGdata_t* mag, mp_t* mp)
{
	if ((fabs(imu->acc[0])>0) && (fabs(imu->acc[1])>0) && (fabs(imu->acc[2]) > 0))
	{
		int static_num = acc_Wlen;
		if (NUM1 <= static_num)
		{
			acc_x_win1[NUM2] = imu->acc[0];
			acc_y_win1[NUM2] = imu->acc[1];
			acc_z_win1[NUM2] = imu->acc[2];

		}
		else
		{
			for (int i = 0;i < static_num;i++)
			{
				acc_x_win1[i] = acc_x_win1[i + 1];
				acc_y_win1[i] = acc_y_win1[i + 1];
				acc_z_win1[i] = acc_z_win1[i + 1];
				//aint->bias_gz += gyro_z_win[i];
			}
			acc_x_win1[static_num - 1] = imu->acc[0];
			acc_y_win1[static_num - 1] = imu->acc[1];
			acc_z_win1[static_num - 1] = imu->acc[2];
		}
		int num = static_num;
		double meanax = 0;
		double meanay = 0;
		double meanaz = 0;
		for (int i = 1; i < num - 1; i++)
		{
			meanax += acc_x_win1[i];
			meanay += acc_y_win1[i];
			meanaz += acc_z_win1[i];
		}
		if (NUM2 > 2)
		{
			meanax = meanax / (static_num - 2);
			meanay = meanay / (static_num - 2);
			meanaz = meanaz / (static_num - 2);
		}
		//printf("%f,%f,%f,%d\n", imu->gyo[0], imu->gyo[1], imu->gyo[2],NUM1);
		NUM2++;
		mp->roll = atan2(meanay, meanaz);//∫·πˆΩ«
		mp->pitch = atan2(-meanax, meanaz);//∫·πˆΩ«
		//mp->heading = atan2(mag->mag_data[0], mag->mag_data[1]);//∫ΩœÚΩ«
		mp->heading = 0;
		double att[3] = { 0 };
		att[0] = mp->roll;
		att[1] = mp->pitch;
		att[2] = mp->heading;
		a2mat_ned(att, mp->Cb2n);
		m2qua_ned(mp->Cb2n, mp->q);
	}
	return 1;
}

int grobias(IMUdata_t* imu, mp_t* mp)
{

	int static_num = Staticbias_Wlen;
	if (NUM1 <= static_num)
	{
		gyro_x_win1[NUM1] = imu->gyo[0];//x÷·¡„∆´
		gyro_y_win1[NUM1] = imu->gyo[1];//y÷·¡„∆´
		gyro_z_win1[NUM1] = imu->gyo[2];//y÷·¡„∆´

	}
	else
	{
		for (int i = 0;i < static_num;i++)
		{
			gyro_x_win1[i] = gyro_x_win1[i + 1];
			gyro_y_win1[i] = gyro_y_win1[i + 1];
			gyro_z_win1[i] = gyro_z_win1[i + 1];
			//aint->bias_gz += gyro_z_win[i];
		}
		gyro_x_win1[static_num - 1] = imu->gyo[0];
		gyro_y_win1[static_num - 1] = imu->gyo[1];
		gyro_z_win1[static_num - 1] = imu->gyo[2];
	}
	int num = static_num;
	double meangx = 0;
	double meangy = 0;
	double meangz = 0;
	for (int i = 1; i < num - 1; i++)
	{
		meangx += gyro_x_win1[i];
		meangy += gyro_y_win1[i];
		meangz += gyro_z_win1[i];
	}
	if (NUM1 > 2)
	{
		mp->gyo_bias[0] = meangx / (static_num - 2);
		mp->gyo_bias[1] = meangy / (static_num - 2);
		mp->gyo_bias[2] = meangz / (static_num - 2);
	}
	//printf("%f,%f,%f,%d\n", imu->gyo[0], imu->gyo[1], imu->gyo[2],NUM1);
	NUM1++;
	if (NUM1 == static_num)
	{

		return 1;
	}
	else
	{
		return 0;
	}
}


void mp_init(mp_t *mp)
{
	mp->accinit = 0;
	mp->maginit = 0;
	mp->mpinit = 0;
	mp->pitch = 0;
	mp->roll = 0;
	mp->heading = 0;
	for (int i = 0;i < 9;i++)
	{
		mp->Cb2n[i] = 0;
	}
	for (int i = 0;i < 4;i++)
	{
	mp->q[i] = 0;
	}
	mp->q[0] = 1;
	for (int i = 0;i < 3;i++)
	{
		mp->sume[i] = 0;
	}
	for (int i = 0;i < 3;i++)
	{
		mp->gyo_offset[i] = 0;
		mp->gyo_bias[i] = 0;
		mp->acc_err[i] = 0;
		mp->acc_norm[i] = 0;
		mp->am_err[i] = 0;
		mp->mag_err[i] = 0;
		mp->mag_norm[i] = 0;
		mp->att[i] = 0;
	}
	mp->normacc = 0;
	mp->temp_Ki = 0;
	mp->temp_Kp = 0;
}

double acc_norm(double acc[])
{
	double normacc = fabs(sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]) / GN - 1.0);
	return normacc;
}