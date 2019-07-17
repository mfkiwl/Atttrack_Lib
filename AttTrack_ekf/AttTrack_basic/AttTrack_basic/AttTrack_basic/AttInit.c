#include "AttInit.h"
static double acc_x_win[Staticbias_Wlen] = { 0 };
static double acc_y_win[Staticbias_Wlen] = { 0 };
static double acc_z_win[Staticbias_Wlen] = { 0 };
static double gyro_x_win[Staticbias_Wlen] = { 0 };
static double gyro_y_win[Staticbias_Wlen] = { 0 };
static double gyro_z_win[Staticbias_Wlen] = { 0 };
static double gyro_win[Staticbias_Wlen] = { 0 };
static  int NUM = 0;
static double angle_win[Staticbias_Wlen] = { 0 };
static  int NUM1 = 0;
void attInit(struct AttInit* aint)
{
	aint->bias_gx = 0;
	aint->bias_gy = 0;
	aint->bias_gz = 0;
	aint->std_gx = 0;
	aint->std_gy = 0;
	aint->std_gz = 0;
	aint->std_ax = 0;
	aint->std_ay = 0;
	aint->std_az = 0;
	aint->std_az2 = 0;
	aint->mean_gpsyaw = 0;
	aint->std_gpsyaw = 0;
	for (int i = 0; i < 3; i++)
	{
		aint->att[i] = 0;
	}
	for (int i = 0; i < 2; i++)
	{
		aint->tilt[i] = 0;
	}
	aint->bfinshinit = 0;
}
int process_singleangle(struct AttInit* aint, Config_t*cfg, double gyo[3], double acc1[3])
{
	int static_num = Staticbias_Wlen;
	if (NUM <= static_num)
	{

		acc_x_win[NUM] = acc1[0];//´°¿Ú³õÊ¼»¯
		acc_y_win[NUM] = acc1[1];
		acc_z_win[NUM] = acc1[2];
		gyro_x_win[NUM] = gyo[0];//xÖáÁãÆ«
		gyro_y_win[NUM] = gyo[1];//yÖáÁãÆ«
		gyro_z_win[NUM] = gyo[2];//zÖáÁãÆ«
	}
	else
	{
		for (int i = 0;i < static_num;i++)
		{
			acc_x_win[i] = acc_x_win[i + 1];//´°¿Ú»¬¶¯
			acc_y_win[i] = acc_y_win[i + 1];
			acc_z_win[i] = acc_z_win[i + 1];
			gyro_x_win[i] = gyro_x_win[i + 1];
			gyro_y_win[i] = gyro_y_win[i + 1];
			gyro_z_win[i] = gyro_z_win[i + 1];
			//aint->bias_gz += gyro_z_win[i];
		}
		acc_x_win[static_num - 1] = acc1[0];
		acc_y_win[static_num - 1] = acc1[1];
		acc_z_win[static_num - 1] = acc1[2];
		gyro_x_win[static_num - 1] = gyo[0];
		gyro_y_win[static_num - 1] = gyo[1];
		gyro_z_win[static_num - 1] = gyo[2];
	}
	NUM++;
	double meanax = 0, meanay = 0, meanaz = 0;
	int num = static_num;
	double meangx = 0;
	double meangy = 0;
	double meangz = 0;
	for (int i = 1; i < num - 1; i++)
	{
		meanax += acc_x_win[i];
		meanay += acc_y_win[i];
		meanaz += acc_z_win[i];
		meangx += gyro_x_win[i];
		meangy += gyro_y_win[i];
		meangz += gyro_z_win[i];
	}



	if (num > 3)
	{
		meanax /= (num - 2);
		meanay /= (num - 2);
		meanaz /= (num - 2);
		aint->bias_gx = meangx / (static_num - 2);
		aint->bias_gy = meangy / (static_num - 2);
		aint->bias_gz = meangz / (static_num - 2);
		for (int i = 1; i < num - 1; i++)
		{
			aint->std_ax += (acc_x_win[i] - meanax)*(acc_x_win[i] - meanax);
			aint->std_ay += (acc_y_win[i] - meanay)*(acc_y_win[i] - meanay);
			aint->std_az += (acc_z_win[i] - meanaz)*(acc_z_win[i] - meanaz);
			aint->std_gx += (gyro_x_win[i] - aint->bias_gx)*(gyro_x_win[i] - aint->bias_gx);
			aint->std_gy += (gyro_y_win[i] - aint->bias_gy)*(gyro_y_win[i] - aint->bias_gy);
			aint->std_gz += (gyro_z_win[i] - aint->bias_gz)*(gyro_z_win[i] - aint->bias_gz);
		}
		aint->std_ax = sqrt(aint->std_ax / (num - 3));
		aint->std_ay = sqrt(aint->std_ay / (num - 3));
		aint->std_az = sqrt(aint->std_az / (num - 3));
		aint->std_gx = sqrt(aint->std_gx / (num - 3));
		aint->std_gy = sqrt(aint->std_gy / (num - 3));
		aint->std_gz = sqrt(aint->std_gz / (num - 3));
	}
	else
	{
		aint->bias_gx = 0;
		aint->bias_gy = 0;
		aint->bias_gz = 0;
		aint->std_ax = 0;
		aint->std_ay = 0;
		aint->std_az = 0;
		aint->std_gx = 0;
		aint->std_gy = 0;
		aint->std_gz = 0;
		meanax = 1;
		meanay = 1;
		meanaz = 1;
	}
	if (aint->std_ax > cfg->astdxthr || aint->std_ay > cfg->astdythr || aint->std_az > cfg->astdzthr) // 0.05
	{
		printf("¼Ó¼Æ¾²Ì¬std³¬ÏÞ£¡\n");
		//return 0;
	}
	if (aint->std_gx >  cfg->astdxthr ||aint->std_gy >  cfg->astdythr|| aint->std_gz >  cfg->astdzthr) //0.001
	{
		printf("ÍÓÂÝ¾²Ì¬std³¬ÏÞ£¡\n");
		//return 0;
	}

	//¼ÆËãºá¹öºÍ¸©Ñö½Ç
	aint->att[0] = atan2(-meanay, -meanaz);//ºá¹ö
	aint->att[1] = atan2(meanax, sqrt(meanay*meanay + meanaz*meanaz));//¸©Ñö½Ç
	aint->att[2] = 0;
	//printf("static_att: %f,%f,%f,%d,%f\n", aint->std_ax,aint->bias_gz,aint->att[1] * R2D,NUM1,gyo[2]);
	if (NUM == static_num)
	{
		aint->bfinshinit = 1;
		return 1;
	}
	else
	{
		aint->bfinshinit = 0;
		return 0;
	}

}

int process_gyobias(Config_t*cfg, double gyo,double gyo_bias)
{
	int static_num = Staticbias_Wlen;
	if (NUM <= static_num)
	{
		gyro_win[NUM] = gyo;//xÖáÁãÆ«
	}
	else
	{
		for (int i = 0;i < static_num;i++)
		{
			gyro_win[i] = gyro_win[i + 1];
			//aint->bias_gz += gyro_z_win[i];
		}
		gyro_win[static_num - 1] = gyo;
	}
	NUM++;
	int num = static_num;
	double meang = 0;
	for (int i = 1; i < num - 1; i++)
	{
		meang += gyro_win[i];
	}

	double gyobias = 0;
	double grostd = 0;
	if (num > 3)
	{
		gyobias = meang / (static_num - 2);
		for (int i = 1; i < num - 1; i++)
		{
			grostd += (gyro_win[i] - gyobias)*(gyro_z_win[i] - gyobias);
		}
		grostd = sqrt(grostd / (num - 3));
	}

	if (grostd  >  cfg->astdxthr ) //0.001
	{
		printf("ÍÓÂÝ¾²Ì¬std³¬ÏÞ£¡\n");
		//return 0;
	}
	//printf("static_att: %f,%f,%f,%d,%f\n", aint->std_ax,aint->bias_gz,aint->att[1] * R2D,NUM1,gyo[2]);
	if (NUM == static_num)
	{
		gyo_bias = gyobias;
		return 1;
	}
	else
	{
		return 0;
	}

}

int angle_equal(double angle, double *equal_angle)
{
	int static_num = Staticbias_Wlen;
	if (NUM1 <= static_num)
	{
		angle_win[NUM1] = angle;//xÖáÁãÆ«
	}
	else
	{
		for (int i = 0;i < static_num;i++)
		{
			angle_win[i] = angle_win[i + 1];
		}
		angle_win[static_num - 1] = angle;
	}
	NUM1++;
	int num = static_num;
	double meanangle= 0;
	for (int i = 1; i < num-1; i++)
	{
		meanangle += angle_win[i];
	}

	if (num > 3)
	{
		meanangle = meanangle / (static_num-2);
	}
	else
	{
		meanangle = 0;
	}
	*equal_angle = meanangle;
	return 1;
}
