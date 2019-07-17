#include "AttInit.h"
static double acc_x_win[Staticbias_Wlen] = { 0 };
static double acc_y_win[Staticbias_Wlen] = { 0 };
static double acc_z_win[Staticbias_Wlen] = { 0 };
static double gyro_x_win[Staticbias_Wlen] = { 0 };
static double gyro_y_win[Staticbias_Wlen] = { 0 };
static double gyro_z_win[Staticbias_Wlen] = { 0 };
static  int NUM = 0;
static double acc_x_win1[Staticbias_Wlen] = { 0 };
static double acc_y_win1[Staticbias_Wlen] = { 0 };
static double acc_z_win1[Staticbias_Wlen] = { 0 };
static double gyro_x_win1[Staticbias_Wlen] = { 0 };
static double gyro_y_win1[Staticbias_Wlen] = { 0 };
static  int NUM1 = 0;
static double acc_x_win2[Staticbias_Wlen] = { 0 };
static double acc_y_win2[Staticbias_Wlen] = { 0 };
static double acc_z_win2[Staticbias_Wlen] = { 0 };
static double gyro_y_win2[Staticbias_Wlen] = { 0 };
int NUM2 = 0;
static double acc_x_win3[Staticbias_Wlen] = { 0 };
static double acc_y_win3[Staticbias_Wlen] = { 0 };
static double acc_z_win3[Staticbias_Wlen] = { 0 };
static double gyro_y_win3[Staticbias_Wlen] = { 0 };
int NUM3 = 0;

static double acc_x_win4[Staticbias_Wlen] = { 0 };
static double acc_y_win4[Staticbias_Wlen] = { 0 };
static double acc_z_win4[Staticbias_Wlen] = { 0 };
static double gyro_x_win4[Staticbias_Wlen] = { 0 };
static double gyro_y_win4[Staticbias_Wlen] = { 0 };
static double gyro_z_win4[Staticbias_Wlen] = { 0 };
int NUM4 = 0;
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
int process_singleangle(struct AttInit* aint, AttTrackCfg_t*cfg, double gyo[3], double acc1[3])
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

int process_singleangle_D(struct AttInit* aint, double gyo[3], double acc1[3])
{
	int static_num = Staticbias_Wlen;
	if (NUM1 <= static_num)
	{

		acc_x_win1[NUM1] = acc1[0];//´°¿Ú³õÊ¼»¯
		acc_y_win1[NUM1] = acc1[1];
		acc_z_win1[NUM1] = acc1[2];
		gyro_x_win1[NUM1] = gyo[0];//xÖáÁãÆ«
		gyro_y_win1[NUM1] = gyo[1];//yÖáÁãÆ«

	}
	else
	{
		for (int i = 0;i < static_num;i++)
		{
			acc_x_win1[i] = acc_x_win1[i + 1];//´°¿Ú»¬¶¯
			acc_y_win1[i] = acc_y_win1[i + 1];
			acc_z_win1[i] = acc_z_win1[i + 1];
			gyro_x_win1[i] = gyro_x_win1[i + 1];
			gyro_y_win1[i] = gyro_y_win1[i + 1];
			//aint->bias_gz += gyro_z_win[i];
		}
		acc_x_win1[static_num - 1] = acc1[0];
		acc_y_win1[static_num - 1] = acc1[1];
		acc_z_win1[static_num - 1] = acc1[2];
		gyro_x_win1[static_num - 1] = gyo[0];
		gyro_y_win1[static_num - 1] = gyo[1];
	}
	NUM1++;
	double meanax = 0, meanay = 0, meanaz = 0;
	int num =static_num;
	double meangx = 0;
	double meangy = 0;
	for (int i = 1; i < num - 1; i++)
	{
		meanax += acc_x_win1[i];
		meanay += acc_y_win1[i];
		meanaz += acc_z_win1[i];
		meangx += gyro_x_win1[i];
		meangy += gyro_y_win1[i];
	}

	

	if (num > 3)
	{
		meanax /= (num - 2);
		meanay /= (num - 2);
		meanaz /= (num - 2);
	aint->bias_gx = meangx / (static_num - 2);
	aint->bias_gy = meangy / (static_num - 2);
		for (int i = 1; i < num - 1; i++)
		{
			aint->std_ax += (acc_x_win1[i] - meanax)*(acc_x_win1[i] - meanax);
			aint->std_ay += (acc_y_win1[i] - meanay)*(acc_y_win1[i] - meanay);
			aint->std_az += (acc_z_win1[i] - meanaz)*(acc_z_win1[i] - meanaz);
			aint->std_gx += (gyro_x_win1[i] - aint->bias_gx)*(gyro_x_win1[i] - aint->bias_gx);
			aint->std_gy += (gyro_y_win1[i] - aint->bias_gy)*(gyro_y_win1[i] - aint->bias_gy);
		}
		aint->std_ax = sqrt(aint->std_ax / (num - 3));
		aint->std_ay = sqrt(aint->std_ay / (num - 3));
		aint->std_az = sqrt(aint->std_az / (num - 3));
		aint->std_gx = sqrt(aint->std_gx / (num - 3));
		aint->std_gy = sqrt(aint->std_gy / (num - 3));
	}
	else
	{
		aint->bias_gx = 0;
		aint->bias_gy = 0;
		aint->std_ax = 0;
		aint->std_ay = 0;
		aint->std_az = 0;
		aint->std_gx = 0;
		aint->std_gy= 0;
		meanax = 1;
		meanay = 1;
		meanaz= 1;
	}
	//if (aint->std_ax > 0.05 || aint->std_ay > 0.05 || aint->std_az > 0.05) // 0.05
	//{
	//	printf("¼Ó¼Æ¾²Ì¬std³¬ÏÞ£¡\n");
	//	//return 0;
	//}
	//if (aint->std_gx > 0.001||aint->std_gy > 0.001) //0.001
	//{
	//	printf("ÍÓÂÝ¾²Ì¬std³¬ÏÞ£¡\n");
	//	//return 0;
	//}

	//¼ÆËãºá¹öºÍ¸©Ñö½Ç
	aint->att[0] = atan2(-meanay , -meanaz);//ºá¹ö
	aint->att[1] = atan2(meanax , sqrt(meanay*meanay + meanaz*meanaz));//¸©Ñö½Ç
	aint->att[2] = 0;
	//printf("static_att: %f,%f,%f,%d,%f\n", aint->std_ax,aint->bias_gz,aint->att[1] * R2D,NUM1,gyo[2]);
	if(NUM1==static_num)
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

int process_singleangle_U(struct AttInit* aint, double gyo[3], double acc1[3])
{
	int static_num = Staticbias_Wlen;
	if (NUM2 <= static_num)
	{
		acc_x_win2[NUM2] = acc1[0];//´°¿Ú³õÊ¼»¯
		acc_y_win2[NUM2] = acc1[1];
		acc_z_win2[NUM2] = acc1[2];
		gyro_y_win2[NUM2] = gyo[1];

	}
	else
	{
		for (int i = 0;i < static_num;i++)
		{
			acc_x_win2[i] = acc_x_win2[i + 1];//´°¿Ú»¬¶¯
			acc_y_win2[i] = acc_y_win2[i + 1];
			acc_z_win2[i] = acc_z_win2[i + 1];
			gyro_y_win2[i] = gyro_y_win2[i + 1];
			//aint->bias_gz += gyro_z_win[i];
		}
		acc_x_win2[static_num - 1] = acc1[0];
		acc_y_win2[static_num - 1] = acc1[1];
		acc_z_win2[static_num - 1] = acc1[2];
		gyro_y_win2[static_num - 1] = gyo[1];
	}
	NUM2++;
	double meanax = 0, meanay = 0, meanaz = 0;
	int num = static_num;
	double meangy = 0;
	for (int i = 1; i < num - 1; i++)
	{
		meanax += acc_x_win2[i];
		meanay += acc_y_win2[i];
		meanaz += acc_z_win2[i];
		meangy += gyro_y_win2[i];
	}

	if (num > 3)
	{
		meanax /= (num - 2);
		meanay /= (num - 2);
		meanaz /= (num - 2);
		aint->bias_gy = meangy / (static_num - 2);
		for (int i = 1; i < num - 1; i++)
		{
			aint->std_ax += (acc_x_win2[i] - meanax)*(acc_x_win2[i] - meanax);
			aint->std_ay += (acc_y_win2[i] - meanay)*(acc_y_win2[i] - meanay);
			aint->std_az += (acc_z_win2[i] - meanaz)*(acc_z_win2[i] - meanaz);
			aint->std_gy+= (gyro_y_win2[i] - aint->bias_gy)*(gyro_y_win2[i] - aint->bias_gy);
		}
		aint->std_ax = sqrt(aint->std_ax / (num - 3));
		aint->std_ay = sqrt(aint->std_ay / (num - 3));
		aint->std_az = sqrt(aint->std_az / (num - 3));
		aint->std_gy = sqrt(aint->std_gy / (num - 3));
	}
	else
	{
		aint->bias_gy = 0;
		aint->std_ax = 0;
		aint->std_ay = 0;
		aint->std_az = 0;
		aint->std_gy = 0;
		meanax = 1;
		meanay = 1;
		meanaz = 1;
	}

	//if (aint->std_ax > 0.05 || aint->std_ay > 0.05 || aint->std_az > 0.05) // 0.05
	//{
	//	printf("¼Ó¼Æ¾²Ì¬std³¬ÏÞ£¡\n");
	//	//return 0;
	//}
	//if (aint->std_gy > 0.001) //0.001
	//{
	//	printf("ÍÓÂÝ¾²Ì¬std³¬ÏÞ£¡\n");
	//	//return 0;
	//}

	//¼ÆËãºá¹öºÍ¸©Ñö½Ç
	aint->att[0] = atan2(-meanay , -meanaz);//ºá¹ö
	aint->att[1] = atan2(meanax , sqrt(meanay*meanay + meanaz*meanaz));//¸©Ñö½Ç
	aint->att[2] = 0;
	//printf("static_att: %f,%f,%f,%d,%f\n", aint->std_ax,aint->bias_gy,aint->att[1] * R2D,NUM2,gyo[2]);

	if (NUM2 >= static_num)
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


int process_singleangle_UE(struct AttInit* aint, double gyo[3])
{
	int static_num = Staticbias_Wlen;
	if (NUM3 <= static_num&&gyo[2]<0.005)
	{
		gyro_y_win3[NUM3] = gyo[1];
		NUM3++;
	}
	else
	{
		for (int i = 0;i < static_num;i++)
		{
			gyro_y_win3[i] = gyro_y_win3[i + 1];
			//aint->bias_gz += gyro_z_win[i];
		}
		gyro_y_win3[static_num - 1] = gyo[1];
	}
	int num = static_num;
	double meangy = 0;
	for (int i = 1; i < num - 1; i++)
	{
		meangy += gyro_y_win3[i];
	}
	aint->bias_gy = meangy / (static_num - 2);
	if (num > 3)
	{

		for (int i = 1; i < num - 1; i++)
		{
			aint->std_gy += (gyro_y_win3[i] - aint->bias_gy)*(gyro_y_win3[i] - aint->bias_gy);
		}
		aint->std_gy = sqrt(aint->std_gy / (num - 3));
	}
	else
	{
		aint->bias_gy = 0;
		aint->std_gy = 0;
	}
	if (aint->std_gy > 0.001) //0.001
	{
		printf("ÍÓÂÝ¾²Ì¬std³¬ÏÞ£¡\n");
		//return 0;
	}
	if (NUM3 >= static_num)
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

int process_singleangle_3D(struct AttInit* aint, double gyo[3], double acc1[3])
{
	int static_num = Staticbias_Wlen;
	if (NUM4 <= static_num&&gyo[2]<0.005)
	{
		acc_x_win4[NUM4] = acc1[0];//´°¿Ú³õÊ¼»¯
		acc_y_win4[NUM4] = acc1[1];
		acc_z_win4[NUM4] = acc1[2];
		gyro_x_win4[NUM4] = gyo[0];
		gyro_y_win4[NUM4] = gyo[1];
		gyro_z_win4[NUM4] = gyo[2];
		NUM4++;
	}
	else
	{
		for (int i = 0;i < static_num;i++)
		{
			acc_x_win4[i] = acc_x_win4[i + 1];//´°¿Ú»¬¶¯
			acc_y_win4[i] = acc_y_win4[i + 1];
			acc_z_win4[i] = acc_z_win4[i + 1];
			gyro_x_win4[i] = gyro_x_win4[i + 1];
			gyro_y_win4[i] = gyro_y_win4[i + 1];
			gyro_z_win4[i] = gyro_z_win4[i + 1];
			//aint->bias_gz += gyro_z_win[i];
		}
		acc_x_win4[static_num - 1] = acc1[0];
		acc_y_win4[static_num - 1] = acc1[1];
		acc_z_win4[static_num - 1] = acc1[2];
		gyro_x_win4[static_num - 1] = gyo[0];
		gyro_y_win4[static_num - 1] = gyo[1];
		gyro_z_win4[static_num - 1] = gyo[2];
	}

	double meanax = 0, meanay = 0, meanaz = 0;
	int num = static_num;
	double meangx = 0, meangy = 0, meangz = 0;
	for (int i = 1; i < num - 1; i++)
	{
		meanax += acc_x_win4[i];
		meanay += acc_y_win4[i];
		meanaz += acc_z_win4[i];
		meangx += gyro_x_win4[i];
		meangy += gyro_y_win4[i];
		meangz += gyro_z_win4[i];
	}
	aint->bias_gx = meangx / (static_num - 2);
	aint->bias_gy = meangy / (static_num - 2);
	aint->bias_gz = meangz / (static_num - 2);
	if (num > 3)
	{
		meanax /= (num - 2);
		meanay /= (num - 2);
		meanaz /= (num - 2);

		for (int i = 1; i < num - 1; i++)
		{
			aint->std_ax += (acc_x_win4[i] - meanax)*(acc_x_win4[i] - meanax);
			aint->std_ay += (acc_y_win4[i] - meanay)*(acc_y_win4[i] - meanay);
			aint->std_az += (acc_z_win4[i] - meanaz)*(acc_z_win4[i] - meanaz);
			aint->std_gx += (gyro_x_win4[i] - aint->bias_gx)*(gyro_x_win4[i] - aint->bias_gx);
			aint->std_gy += (gyro_y_win4[i] - aint->bias_gy)*(gyro_y_win4[i] - aint->bias_gy);
			aint->std_gz += (gyro_z_win4[i] - aint->bias_gz)*(gyro_z_win4[i] - aint->bias_gz);
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
		aint->bias_gy = 0;
		aint->std_ax = 0;
		aint->std_ay = 0;
		aint->std_az = 0;
		aint->std_gy = 0;
		meanax = 1;
		meanay = 1;
		meanaz = 1;
	}

	if (aint->std_ax > 0.05 || aint->std_ay > 0.05 || aint->std_az > 0.05) // 0.05
	{
		printf("¼Ó¼Æ¾²Ì¬std³¬ÏÞ£¡\n");
		//return 0;
	}
	if (aint->std_gz > 0.001) //0.001
	{
		printf("ÍÓÂÝ¾²Ì¬std³¬ÏÞ£¡\n");
		//return 0;
	}

	//¼ÆËãºá¹öºÍ¸©Ñö½Ç
	aint->att[0] = atan(meanay / meanaz);//ºá¹ö
	aint->att[1] = atan(meanax / sqrt(meanay*meanay + meanaz*meanaz));//¸©Ñö½Ç
	aint->att[2] = 0;
	//printf("static_att: %f,%f,%f,%d,%f\n", aint->std_ax,aint->bias_gz,aint->att[1] * R2D,NUM,gyo[2]);

	if (NUM4 >= static_num)
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


