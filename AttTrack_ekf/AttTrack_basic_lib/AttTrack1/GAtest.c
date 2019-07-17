#include "AttTrack_lib.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "DATAread.h"
#include "DataStruct.h"
#include "ComFunc.h"
double gyro_y1_win1[200] = { 0 };
double gyro_x1_win1[200] = { 0 };

void main()
{
	int pp_num1 = 0;
	FILE *fp;
	FILE *fpp;
	struct ATdata atd;
	struct chardata charreaddata;
	double integ_pitch1 = 0;
	double integ_roll1 = 0;
	static double biasx1 = 0;
	static double biasy1 = 0;
	static int initstruct = 0;
	//fp = fopen("C:/Users/huace/Desktop/2.22/gadata03.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/2.22/compass1/gadatac02.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/301/tmp-w1.txt", "rt");
	//fp = fopen("E:/vs_project/AHRS/ahrs/data/GA-C2-1226.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/304/boom01.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/304/stick02.txt", "rt");
	fp = fopen("C:/Users/huace/Desktop/304/dogbone03.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/304/body03.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/3-16温度测试/data70.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/安装误差验证/data22.txt", "rt");
	if (!fp)
	{
		printf("cannot open file01\n");
	}
	//fpp = fopen("C:/Users/huace/Desktop/2.22/gadata03out.txt", "wt");
	//fpp = fopen("C:/Users/huace/Desktop/2.22/compass1/gadatac02out.txt", "wt");
	//fpp = fopen("C:/Users/huace/Desktop/301/tmp-w1out.txt", "wt");
	//fpp = fopen("C:/Users/huace/Desktop/301/GA-C2out.txt", "wt");
	//fpp = fopen("C:/Users/huace/Desktop/304/boom01out.txt", "wt");
	//fpp = fopen("C:/Users/huace/Desktop/304/stick02out.txt", "wt");
	fpp = fopen("C:/Users/huace/Desktop/304/dogbone03out.txt", "wt");
	//fpp = fopen("C:/Users/huace/Desktop/304/body03out.txt", "wt");
	//fpp = fopen("C:/Users/huace/Desktop/3-16温度测试/data70out.txt", "wt");
	//fpp = fopen("C:/Users/huace/Desktop/安装误差验证/data22out.txt", "wt");
	if (!fpp)
	{
		printf("cannot open file02\n");
	}
	double acc_roll2=0;
	double acc_pitch2=0;
#if 1
	while (!feof(fp))
	{
		if (!initstruct)
		{
			IMUstructinit(&charreaddata);
			initstruct = 1;
		}
		//GAsensordataread1(&charreaddata, fp);
		GAdataread(&charreaddata, fp);//三一GAsensor数据
		//IS203dataread1(&charreaddata, fp);//三一GAsensor数据
		//GAsensordataread(&charreaddata, fp);//12.26gasensor数据
		double accm1[3] = { 0 }, gyom1[3] = { 0 };
#if 0
		accm1[0] = charreaddata.acc_data[1] * 9.80665; //三轴加速度计(坐标系统一 （NED）)
		accm1[1] = -charreaddata.acc_data[2] * 9.80665;
		accm1[2] = charreaddata.acc_data[0] * 9.80665;
		gyom1[0] = charreaddata.gyro_data[0];//(坐标系统一 （NED）)
		gyom1[1] = charreaddata.gyro_data[1]*D2R;
		gyom1[2] = charreaddata.gyro_data[2];
#endif
#if 1
		accm1[0] = charreaddata.acc_data[0] * 9.80665; //三轴加速度计(坐标系统一 （NED）)
		accm1[1] = charreaddata.acc_data[2 ]* 9.80665;
		accm1[2] = charreaddata.acc_data[1] * 9.80665;
		gyom1[0] = charreaddata.gyro_data[0];//(坐标系统一 （NED）)
		gyom1[1] = charreaddata.gyro_data[1];
		gyom1[2] = charreaddata.gyro_data[2];
#endif

		//算法接口数据结构体赋值
		atd.imu.accx= accm1[0];
		atd.imu.accy = accm1[1];
		atd.imu.accz = accm1[2];
		atd.imu.gyox = gyom1[0];
		atd.imu.gyoy = gyom1[1];
		atd.imu.gyoz = gyom1[2];
		atd.imu.imutimetarget= charreaddata.imu_Ts;
		//ahrstestdata.imu.imutimetarget = 0.005;
		atd.bimu_updta = 1;
		atd.bcompass_updata = 0;
		atd.bgnss_updata = 0;

#if 1
		/****************************加速度计计算姿态角*******************************/
		 acc_roll2 = atan2((-accm1[1]),(-accm1[2]))*R2D;
		 acc_pitch2 = atan2(accm1[0],  (-accm1[2]))*R2D;//三轴加速度计算俯仰角
		//陀螺仪积分
		static double sum_gx = 0;
		static double sum_gy = 0;
		if (pp_num1<200)//窗口数据初始化
		{
			gyro_y1_win1[pp_num1] = gyom1[1];
			gyro_x1_win1[pp_num1] = gyom1[0];
			integ_pitch1 = acc_pitch2;
			integ_roll1 = acc_roll2;
		}
		else if (pp_num1 == 200)
		{

			for (int i = 0;i < 200;i++)
			{
				sum_gy += gyro_y1_win1[i];
				sum_gx += gyro_x1_win1[i];
			}
			biasx1 = sum_gx / 200;			biasy1 = sum_gy / 200;

		}
		integ_pitch1 += charreaddata.imu_Ts*(gyom1[1]-biasy1 ) * R2D;//无反馈修正
		pp_num1++;
#endif



		double ahrspitchout[2] = {0};
		double ahrspitchout1;
#if 1 //角度范围转换-180~180
		if (acc_pitch2 > 180)
		{
			acc_pitch2 = acc_pitch2 - 360;
		}
		if (acc_pitch2 <- 180)
		{
			acc_pitch2 = acc_pitch2 + 360;
		}
		if (integ_pitch1 > 180)
		{
			integ_pitch1 = integ_pitch1 - 360;
		}
		if (integ_pitch1 <- 180)
		{
			integ_pitch1 = integ_pitch1 + 360;
		}
#endif 
		double exter_angle = 0;
		double result=0;
		int flag1 = 0;
		//if (atd.bimu_updta == 1 && atd.bgnss_updata == 0 && atd.bcompass_updata == 0)//IMU有更新，无GNSS,无COMPASS
		//{
		//	flag1 = AngleTrack_Process(&atd.imu, &atd.config, exter_angle, &result);
		//}
		//if (atd.bimu_updta == 1 && atd.bgnss_updata == 1 && atd.bcompass_updata == 0)//IMU有更新，有GNSS,无COMPASS
		//{
		//	double gnss_heading = 0;
		//	int gnss_flag=GnssHeading_Process(&atd.gnss, &gnss_heading);
		//	flag1 = AngleTrack_Process(&atd.imu, &atd.config, gnss_heading, &result);
		//}
		//if (atd.bimu_updta == 1 && atd.bgnss_updata == 0 && atd.bcompass_updata == 1)//IMU有更新，无GNSS,有COMPASS
		//{
		//	double gnss_heading = 0;
		//	int gnss_flag = CompassHeading_Process(&atd.compass, &gnss_heading);
		//	flag1 = AngleTrack_Process(&atd.imu, &atd.config, gnss_heading, &result);
		//}
		AngleTrack_Process_Demo(&atd, &atd.config, &result);
		fprintf(fpp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			result*R2D, acc_pitch2, acc_roll2, integ_roll1, integ_pitch1,
			accm1[0], accm1[1], accm1[2], gyom1[0], gyom1[1], gyom1[2] , result*R2D);

	}
#endif
	fclose(fp);
	fclose(fpp);
	return 1;
}
