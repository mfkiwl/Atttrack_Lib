#include "AttTrack_lib.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "DATAread.h"
#include "mahony.h"
double gyro_y1_win1[200] = { 0 };
double gyro_x1_win1[200] = { 0 };
double gyro_y1_win[200] = { 0 };
double gyro_x1_win[200] = { 0 };
void main()
{
	int pp_num1 = 0;
	FILE *fp;
	FILE *fpp;
	struct MAGdata magdata;
	struct result res;
	struct chardata charreaddata;
	struct IMUdata mahonyimudata;
	//struct AngleTrackData ATD1;
	double integ_pitch1 = 0;
	double integ_roll1 = 0;
	static double biasx1 = 0;
	static double biasy1 = 0;
	int initstruct = 0;
	static double biasx = 0;
	static double biasy = 0;
	//fp = fopen("C:/Users/huace/Desktop/2.22/gadata03.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/2.22/compass1/gadatac02.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/301/tmp-w1.txt", "rt");
	//fp = fopen("E:/vs_project/AHRS/ahrs/data/GA-C2-1226.txt", "rt");
	fp = fopen("C:/Users/huace/Desktop/304/boom02.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/304/dogbone01.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/304/stick03.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/304/body03.txt", "rt");
	//fp = fopen("C:/Users/huace/Desktop/304/D-data-2.txt", "rt");
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
	//fpp = fopen("C:/Users/huace/Desktop/304/dogbone03out.txt", "wt");
	fpp = fopen("C:/Users/huace/Desktop/304/mahonyout.txt", "wt");
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
		//BDFdataread(&charreaddata, fp);
		//GAsensordataread(&charreaddata, fp);//12.26gasensor数据
		double accm1[3] = { 0 }, gyom1[3] = { 0 };
#if 1
		accm1[0] = charreaddata.acc_data[0] * 9.80665; //三轴加速度计(坐标系统一 （NED）)
		accm1[1] = charreaddata.acc_data[1] * 9.80665;
		accm1[2] = charreaddata.acc_data[2] * 9.80665;
		gyom1[0] = 0;//(坐标系统一 （NED）)
		gyom1[1] = charreaddata.gyro_data[1];
		gyom1[2] = 0;
		mahonyimudata.imu_time = charreaddata.imu_Ts;
#endif
#if 0
		accm1[0] = charreaddata.acc_data[0] * 9.80665; //三轴加速度计(坐标系统一 （NED）)
		accm1[1] = charreaddata.acc_data[1 ]* 9.80665;
		accm1[2] = charreaddata.acc_data[2] * 9.80665;
		gyom1[0] = charreaddata.gyro_data[0];//(坐标系统一 （NED）)
		gyom1[1] = charreaddata.gyro_data[1];
		gyom1[2] = charreaddata.gyro_data[2];
		mahonyimudata.imu_time = 0.005;
#endif

		//ahrsimudata.imutimetarget = 0.005;
		mahonyimudata.acc[0] = accm1[0];
		mahonyimudata.acc[1] = accm1[1];
		mahonyimudata.acc[2] = accm1[2];
		mahonyimudata.gyo[0] = gyom1[0] ;
		mahonyimudata.gyo[1] = -gyom1[1];
		mahonyimudata.gyo[2] = gyom1[2] ;


		///****************************加速度计计算姿态角*******************************/
		 acc_roll2 = atan((accm1[1])/(accm1[2]))*R2D;//ned系
		 acc_pitch2 = atan2(accm1[0],accm1[2])*R2D;//三轴加速度计算俯仰角
		 //acc_pitch2 = atan(accm1[0]/(-accm1[2]))*R2D;//三轴加速度计算俯仰角

		 //陀螺仪积分
		 static double sum_gx = 0;
		 static double sum_gy = 0;
		 if (pp_num1<200)//窗口数据初始化
		 {
			 //if (fabs(gyom1[2])>0.02)
			 //{
			 //	gyom1[2] = 0;
			 //}
			 gyro_y1_win[pp_num1] = gyom1[1];
			 gyro_x1_win[pp_num1] = gyom1[0];
			 integ_pitch1 = acc_pitch2;
			 integ_roll1 = acc_roll2;

		 }
		 else if (pp_num1 == 200)
		 {

			 for (int i = 0;i < 200;i++)
			 {
				 sum_gy += gyro_y1_win[i];
				 sum_gx += gyro_x1_win[i];
			 }
			 biasx = sum_gx / 200;
			 biasy = sum_gy / 200;
			 //bias = getmean(gyro_y1_win, 500);

	}
		 integ_pitch1 += mahonyimudata.imu_time*(gyom1[1] - biasy) * R2D;//无反馈修正
		 integ_roll1 += mahonyimudata.imu_time*(gyom1[0] - biasx) * R2D;//无反馈修正
		 pp_num1++;

#if 0 //角度范围转换-180~180
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


		//flag1 = AngleTrack_Process_DualAngle(&ahrsimudata, ahrspitchout);
		//flag1 = AngleTrack_Process_UniaxialAngle(&ahrsimudata, &ahrspitchout1);

		magdata.mag_data[0] = 0;
		magdata.mag_data[1] = 0;
		magdata.mag_data[2] = 0;
		int flag1= Mahony_Process(&mahonyimudata, &magdata, &res);
		//printf("data:%f,%f,%f,%f,%f\n",
		//	 acc_roll2, acc_pitch2,res.roll*R2D,res.pitch*R2D,res.heading*R2D);
		fprintf(fpp,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", accm1[0], accm1[1], accm1[2],gyom1[0], gyom1[1], gyom1[2],
			res.roll*R2D, res.pitch*R2D,acc_roll2, acc_pitch2, integ_roll1 ,integ_pitch1,res.normacc);
	}
#endif
	fclose(fp);
	fclose(fpp);
}
