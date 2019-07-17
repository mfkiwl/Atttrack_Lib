#include <stdio.h>
#include "ATProcess.h"
#include "AttTrack_lib.h"
#include "Calibrate.h"
#include "config.h"
int Angle_init = 0;
int external_init = 0;

int  AngleTrack_Process(Config_t* conf, double gyo, double IMU_time, double ExterAngle, double* result)//陀螺仪与外部角度融合，输出单轴角度
{
	static struct  ATProcessSingleAngle atp;//中间数据处理结构体
	if (!Angle_init)
	{
		char outfilepath[] = FILE_SAVE_PATH;//数据保存路径
		fileopen(outfilepath);
		if (!init_config())//检查是否有配置文件，没有需要生成
		{
			generate_default_config_Angle();
		}
		get_conf(conf);//获取配置文件
		ATPinit(&atp);//中间解码结构体初始化
		Angle_init = 1;
	}
	if (!atp.battinit)
	{
		int biasflag=process_gyobias(conf, gyo, atp.gyobias[0]);
		double equal_ExterAngle = 0;
		int anglemeanflag= angle_equal(ExterAngle, &equal_ExterAngle);
		atp.pitch = equal_ExterAngle;
		if (biasflag == 1)//零偏计算完成
		{
			atp.battinit = 1;
		}
	}
	if (atp.battinit == 1)
	{
		int flag=process_AttTrack(&atp, conf, gyo, IMU_time, ExterAngle);
			*result = atp.pitch;
	}
	return 1;
}


int AngleTrack_Process_Demo(ATdata_t *atd, Config_t* conf, double* result)
{
	double res=0;
	double ExterAngle = 0;
		int acc_flag = 0;
		int gnss_flag = 0;
		int compass_flag = 0;
		if (conf->angle_flag == 1 && atd->bimu_updta == 1)//陀螺仪+加速度计
		{
			acc_flag = IMU_AccAngle_Process(&atd->imu,conf, &ExterAngle);
		}
		if (conf->angle_flag == 2 &&atd->bimu_updta==1&& atd->bgnss_updata == 1 && atd->bcompass_updata == 0)//陀螺仪+gnss
		{
			gnss_flag = GnssHeading_Process(&atd->gnss, &ExterAngle);//单天线计算航向角
		}
		if (conf->angle_flag == 2 && atd->bimu_updta == 1&& atd->bgnss_updata == 0 && atd->bcompass_updata == 1)//陀螺仪+磁力计
		{
			compass_flag = CompassHeading_Process(&atd->compass, &ExterAngle);//磁力计计算航向角
		}
		//printf("ExterAngle:%f\n", ExterAngle);
		int angle_flag = AngleTrack_Process(conf, atd->imu.gyoy, atd->imu.imutimetarget, ExterAngle, &res);//陀螺仪与外部角度融合，输出单轴角度
		*result = res;
		return 1;
}
