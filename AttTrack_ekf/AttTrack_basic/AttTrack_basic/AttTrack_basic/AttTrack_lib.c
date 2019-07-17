#include <stdio.h>
#include "ATProcess.h"
#include "AttTrack_lib.h"
#include "Calibrate.h"
#include "config.h"
int Angle_init = 0;
int external_init = 0;

int  AngleTrack_Process(Config_t* conf, double gyo, double IMU_time, double ExterAngle, double* result)//���������ⲿ�Ƕ��ںϣ��������Ƕ�
{
	static struct  ATProcessSingleAngle atp;//�м����ݴ���ṹ��
	if (!Angle_init)
	{
		char outfilepath[] = FILE_SAVE_PATH;//���ݱ���·��
		fileopen(outfilepath);
		if (!init_config())//����Ƿ��������ļ���û����Ҫ����
		{
			generate_default_config_Angle();
		}
		get_conf(conf);//��ȡ�����ļ�
		ATPinit(&atp);//�м����ṹ���ʼ��
		Angle_init = 1;
	}
	if (!atp.battinit)
	{
		int biasflag=process_gyobias(conf, gyo, atp.gyobias[0]);
		double equal_ExterAngle = 0;
		int anglemeanflag= angle_equal(ExterAngle, &equal_ExterAngle);
		atp.pitch = equal_ExterAngle;
		if (biasflag == 1)//��ƫ�������
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
		if (conf->angle_flag == 1 && atd->bimu_updta == 1)//������+���ٶȼ�
		{
			acc_flag = IMU_AccAngle_Process(&atd->imu,conf, &ExterAngle);
		}
		if (conf->angle_flag == 2 &&atd->bimu_updta==1&& atd->bgnss_updata == 1 && atd->bcompass_updata == 0)//������+gnss
		{
			gnss_flag = GnssHeading_Process(&atd->gnss, &ExterAngle);//�����߼��㺽���
		}
		if (conf->angle_flag == 2 && atd->bimu_updta == 1&& atd->bgnss_updata == 0 && atd->bcompass_updata == 1)//������+������
		{
			compass_flag = CompassHeading_Process(&atd->compass, &ExterAngle);//�����Ƽ��㺽���
		}
		//printf("ExterAngle:%f\n", ExterAngle);
		int angle_flag = AngleTrack_Process(conf, atd->imu.gyoy, atd->imu.imutimetarget, ExterAngle, &res);//���������ⲿ�Ƕ��ںϣ��������Ƕ�
		*result = res;
		return 1;
}
