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

