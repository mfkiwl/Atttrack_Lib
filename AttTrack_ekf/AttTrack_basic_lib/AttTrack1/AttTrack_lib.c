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

