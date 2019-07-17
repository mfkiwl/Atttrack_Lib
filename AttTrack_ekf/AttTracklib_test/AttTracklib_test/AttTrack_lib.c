#include <stdio.h>
#include "ATProcess.h"
#include "AttTrack_lib.h"
#include "Calibrate.h"
#include "config.h"
int Angle_init = 0;
int DualAngle_init = 0;
int UniaxialAngle_init = 0;
int UniaxialAngle_init1 = 0;

int  AngleTrack_Process(ATdata_t* atdata, result_t* res)//通用接口，输出角度可选
{
	struct  AngleTrackData iatd;//原始数据处理结构体
	static struct  ATProcessSingleAngle atp;//中间数据处理结构体
	struct  AttInit aint;//角度初始化结构体
	struct  Cal_Install_Error Cal;//安装误差角结构体
	static struct  AttTrackCfg cfg;//配置文件结构体
	if (!Angle_init)
	{
		char outfilepath[] = FILE_SAVE_PATH;//数据保存路径
		fileopen(outfilepath);
		if (!init_config())//检查是否有配置文件，没有需要生成
		{
			generate_default_config_Angle();
		}
		get_config(&cfg);//获取配置文件
		calinstallerr_init(&Cal);//安装误差补偿结构体初始化
		ATPinit(&atp);//中间解码结构体初始化
		attInit(&aint);//静态零偏结构体初始化
		AngleTrackData_Init(&iatd);//数据结构体初始化
		Angle_init = 1;
	}

	decode_gasensor(&atdata->imu, &iatd);//IMU数据解码
	double accbias[3] = { 0 };
	acc_bias(&cfg, &atdata->imu,accbias);//加速度零偏处理
	atdata->imu.accx = accbias[0];
	atdata->imu.accy = accbias[1];
	atdata->imu.accz = accbias[2];
	//Install_Angle_Process(iatd,Cal);//安装误差标定和补偿
	if (!atp.battinit)
	{
		int finshattinit = 0;
		finshattinit = process_singleangle(&aint, &cfg,iatd.gyo3, iatd.acc3);
		if (finshattinit == 1)
		{
			atp.roll = aint.att[0];//车体横滚角
			atp.pitch = aint.att[1];//车体俯仰角
			if (atdata->bgnss_updata==1&&atdata->gnss.heading_flag == 0)
			{
				atp.heading = atdata->gnss.heading;
			}
			if (atdata->bgnss_updata == 1 && atdata->gnss.heading_flag == 1)
			{
				atp.heading = atan2(atdata->gnss.gnss_v[0],atdata->gnss.gnss_v[1]);
			}
			if (atdata->bcompass_updata == 1)
			{
				atp.heading = atan2(atdata->compass.my,atdata->compass.mx);
			}
			atp.gyobias[0] = aint.bias_gx;//x轴静态零偏
			atp.gyobias[1] = aint.bias_gy;//y轴静态零偏
			atp.gyobias[2] = aint.bias_gz;//z轴静态零偏
			config_set_gyo_staticbiasx(aint.bias_gx);//陀螺零偏写入配置文件
			config_set_gyo_staticbiasy(aint.bias_gy);
			config_set_gyo_staticbiasz(aint.bias_gz);
			a2mat_ned(aint.att, atp.Cb2n);
			m2qua_ned(atp.Cb2n, atp.qua);
			atp.battinit = 1;
		}
	}
	if (atp.battinit == 1)
	{
		if (atdata->Angle_flag == 1)//单轴角度跟踪
		{
			if (atdata->bgnss_updata)//GNSS+单轴陀螺融合 输出航向角
			{
				double ExterHeading = 0;
				int gnss_p = GnssHeading_Process(&atdata->gnss, &ExterHeading);
				process_gasensor_UE(&iatd, &cfg, ExterHeading, &atp);
			}
			else  //三轴加速度计+单轴陀螺融合 输出单轴角度
			{
				process_gasensor_U(&iatd, &cfg, &atp);
			}
			//结果结构体赋值
			res->pitch = atp.pitch;
		}
		if (atdata->Angle_flag == 2)//双轴角度跟踪
		{
			process_gasensor_D(&iatd, &cfg, &atp);
			//结果结构体赋值
			res->roll = atp.roll;
			res->pitch = atp.pitch;
		}
		if (atdata->Angle_flag == 3)//三轴角度跟踪
		{
			double ExterHeading = 0;
			if (atdata->bgnss_updata)//gnss航向跟踪
			{
				int gnss_p = GnssHeading_Process(&atdata->gnss, &ExterHeading);
			}
			if (atdata->bcompass_updata)//电子罗盘航向跟踪
			{
				int compass = CompassHeading_Process(&atdata->compass, &ExterHeading);
			}
			process_gasensor_3D(&iatd, &cfg, ExterHeading, &atp);
			//结果结构体赋值
			res->roll = atp.roll;
			res->pitch = atp.pitch;
			res->heading = atp.heading;
		}
	}
	AngleTrackData_Rest(&iatd);
	return 1;
}




int AngleTrack_Process_DualAngle(IMUdata_t* imu, double result[2])
{
	 struct  AngleTrackData iatd_D;//原始数据处理结构体
	 static struct  ATProcessSingleAngle atp_D;//中间数据处理结构体
	 struct  AttInit aint_D;//角度初始化结构体
	 struct  Cal_Install_Error Cal_D;//安装误差角结构体
	 struct  AttTrackCfg cfg_D;//配置文件结构体
	if (!DualAngle_init)
	{
		char outfilepath[] = FILE_SAVE_PATH;//数据保存路径
		fileopen(outfilepath);
		if (!init_config())//检查是否有配置文件，没有需要生成
		{
			generate_default_config_Angle();  //要改成对应传感器 
		}
		get_config(&cfg_D);//获取配置文件
		calinstallerr_init(&Cal_D);//安装误差补偿结构体初始化
		ATPinit(&atp_D);//中间解码结构体初始化
		attInit(&aint_D);//静态零偏结构体初始化
		AngleTrackData_Init(&iatd_D);//数据结构体初始化

		DualAngle_init = 1;

	}
	//Install_Angle_Process(iatd_D,Cal_D);//安装误差标定和补偿
	decode_gasensor(imu, &iatd_D);//数据解码
	if (!atp_D.battinit)
	{
		int finshattinit = 0;
		finshattinit =process_singleangle_D(&aint_D,iatd_D.gyo3, iatd_D.acc3);
		if (finshattinit==1)
		{
			atp_D.roll = aint_D.att[0];//车体横滚角
			atp_D.pitch = aint_D.att[1];//车体俯仰角
			atp_D.gyobias[0] = aint_D.bias_gx;//x轴静态零偏
			atp_D.gyobias[1] = aint_D.bias_gy;//y轴静态零偏
			a2mat_ned(aint_D.att, atp_D.Cb2n);
			m2qua_ned(atp_D.Cb2n, atp_D.qua);
			atp_D.battinit = 1;
		};
		//printf("init_data:%f,%f\n", atp_D.roll, atp_D.pitch);
	}

	if (atp_D.battinit==1)
	{
		process_gasensor_D(&iatd_D,&cfg_D,&atp_D);
	}
	//结果结构体赋值
	result[0] = atp_D.roll;
	result[1] = atp_D.pitch;
	AngleTrackData_Rest(&iatd_D);
	return 1;
}


int AngleTrack_Process_UniaxialAngle(IMUdata_t* imu, double* result)
{
	static struct  AngleTrackData iatd_U;
	static struct  ATProcessSingleAngle atp_U;
	static struct  AttInit aint_U;
	struct  Cal_Install_Error Cal_U;
	struct  AttTrackCfg cfg_U;//配置文件结构体
	if(!UniaxialAngle_init)
	{
		char outfilepath[] = FILE_SAVE_PATH;
		fileopen(outfilepath);
		if (!init_config())//检查是否有配置文件，没有需要生成
		{
			generate_default_config_Angle();  //要改成对应传感器 
		}
		calinstallerr_init(&Cal_U);//安装误差补偿结构体初始化
		ATPinit(&atp_U);//中间解码结构体初始化
		attInit(&aint_U);//静态零偏结构体初始化
		AngleTrackData_Init(&iatd_U);//数据结构体初始化
		UniaxialAngle_init=1;
	}
	//Install_Angle_Process(iatd_U, Cal_U);//安装误差标定和补偿
	decode_gasensor(imu, &iatd_U);
	if (!atp_U.battinit)
	{
		int finshattinit = 0;
		finshattinit = process_singleangle_U(&aint_U, iatd_U.gyo3, iatd_U.acc3);
		if (finshattinit == 1)
		{
			atp_U.pitch = aint_U.att[1];
			atp_U.integ_pitch=aint_U.att[1];
			atp_U.gyobias[1] = aint_U.bias_gy;
			atp_U.battinit = 1;
		}
		//printf("init_data:%f,%f\n", atp_U.pitch, atp_U.gyobias[1]);
	}
	//printf("init_data:%f,%f\n", atp_U.pitch, atp_U.gyobias[1]);
	if (atp_U.battinit == 1)
	{
		process_gasensor_U(&iatd_U, &cfg_U, &atp_U);
	}
	//结果结构体赋值
	*result = atp_U.pitch;
	AngleTrackData_Rest(&iatd_U);
	return 1;
}


int  AngleTrack_Process_UEniaxialAngle(IMUdata_t* imu, double ExterAngle, double* result)
{
	struct  AngleTrackData iatd_UE;
	struct  ATProcessSingleAngle atp_UE;
	struct  AttInit aint_UE;
	struct  Cal_Install_Error Cal_UE;
	struct  AttTrackCfg cfg_UE;//配置文件结构体
	if (!UniaxialAngle_init1)
	{
		char outfilepath[] = FILE_SAVE_PATH;
		fileopen(outfilepath);
		calinstallerr_init(&Cal_UE);//安装误差补偿结构体初始化
		ATPinit(&atp_UE);//中间解码结构体初始化
		attInit(&aint_UE);//静态零偏结构体初始化
		AngleTrackData_Init(&iatd_UE);//数据结构体初始化
		UniaxialAngle_init1 = 1;
	}
	Install_Angle_Process(iatd_UE, Cal_UE);//安装误差标定和补偿
	decode_gasensor(imu, &iatd_UE);
	if (!atp_UE.battinit)
	{
		int finshattinit = 0;
		finshattinit = process_singleangle_UE(&aint_UE, iatd_UE.gyo3);
		if (finshattinit == 1)
		{
			atp_UE.pitch = ExterAngle;//外部角度 deg
			atp_UE.integ_pitch= aint_UE.att[1];
			atp_UE.gyobias[1] = aint_UE.bias_gy;
			atp_UE.tpre = iatd_UE.imutime - atp_UE.dt;
			Mequalm(iatd_UE.acc3, 3, 1, atp_UE.accpre);
			Mequalm(iatd_UE.gyo3, 3, 1, atp_UE.gyopre);
			atp_UE.battinit = 1;
		}
	}
	if (atp_UE.battinit == 1)
	{
		process_gasensor_UE(&iatd_UE, &cfg_UE, ExterAngle, &atp_UE);
	}
	//结果结构体赋值
		*result = atp_UE.pitch;
		AngleTrackData_Rest(&iatd_UE);
		return 1;
}

int  AngleTrack_Process_3DAngle(IMUdata_t* imu, double ExterHeading, double result[3])
{
	static struct  AngleTrackData iatd_3D;
	static struct  ATProcessSingleAngle atp_3D;
	static struct  AttInit aint_3D;
	static struct  Cal_Install_Error Cal_3D;
	struct  AttTrackCfg cfg_3D;//配置文件结构体
	if (!atp_3D.bprocessinit)
	{
		char outfilepath[] = FILE_SAVE_PATH;
		fileopen(outfilepath);
		calinstallerr_init(&Cal_3D);//安装误差补偿结构体初始化
		ATPinit(&atp_3D);//中间解码结构体初始化
		attInit(&aint_3D);//静态零偏结构体初始化
		AngleTrackData_Init(&iatd_3D);//数据结构体初始化
		atp_3D.bprocessinit = 1;
	}
	Install_Angle_Process(iatd_3D, Cal_3D);//安装误差标定和补偿
	decode_gasensor(imu, &iatd_3D);
	if (!atp_3D.battinit)
	{
		int finshattinit = 0;
		finshattinit = process_singleangle_3D(&aint_3D, iatd_3D.gyo3, iatd_3D.acc3);
		if (finshattinit == 1)
		{
			atp_3D.roll = aint_3D.att[0];//横滚角
			atp_3D.pitch = aint_3D.att[1];//俯仰角
			atp_3D.heading = ExterHeading;//航向角
			atp_3D.gyobias[0] = aint_3D.bias_gx;//x轴静态零偏
			atp_3D.gyobias[1] = aint_3D.bias_gy;//y轴静态零偏
			atp_3D.gyobias[2] = aint_3D.bias_gz;//z轴静态零偏
			a2mat_ned(aint_3D.att, atp_3D.Cb2n);
			m2qua_ned(atp_3D.Cb2n, atp_3D.qua);
			atp_3D.battinit = 1;
		}
	}
	if (atp_3D.battinit == 1)
	{
		process_gasensor_3D(&iatd_3D, &cfg_3D,ExterHeading, &atp_3D);
	}
	//结果结构体赋值
	result[0] = atp_3D.roll;
	result[1] = atp_3D.pitch;
	result[2] = atp_3D.heading;
	AngleTrackData_Rest(&iatd_3D);
	return 1;
}

