#include <stdio.h>
#include "ATProcess.h"
#include "AttTrack_lib.h"
#include "Calibrate.h"
#include "config.h"
int Angle_init = 0;
int DualAngle_init = 0;
int UniaxialAngle_init = 0;
int UniaxialAngle_init1 = 0;

int  AngleTrack_Process(ATdata_t* atdata, result_t* res)//ͨ�ýӿڣ�����Ƕȿ�ѡ
{
	struct  AngleTrackData iatd;//ԭʼ���ݴ���ṹ��
	static struct  ATProcessSingleAngle atp;//�м����ݴ���ṹ��
	struct  AttInit aint;//�Ƕȳ�ʼ���ṹ��
	struct  Cal_Install_Error Cal;//��װ���ǽṹ��
	static struct  AttTrackCfg cfg;//�����ļ��ṹ��
	if (!Angle_init)
	{
		char outfilepath[] = FILE_SAVE_PATH;//���ݱ���·��
		fileopen(outfilepath);
		if (!init_config())//����Ƿ��������ļ���û����Ҫ����
		{
			generate_default_config_Angle();
		}
		get_config(&cfg);//��ȡ�����ļ�
		calinstallerr_init(&Cal);//��װ�����ṹ���ʼ��
		ATPinit(&atp);//�м����ṹ���ʼ��
		attInit(&aint);//��̬��ƫ�ṹ���ʼ��
		AngleTrackData_Init(&iatd);//���ݽṹ���ʼ��
		Angle_init = 1;
	}

	decode_gasensor(&atdata->imu, &iatd);//IMU���ݽ���
	double accbias[3] = { 0 };
	acc_bias(&cfg, &atdata->imu,accbias);//���ٶ���ƫ����
	atdata->imu.accx = accbias[0];
	atdata->imu.accy = accbias[1];
	atdata->imu.accz = accbias[2];
	//Install_Angle_Process(iatd,Cal);//��װ���궨�Ͳ���
	if (!atp.battinit)
	{
		int finshattinit = 0;
		finshattinit = process_singleangle(&aint, &cfg,iatd.gyo3, iatd.acc3);
		if (finshattinit == 1)
		{
			atp.roll = aint.att[0];//��������
			atp.pitch = aint.att[1];//���帩����
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
			atp.gyobias[0] = aint.bias_gx;//x�ᾲ̬��ƫ
			atp.gyobias[1] = aint.bias_gy;//y�ᾲ̬��ƫ
			atp.gyobias[2] = aint.bias_gz;//z�ᾲ̬��ƫ
			config_set_gyo_staticbiasx(aint.bias_gx);//������ƫд�������ļ�
			config_set_gyo_staticbiasy(aint.bias_gy);
			config_set_gyo_staticbiasz(aint.bias_gz);
			a2mat_ned(aint.att, atp.Cb2n);
			m2qua_ned(atp.Cb2n, atp.qua);
			atp.battinit = 1;
		}
	}
	if (atp.battinit == 1)
	{
		if (atdata->Angle_flag == 1)//����Ƕȸ���
		{
			if (atdata->bgnss_updata)//GNSS+���������ں� ��������
			{
				double ExterHeading = 0;
				int gnss_p = GnssHeading_Process(&atdata->gnss, &ExterHeading);
				process_gasensor_UE(&iatd, &cfg, ExterHeading, &atp);
			}
			else  //������ٶȼ�+���������ں� �������Ƕ�
			{
				process_gasensor_U(&iatd, &cfg, &atp);
			}
			//����ṹ�帳ֵ
			res->pitch = atp.pitch;
		}
		if (atdata->Angle_flag == 2)//˫��Ƕȸ���
		{
			process_gasensor_D(&iatd, &cfg, &atp);
			//����ṹ�帳ֵ
			res->roll = atp.roll;
			res->pitch = atp.pitch;
		}
		if (atdata->Angle_flag == 3)//����Ƕȸ���
		{
			double ExterHeading = 0;
			if (atdata->bgnss_updata)//gnss�������
			{
				int gnss_p = GnssHeading_Process(&atdata->gnss, &ExterHeading);
			}
			if (atdata->bcompass_updata)//�������̺������
			{
				int compass = CompassHeading_Process(&atdata->compass, &ExterHeading);
			}
			process_gasensor_3D(&iatd, &cfg, ExterHeading, &atp);
			//����ṹ�帳ֵ
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
	 struct  AngleTrackData iatd_D;//ԭʼ���ݴ���ṹ��
	 static struct  ATProcessSingleAngle atp_D;//�м����ݴ���ṹ��
	 struct  AttInit aint_D;//�Ƕȳ�ʼ���ṹ��
	 struct  Cal_Install_Error Cal_D;//��װ���ǽṹ��
	 struct  AttTrackCfg cfg_D;//�����ļ��ṹ��
	if (!DualAngle_init)
	{
		char outfilepath[] = FILE_SAVE_PATH;//���ݱ���·��
		fileopen(outfilepath);
		if (!init_config())//����Ƿ��������ļ���û����Ҫ����
		{
			generate_default_config_Angle();  //Ҫ�ĳɶ�Ӧ������ 
		}
		get_config(&cfg_D);//��ȡ�����ļ�
		calinstallerr_init(&Cal_D);//��װ�����ṹ���ʼ��
		ATPinit(&atp_D);//�м����ṹ���ʼ��
		attInit(&aint_D);//��̬��ƫ�ṹ���ʼ��
		AngleTrackData_Init(&iatd_D);//���ݽṹ���ʼ��

		DualAngle_init = 1;

	}
	//Install_Angle_Process(iatd_D,Cal_D);//��װ���궨�Ͳ���
	decode_gasensor(imu, &iatd_D);//���ݽ���
	if (!atp_D.battinit)
	{
		int finshattinit = 0;
		finshattinit =process_singleangle_D(&aint_D,iatd_D.gyo3, iatd_D.acc3);
		if (finshattinit==1)
		{
			atp_D.roll = aint_D.att[0];//��������
			atp_D.pitch = aint_D.att[1];//���帩����
			atp_D.gyobias[0] = aint_D.bias_gx;//x�ᾲ̬��ƫ
			atp_D.gyobias[1] = aint_D.bias_gy;//y�ᾲ̬��ƫ
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
	//����ṹ�帳ֵ
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
	struct  AttTrackCfg cfg_U;//�����ļ��ṹ��
	if(!UniaxialAngle_init)
	{
		char outfilepath[] = FILE_SAVE_PATH;
		fileopen(outfilepath);
		if (!init_config())//����Ƿ��������ļ���û����Ҫ����
		{
			generate_default_config_Angle();  //Ҫ�ĳɶ�Ӧ������ 
		}
		calinstallerr_init(&Cal_U);//��װ�����ṹ���ʼ��
		ATPinit(&atp_U);//�м����ṹ���ʼ��
		attInit(&aint_U);//��̬��ƫ�ṹ���ʼ��
		AngleTrackData_Init(&iatd_U);//���ݽṹ���ʼ��
		UniaxialAngle_init=1;
	}
	//Install_Angle_Process(iatd_U, Cal_U);//��װ���궨�Ͳ���
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
	//����ṹ�帳ֵ
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
	struct  AttTrackCfg cfg_UE;//�����ļ��ṹ��
	if (!UniaxialAngle_init1)
	{
		char outfilepath[] = FILE_SAVE_PATH;
		fileopen(outfilepath);
		calinstallerr_init(&Cal_UE);//��װ�����ṹ���ʼ��
		ATPinit(&atp_UE);//�м����ṹ���ʼ��
		attInit(&aint_UE);//��̬��ƫ�ṹ���ʼ��
		AngleTrackData_Init(&iatd_UE);//���ݽṹ���ʼ��
		UniaxialAngle_init1 = 1;
	}
	Install_Angle_Process(iatd_UE, Cal_UE);//��װ���궨�Ͳ���
	decode_gasensor(imu, &iatd_UE);
	if (!atp_UE.battinit)
	{
		int finshattinit = 0;
		finshattinit = process_singleangle_UE(&aint_UE, iatd_UE.gyo3);
		if (finshattinit == 1)
		{
			atp_UE.pitch = ExterAngle;//�ⲿ�Ƕ� deg
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
	//����ṹ�帳ֵ
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
	struct  AttTrackCfg cfg_3D;//�����ļ��ṹ��
	if (!atp_3D.bprocessinit)
	{
		char outfilepath[] = FILE_SAVE_PATH;
		fileopen(outfilepath);
		calinstallerr_init(&Cal_3D);//��װ�����ṹ���ʼ��
		ATPinit(&atp_3D);//�м����ṹ���ʼ��
		attInit(&aint_3D);//��̬��ƫ�ṹ���ʼ��
		AngleTrackData_Init(&iatd_3D);//���ݽṹ���ʼ��
		atp_3D.bprocessinit = 1;
	}
	Install_Angle_Process(iatd_3D, Cal_3D);//��װ���궨�Ͳ���
	decode_gasensor(imu, &iatd_3D);
	if (!atp_3D.battinit)
	{
		int finshattinit = 0;
		finshattinit = process_singleangle_3D(&aint_3D, iatd_3D.gyo3, iatd_3D.acc3);
		if (finshattinit == 1)
		{
			atp_3D.roll = aint_3D.att[0];//�����
			atp_3D.pitch = aint_3D.att[1];//������
			atp_3D.heading = ExterHeading;//�����
			atp_3D.gyobias[0] = aint_3D.bias_gx;//x�ᾲ̬��ƫ
			atp_3D.gyobias[1] = aint_3D.bias_gy;//y�ᾲ̬��ƫ
			atp_3D.gyobias[2] = aint_3D.bias_gz;//z�ᾲ̬��ƫ
			a2mat_ned(aint_3D.att, atp_3D.Cb2n);
			m2qua_ned(atp_3D.Cb2n, atp_3D.qua);
			atp_3D.battinit = 1;
		}
	}
	if (atp_3D.battinit == 1)
	{
		process_gasensor_3D(&iatd_3D, &cfg_3D,ExterHeading, &atp_3D);
	}
	//����ṹ�帳ֵ
	result[0] = atp_3D.roll;
	result[1] = atp_3D.pitch;
	result[2] = atp_3D.heading;
	AngleTrackData_Rest(&iatd_3D);
	return 1;
}

