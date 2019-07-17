#include "AttTrack_lib.h"
#include "mahony.h"
#include "DateStruct.h"
static int process_init = 0;
static int bias_flag = 0;
int Mahony_Process(IMUdata_t* imu, MAGdata_t* magn, result_t* res)
{
	static struct MahonyProcess mp_t;
	double acc_b[3] = { 0 };
	double mag_n[3] = { 0 };
	if (!process_init)
	{
		mp_init(&mp_t);//�м�ṹ���ʼ��
		process_init = 1;
	}
	if (process_init==1)
	{
		if (!bias_flag)
		{
			init_process(imu, magn, &mp_t);//��Ԫ����ʼ��
			bias_flag = grobias(imu, &mp_t);//���ݼ�����ƫ
		}
		double smooth_acc[3] = { 0 };
		acc_filter(imu->acc,smooth_acc);//���ٶ�ƽ���˲�����
		//for (int i = 0;i < 3;i++)
		//{
		//	imu->acc[i] = smooth_acc[i];
		//}
		Accnorm(imu->acc, mp_t.acc_norm);//���ٶȹ�һ��
		AccCn2b(mp_t.q, acc_b);//n��׼���ٶ�ת����bϵ
		Accerr(mp_t.acc_norm, acc_b, mp_t.acc_err);//���ٶ����(������)
		mp_t.normacc = acc_norm(imu->acc);//���ż��ٶ�
		res->normacc = mp_t.normacc;
		//printf("acc:%f,%f,%f\n", imu->acc[0], imu->acc[1], imu->acc[2]);
		//printf("q:%f,%f,%f,%f\n",mp_t.q[0], mp_t.q[1], mp_t.q[2],mp_t.q[3]);
		mp_t.accinit = 1;
	}
	if (process_init == 1)
	{
		Magnorm(magn->mag_data, mp_t.mag_norm);//�����ƹ�һ��
		MagCb2n(magn->mag_data, mp_t.q, mag_n);//����nϵ�´�������
		Magerr(mag_n, magn->mag_data, mp_t.q, mp_t.mag_err);//���������(������)
		mp_t.maginit = 1;
		printf("amerr:%f,%f,%f\n", mp_t.acc_err[0], mp_t.acc_err[1], mp_t.acc_err[2]);
	}
	if (mp_t.accinit == 1 && mp_t.maginit == 1)
	{
		AccMag_err(mp_t.acc_err, mp_t.mag_err, mp_t.am_err);//���ٶ�����������֮��
		PIcontrol(imu->gyo, mp_t.acc_err,&mp_t);//PI�������������
		gyo2qua(mp_t.gyo_offset,imu->imu_time, mp_t.q);//һ�����������Ԫ������
		//gyo2qua(imu->gyo, imu->imu_time, mp_t.q);
		qua2att(mp_t.q, mp_t.att);//��Ԫ������ŷ����
		double smooth_acc1[3] = { 0 };
		acc_filter(imu->acc, smooth_acc1);
		angle_change(smooth_acc1, mp_t.att);//z����ٶ��ж��Ƿ�����90��
		res->roll = mp_t.att[0];//NED
		res->pitch = mp_t.att[1];
		res->heading= mp_t.att[2];
		//printf("result:%f,%f,%f\n", att[0]*R2D, att[1] * R2D, att[2] * R2D);
		return 1;
	}
	else
	{
		return 0;
	}

}