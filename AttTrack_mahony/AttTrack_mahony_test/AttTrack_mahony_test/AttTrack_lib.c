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
		mp_init(&mp_t);//中间结构体初始化
		process_init = 1;
	}
	if (process_init==1)
	{
		if (!bias_flag)
		{
			init_process(imu, magn, &mp_t);//四元数初始化
			bias_flag = grobias(imu, &mp_t);//陀螺计算零偏
		}
		double smooth_acc[3] = { 0 };
		acc_filter(imu->acc,smooth_acc);//加速度平滑滤波处理
		//for (int i = 0;i < 3;i++)
		//{
		//	imu->acc[i] = smooth_acc[i];
		//}
		Accnorm(imu->acc, mp_t.acc_norm);//加速度归一化
		AccCn2b(mp_t.q, acc_b);//n标准加速度转换到b系
		Accerr(mp_t.acc_norm, acc_b, mp_t.acc_err);//加速度误差(向量积)
		mp_t.normacc = acc_norm(imu->acc);//干扰加速度
		res->normacc = mp_t.normacc;
		//printf("acc:%f,%f,%f\n", imu->acc[0], imu->acc[1], imu->acc[2]);
		//printf("q:%f,%f,%f,%f\n",mp_t.q[0], mp_t.q[1], mp_t.q[2],mp_t.q[3]);
		mp_t.accinit = 1;
	}
	if (process_init == 1)
	{
		Magnorm(magn->mag_data, mp_t.mag_norm);//磁力计归一化
		MagCb2n(magn->mag_data, mp_t.q, mag_n);//计算n系下磁力分量
		Magerr(mag_n, magn->mag_data, mp_t.q, mp_t.mag_err);//磁力计误差(向量积)
		mp_t.maginit = 1;
		printf("amerr:%f,%f,%f\n", mp_t.acc_err[0], mp_t.acc_err[1], mp_t.acc_err[2]);
	}
	if (mp_t.accinit == 1 && mp_t.maginit == 1)
	{
		AccMag_err(mp_t.acc_err, mp_t.mag_err, mp_t.am_err);//加速度与磁力计误差之和
		PIcontrol(imu->gyo, mp_t.acc_err,&mp_t);//PI控制器调整误差
		gyo2qua(mp_t.gyo_offset,imu->imu_time, mp_t.q);//一阶龙哥库塔四元数更新
		//gyo2qua(imu->gyo, imu->imu_time, mp_t.q);
		qua2att(mp_t.q, mp_t.att);//四元数计算欧拉角
		double smooth_acc1[3] = { 0 };
		acc_filter(imu->acc, smooth_acc1);
		angle_change(smooth_acc1, mp_t.att);//z轴加速度判断是否超正负90度
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