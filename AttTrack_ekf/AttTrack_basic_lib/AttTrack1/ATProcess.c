#include "ATProcess.h"
 static double acc_x_win2[5] = { 0 };
 static double acc_y_win2[5] = { 0 };
 static double acc_z_win2[5] = { 0 };
int NUM_acc2 = 0;
int extern zero_num;
static double static_accpitch=0;

/****************挖土机单轴角度跟踪********************/
void ProcessSingleAngle(struct ATProcessSingleAngle* atp)
{
	atp->battinit = 0;
	atp->bkfinit = 0;
	atp->bprocessinit = 0;
	atp->dt = 0.02;
	atp->tpre = 0;
	atp->pitch = 0;

	atp->num_accnorm = 0;
	atp->num_gyonorm = 0;
	for (int i = 0; i < 3; i++)
	{
		atp->gyobias[i] = 0.0;
		atp->gyopre[i] = 0.0;
		atp->accpre[i] = 0.0;
	}
}

void ATPinit(struct ATProcessSingleAngle* atp)
{
	atp->battinit = 0;
	atp->bprocessinit = 0;
	atp->bkfinit = 0;
	atp->num_accnorm = 0;
	atp->pitch=0;
	atp->integ_pitch=0;
	atp->roll=0;

}


/**********************************************************/
int process_gasensor(struct AngleTrackData* iatd, struct Config* cfg, struct ATProcessSingleAngle* atp, double ExterAngle)
{
	static struct  kalmanfilter kft2;
	double roll_acc = 0;
	double pitch_acc = 0;
	if (!atp->bkfinit)
	{
		/*********水平角度跟踪滤波器初始化********/
		//kalmanfilterinit(kft2, 2, 1);
		InitialMatrix_xk(&kft2);
		InitialMatrix_Pxk(&kft2);
		InitialMatrix_Phi(&kft2);
		InitialMatrix_Qk(&kft2);
		InitialMatrix_Hk(&kft2);
		InitialMatrix_Rk(&kft2);
		InitialMatrix_zk(&kft2);
		double dxk0[2] = { 0.01*D2R,0.1*D2R };
		setxk(&kft2, dxk0);
		setPxk(&kft2, dxk0);
		double noise_imuy[2] = { cfg->gnoisex*D2R,cfg->gstabilityx*D2R / 3600.0 };//0.025,8.0(GAsensor ------scc2230)
		setQk(&kft2, noise_imuy);
		atp->bkfinit = 1;
	}
#if 1
	double dtime = iatd->imutime;
	double gyo_bias[3] = { 0 };
	double accmean[3] = { 0 }, gyomean[3] = { 0 };
	Maddn(atp->accpre, iatd->acc3, accmean, 3, 1);
	Maddn(atp->gyopre, iatd->gyo3, gyomean, 3, 1);
	Mmul(accmean, 3, 1, 0.5);
	Mmul(gyomean, 3, 1, 0.5);
	Mequalm(iatd->acc3, 3, 1, atp->accpre);
	Mequalm(iatd->gyo3, 3, 1, atp->gyopre);
	Mminn(gyomean, atp->gyobias, gyo_bias, 3, 1); //陀螺数据取均值										 
	atp->pitch += iatd->imutime*(iatd->gyo3[1] - atp->gyobias[1]);//陀螺积分求俯仰角
	setPhi(&kft2, dtime);
	TUpdate(&kft2, dtime);
	double noise_pitch[1] = { 0 };
	double normacc = 0;
	if (cfg->angle_flag == 1)
	{
		/************加速度计开窗平滑，滤除高频信号***********/
		if (cfg->ekf_strategy == 1) //加速度平滑滤波，量测噪声取定值
		{
			double smooth_ax = 0, smooth_ay = 0, smooth_az = 0, smooth_gz = 0;
			if (NUM_acc2 < WinLen)
			{
				acc_x_win2[NUM_acc2] = iatd->acc3[0];
				acc_y_win2[NUM_acc2] = iatd->acc3[1];
				acc_z_win2[NUM_acc2] = iatd->acc3[2];
			}
			else
			{
				for (int i = 0;i < WinLen;i++)
				{
					acc_x_win2[i] = acc_x_win2[i + 1];
					acc_y_win2[i] = acc_y_win2[i + 1];
					acc_z_win2[i] = acc_z_win2[i + 1];
				}
				acc_x_win2[WinLen - 1] = iatd->acc3[0];
				acc_y_win2[WinLen - 1] = iatd->acc3[1];
				acc_z_win2[WinLen - 1] = iatd->acc3[2];
				smooth_ax = dataFilter(acc_x_win2, WinLen);
				smooth_ay = dataFilter(acc_y_win2, WinLen);
				smooth_az = dataFilter(acc_z_win2, WinLen);
				//roll_acc = atan2(-smooth_ay, -smooth_az);
				//pitch_acc = atan2(smooth_ax, sqrt(smooth_ay * smooth_ay + smooth_az * smooth_az));
				pitch_acc = atan2(smooth_ax, (-smooth_az));//双轴加速度计算俯仰角
			}
			noise_pitch[0] = 0.5*D2R;
			normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
			if (normacc > 0.1)//加速度超限处理
			{
				return 1;
			}
		}
		/************加速度计均值***********/
		if (cfg->ekf_strategy == 2)//加速度取均值，量测噪声动静态区分
		{
			//	pitch_acc = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));
			pitch_acc = atan2(accmean[0], (-accmean[2]));
			//误差传递公式(与干扰加速度大小相关)
			/*******************噪声动静态区分*************/
			normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
			//double normacc1 = fabs((pitch_acc - static_accpitch)*R2D);
			if (normacc < 0.02)  //10mg
			{
				atp->num_accnorm++;
			}
			else
			{
				atp->num_accnorm = 0;
			}
			if (atp->num_accnorm > 100)  //准静态超过1秒
			{
				noise_pitch[0] = 0.3*D2R;
				//	static_accpitch = pitch_acc;
				/********增加静态的处理，加计开窗平滑*********/
			}
			else
			{
				noise_pitch[0] = (0.5 + (normacc - 0.02) * 10)*D2R;
				//noise_pitch[0] = (0.5 + normacc1* 1.5)*D2R;
			}
			if (normacc > 0.1)
			{
				//printf("干扰加速度大！\n");
				return 1;
			}
		}
	}
	if (cfg->angle_flag == 2)
	{
		pitch_acc = ExterAngle;
		noise_pitch[0] = cfg->gnssyawvar*D2R;
	}
	double zk_pitch[1] = { 0 };
	if (atp->pitch> PI)//量测值出现异常处理
	{
		atp->pitch = atp->pitch - 2 * PI;
	}
	if (atp->pitch<-PI)//量测值出现异常处理
	{
		atp->pitch = atp->pitch + 2 * PI;
	}
	zk_pitch[0] = atp->pitch - pitch_acc;
	if (zk_pitch[0]> 2 * PI)//量测值出现异常处理
	{
		zk_pitch[0] = zk_pitch[0] - 2 * PI;
	}
	if (zk_pitch[0]<-2 * PI)//量测值出现异常处理
	{
		zk_pitch[0] = zk_pitch[0] + 2 * PI;
	}
	if (fabs(zk_pitch[0])> PI / 4)//量测值出现异常处理
	{
		zk_pitch[0] = 0;
	}

	setzk(&kft2, zk_pitch);
	setRk(&kft2, noise_pitch, 1);
	setHk(&kft2, 1, 0);
	MUpdate(&kft2, noise_pitch, zk_pitch);
	atp->pitch -= kft2.xk[0][0];
	atp->gyobias[1] += kft2.xk[1][0];
#endif
	// printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",kft2.xk[0][0], kft2.xk[1][0],atp->gyobias[1],pitch_acc*R2D,atp->pitch*R2D,iatd->gyo3[1],iatd->acc3[0],iatd->acc3[1],iatd->acc3[2],iatd->imutime,atp->integ_pitch*R2D);
	outfile(2, "%f, %f, %f,%f,%f,%f,%f\n",
		kft2.xk[0][0], kft2.xk[1][0], atp->gyobias[1] * R2D, pitch_acc*R2D, normacc, zk_pitch[0] * R2D, atp->integ_pitch*R2D);
	kft2.xk[0][0] = 0;
	kft2.xk[1][0] = 0;
	NUM_acc2++;
	return 1;
}



int process_AttTrack(struct ATProcessSingleAngle* atp, struct Config* cfg,double gyo, double IMU_time, double ExterAngle)
{
	static struct  kalmanfilter kft2;
	double roll_acc = 0;
	double pitch_acc = 0;
	if (!atp->bkfinit)
	{
		/*********水平角度跟踪滤波器初始化********/
		//kalmanfilterinit(kft2, 2, 1);
		InitialMatrix_xk(&kft2);
		InitialMatrix_Pxk(&kft2);
		InitialMatrix_Phi(&kft2);
		InitialMatrix_Qk(&kft2);
		InitialMatrix_Hk(&kft2);
		InitialMatrix_Rk(&kft2);
		InitialMatrix_zk(&kft2);
		double dxk0[2] = { cfg->angle_err_init*D2R,cfg->bias_err_init*D2R };
		setxk(&kft2, dxk0);
		setPxk(&kft2, dxk0);
		double noise_imuy[2] = { cfg->gnoisex*D2R,cfg->gstabilityx*D2R / 3600.0 };//0.025,8.0(GAsensor ------scc2230)
		setQk(&kft2, noise_imuy);
		atp->bkfinit = 1;
	}
#if 1	

	atp->pitch += IMU_time*(gyo - atp->gyobias[0]);//陀螺积分求俯仰角
	setPhi(&kft2, IMU_time);
	TUpdate(&kft2, IMU_time);
	double noise_pitch[1] = { 0 };
	double normacc = 0;
	if (cfg->angle_flag == 1)
	{
		/************加速度计开窗平滑，滤除高频信号***********/
		if (cfg->ekf_strategy == 1) //加速度平滑滤波，量测噪声取定值
		{
			noise_pitch[0] = cfg->gnoisestd*D2R;
		}
		/************加速度计均值***********/
		if (cfg->ekf_strategy == 2)//加速度取均值，量测噪声动静态区分
		{
			if (cfg->acc_norm < 0.02)  //10mg
			{
				atp->num_accnorm++;
			}
			else
			{
				atp->num_accnorm = 0;
			}
			if (atp->num_accnorm > 100)  //准静态超过1秒
			{
				noise_pitch[0] = cfg->gnoisestd*D2R;
			}
			else
			{
				noise_pitch[0] = (cfg->gnoisestd + (normacc - 0.02) * 10)*D2R;
			}
			if (cfg->acc_norm  > 0.1)
			{
				printf("干扰加速度大！\n");
				return 1;
			}
		}
	}
	if (cfg->angle_flag == 2)
	{
		noise_pitch[0] = cfg->gnssyawvar*D2R;
	}
	


	double zk_pitch[1] = { 0 };
	if (atp->pitch> PI)//量测值出现异常处理
	{
		atp->pitch = atp->pitch - 2 * PI;
	}
	if (atp->pitch<-PI)//量测值出现异常处理
	{
		atp->pitch = atp->pitch + 2 * PI;
	}
	zk_pitch[0] = atp->pitch - ExterAngle;
	if (zk_pitch[0]> 2 * PI)//量测值出现异常处理
	{
		zk_pitch[0] = zk_pitch[0] - 2 * PI;
	}
	if (zk_pitch[0]<-2 * PI)//量测值出现异常处理
	{
		zk_pitch[0] = zk_pitch[0] + 2 * PI;
	}
	if (fabs(zk_pitch[0])> PI / 4)//量测值出现异常处理
	{
		zk_pitch[0] = 0;
	}

	setzk(&kft2, zk_pitch);
	setRk(&kft2, noise_pitch, 1);
	setHk(&kft2, 1, 0);
	MUpdate(&kft2, noise_pitch, zk_pitch);
	atp->pitch -= kft2.xk[0][0];
	atp->gyobias[1] += kft2.xk[1][0];
#endif
	// printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",kft2.xk[0][0], kft2.xk[1][0],atp->gyobias[1],pitch_acc*R2D,atp->pitch*R2D,iatd->gyo3[1],iatd->acc3[0],iatd->acc3[1],iatd->acc3[2],iatd->imutime,atp->integ_pitch*R2D);
	outfile(2, "%f, %f, %f,%f,%f,%f,%f\n",
		kft2.xk[0][0], kft2.xk[1][0], atp->gyobias[1] * R2D, pitch_acc*R2D, normacc, zk_pitch[0] * R2D, atp->integ_pitch*R2D);
	kft2.xk[0][0] = 0;
	kft2.xk[1][0] = 0;
	return 1;
}
