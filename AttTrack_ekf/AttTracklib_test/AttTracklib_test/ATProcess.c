#include "ATProcess.h"


 static double acc_x_win1[5] = { 0 };
 static double acc_y_win1[5] = { 0 };
 static double acc_z_win1[5] = { 0 };
 static double acc_x_win2[5] = { 0 };
 static double acc_y_win2[5] = { 0 };
 static double acc_z_win2[5] = { 0 };
 static double acc_x_win3[5] = { 0 };
 static double acc_y_win3[5] = { 0 };
 static double acc_z_win3[5] = { 0 };
 static double acc_x_win4[5] = { 0 };
 static double acc_y_win4[5] = { 0 };
 static double acc_z_win4[5] = { 0 };
int NUM_acc1 = 0;
int NUM_acc2 = 0;
int NUM_acc3 = 0;
int NUM_acc4 = 0;
int extern zero_num;
static double static_accpitch=0;

/****************����������Ƕȸ���********************/
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
int process_gasensor_D(struct AngleTrackData* iatd, struct AttTrackCfg* cfg, struct ATProcessSingleAngle* atp)
{
		double roll_acc = 0;
		double pitch_acc = 0;
		static struct  kalmanfilter kft1p;//�����ǽṹ��
		static struct  kalmanfilter kft1r;//����ǽṹ��
		if (!atp->bkfinit)
	{
		/*********ˮƽ�Ƕȸ����˲�����ʼ��********/
		//kalmanfilterinit(kft1r, 2,1);//����ǽṹ���ʼ
		InitialMatrix_xk(&kft1r);//��̬��ά�����ʼ��
		InitialMatrix_Pxk(&kft1r);
		InitialMatrix_Phi(&kft1r);
		InitialMatrix_Qk(&kft1r);
		InitialMatrix_Hk(&kft1r);
		InitialMatrix_Rk(&kft1r);
		InitialMatrix_zk(&kft1r);
		//kalmanfilterinit(kft1p, 2,1);//�����ǽṹ���ʼ
		InitialMatrix_xk(&kft1p);
		InitialMatrix_Pxk(&kft1p);
		InitialMatrix_Phi(&kft1p);
		InitialMatrix_Qk(&kft1p);
		InitialMatrix_Hk(&kft1p);
		InitialMatrix_Rk(&kft1p);
		InitialMatrix_zk(&kft1p);
		double dxk0[2] = { 0.1*D2R,0.1*D2R };
		setPxk(&kft1p, dxk0);
		setPxk(&kft1r, dxk0);
		double noise_imuy[2] = { cfg->gnoisex*D2R,cfg->gstabilityx*D2R / 3600.0 };//0.025,8.0(IS203-----SCC3300)
		setQk(&kft1r, noise_imuy);
		setQk(&kft1p, noise_imuy);
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
		Mminn(gyomean, atp->gyobias, gyo_bias, 3, 1); //��������ȡ��ֵ		
#if 1
		Mmul(gyo_bias, 3, 1, dtime);
		qupdt(atp->qua, gyo_bias);
		double atttemp[3] = { 0.0 };
		q2mat_ned(atp->qua, atp->Cb2n);
		m2att_ned(atp->Cb2n, atttemp);
		atp->roll = atttemp[0];
		atp->pitch = atttemp[1];
		atp->heading = atttemp[2];
	//atp->pitch += iatd->imutime*(iatd->gyo3[1]-atp->gyobias[1]);//���ݻ���������
	//atp->roll += iatd->imutime*(iatd->gyo3[0]-atp->gyobias[0]);//���ݻ���������
#endif
		setPhi(&kft1r, dtime);
		setPhi(&kft1p, dtime);
		TUpdate(&kft1r, dtime);
		TUpdate(&kft1p, dtime);
		
		double noise_pitch[1] = { 0 };
		double noise_roll[1] = { 0 };
		double normacc = 0;
	/************���ٶȼƿ���ƽ�����˳���Ƶ�ź�***********/

		if (cfg->ekf_strategy == 1)
		{
			double smooth_ax = 0, smooth_ay = 0, smooth_az = 0, smooth_gz = 0;
			if (NUM_acc1 < WinLen)
			{
				acc_x_win1[NUM_acc1] = iatd->acc3[0];
				acc_y_win1[NUM_acc1] = iatd->acc3[1];
				acc_z_win1[NUM_acc1] = iatd->acc3[2];
			}
			if (NUM_acc1 >= WinLen)
			{
				for (int i = 0;i < WinLen;i++)
				{
					acc_x_win1[i] = acc_x_win1[i + 1];
					acc_y_win1[i] = acc_y_win1[i + 1];
					acc_z_win1[i] = acc_z_win1[i + 1];
				}
				acc_x_win1[WinLen - 1] = iatd->acc3[0];
				acc_y_win1[WinLen - 1] = iatd->acc3[1];
				acc_z_win1[WinLen - 1] = iatd->acc3[2];
				smooth_ax = dataFilter(acc_x_win1, WinLen);
				smooth_ay = dataFilter(acc_y_win1, WinLen);
				smooth_az = dataFilter(acc_z_win1, WinLen);
				roll_acc = atan2(-smooth_ay, -smooth_az);
				pitch_acc = atan2(smooth_ax, sqrt(smooth_ay * smooth_ay + smooth_az * smooth_az));
			}
			//���ݹ�ʽ(����ż��ٶȴ�С���)
			/******************������Ϊ��ֵ*******************/
			noise_roll[0] = 1.5*D2R;
			noise_pitch[0] = 1.5*D2R;
			normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
		}
		
	/************���ٶȼƾ�ֵ***********/
		if (cfg->ekf_strategy == 2)
		{
			pitch_acc = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));//���ٶȼ��㸩����
			roll_acc = atan2(-accmean[1], -accmean[2]);//���ٶȼƼ�������
		/*******************��������̬����*************/
			normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
			double normacc1 = fabs((pitch_acc - static_accpitch)*R2D);
			if (normacc < 0.05)  //10mg
			{
				atp->num_accnorm++;
			}
			else
			{
				atp->num_accnorm = 0;
			}
			if (atp->num_accnorm >100)  //׼��̬����1��
			{
				noise_pitch[0] = 0.5*D2R;//0.5
				noise_roll[0] = 0.5*D2R;
				/********���Ӿ�̬�Ĵ����Ӽƿ���ƽ��*********/
			}
			else
			{
				noise_pitch[0] = (0.5 + (normacc - 0.05) * 50)*D2R;
				noise_roll[0] = (0.5 + (normacc - 0.05) * 50)*D2R;
				//noise_pitch[0] = (0.5 + normacc1* 1.5)*D2R;
			}
			if (normacc > 0.1)
			{
				//printf("���ż��ٶȴ�\n");
		//		return 1;
			}
		}
		double zk_pitch[1] = { 0 };
		double zk_roll[1]={0};
		zk_pitch[0] = atp->pitch - pitch_acc;//����������ֵ
		zk_roll[0]=atp->roll-roll_acc;//���������ֵ
		if((zk_pitch[0])>2*PI)//����ֵ�����쳣����
	{
		zk_pitch[0]=zk_pitch[0]-2*PI;
	}
			if((zk_pitch[0])<-2*PI)//����ֵ�����쳣����
	{
		zk_pitch[0]=zk_pitch[0]+2*PI;
	}
	if((zk_roll[0])>2*PI)//����ֵ�����쳣����
	{
		zk_roll[0]=zk_roll[0]-2*PI;
	}
			if((zk_roll[0])<-2*PI)//����ֵ�����쳣����
	{
		zk_roll[0]=zk_roll[0]+2*PI;
	}
	
	
		setzk(&kft1r, zk_roll);
		setzk(&kft1p, zk_pitch);
		setRk(&kft1r, noise_roll, 1);
		setRk(&kft1p, noise_pitch, 1);
		setHk(&kft1r, 1, 0);
		setHk(&kft1p, 1, 0);
		MUpdate(&kft1r, noise_roll, zk_roll);
		MUpdate(&kft1p, noise_pitch, zk_pitch);
		atp->roll -= kft1r.xk[0][0];//����ǽǶȲ���
		atp->gyobias[0] += kft1r.xk[1][0];//x����ƫ����
		atp->pitch -= kft1p.xk[0][0];//�����Ƿ���
		atp->gyobias[1] += kft1p.xk[1][0];//y����ƫ��������
		outfile(2, "%f, %f, %f,%f,%f,%f,%f\n",
			kft1p.xk[0], kft1p.xk[1], atp->gyobias[1] * R2D, pitch_acc*R2D, normacc, noise_pitch[0] * R2D, atp->integ_pitch*R2D);
	//���ݴ�ӡ������
	//printf("acc_angle:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
	//atp->roll*R2D,roll_acc*R2D,atp->pitch*R2D,pitch_acc*R2D,iatd->gyo3[0],iatd->gyo3[1],iatd->gyo3[2],iatd->acc3[0],iatd->acc3[1],iatd->acc3[2],iatd->imutime,
	//zk_roll[0]*R2D,zk_pitch[0]*R2D,kft1r.xk[0],kft1r.xk[1]);
		kft1r.xk[0][0] = 0;//״̬����
		kft1r.xk[1][0] = 0;
		kft1p.xk[0][0] = 0;//״̬����
		kft1p.xk[1][0] = 0;
		NUM_acc1++;
		double atti[3] = { atp->roll,atp->pitch,atp->heading }; 
		a2mat_ned(atti, atp->Cb2n);
		m2qua_ned(atp->Cb2n, atp->qua);

#endif
		//�����ǵ�ƽ
		//atp->roll = atan2(sin(atp->roll)*cos(atp->pitch), sqrt(sin(atp->roll)*sin(atp->roll)*sin(atp->pitch)*sin(atp->pitch) + cos(atp->roll)*cos(atp->roll)));
		return 1;
}

/**********************************************************/
int process_gasensor_U(struct AngleTrackData* iatd, struct AttTrackCfg* cfg, struct ATProcessSingleAngle* atp)
{
	 static struct  kalmanfilter kft2;
		double roll_acc = 0;
		double pitch_acc = 0;
		if (!atp->bkfinit)
	{
		/*********ˮƽ�Ƕȸ����˲�����ʼ��********/
		//kalmanfilterinit(kft2, 2, 1);
		InitialMatrix_xk(&kft2);
		InitialMatrix_Pxk(&kft2);
		InitialMatrix_Phi(&kft2);
		InitialMatrix_Qk(&kft2);
		InitialMatrix_Hk(&kft2);
		InitialMatrix_Rk(&kft2);
		InitialMatrix_zk(&kft2);
		double dxk0[2] = {0.01*D2R,0.1*D2R };
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
		Mminn(gyomean, atp->gyobias, gyo_bias, 3, 1); //��������ȡ��ֵ										 
		atp->pitch += iatd->imutime*(iatd->gyo3[1] - atp->gyobias[1]);//���ݻ���������
		atp->integ_pitch+= iatd->imutime*(iatd->gyo3[1] - atp->gyobias[1]);//���������ǣ��޷���������
		setPhi(&kft2, dtime);
		TUpdate(&kft2, dtime);
		double noise_pitch[1] = { 0 };
		double normacc = 0;
	/************���ٶȼƿ���ƽ�����˳���Ƶ�ź�***********/
		if (cfg->ekf_strategy == 1) //���ٶ�ƽ���˲�����������ȡ��ֵ
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
				pitch_acc = atan2(smooth_ax, (-smooth_az));//˫����ٶȼ��㸩����
			}
			noise_pitch[0] = 0.5*D2R ;
			normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
			if (normacc > 0.1)//���ٶȳ��޴���
			{
				return 1;
			}
		}
	/************���ٶȼƾ�ֵ***********/
		if (cfg->ekf_strategy == 2)//���ٶ�ȡ��ֵ��������������̬����
		{
			//	pitch_acc = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));
			pitch_acc = atan2(accmean[0], (-accmean[2]));
			//���ݹ�ʽ(����ż��ٶȴ�С���)
		/*******************��������̬����*************/
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
			if (atp->num_accnorm > 100)  //׼��̬����1��
			{
				noise_pitch[0] = 0.3*D2R;
				//	static_accpitch = pitch_acc;
					/********���Ӿ�̬�Ĵ����Ӽƿ���ƽ��*********/
			}
			else
			{
				noise_pitch[0] = (0.5 + (normacc - 0.02) * 10)*D2R;
				//noise_pitch[0] = (0.5 + normacc1* 1.5)*D2R;
			}
			if (normacc > 0.1)
			{
				//printf("���ż��ٶȴ�\n");
				return 1;
			}
		}
		double zk_pitch[1] = { 0 };
		if (atp->pitch> PI)//����ֵ�����쳣����
		{
			atp->pitch = atp->pitch - 2 * PI;
		}
		if (atp->pitch<-PI)//����ֵ�����쳣����
		{
			atp->pitch = atp->pitch + 2 * PI;
		}
		zk_pitch[0] = atp->pitch - pitch_acc;
		if (zk_pitch[0]> 2*PI)//����ֵ�����쳣����
		{
			zk_pitch[0] = zk_pitch[0] - 2 * PI;
		}
		if (zk_pitch[0]<-2*PI)//����ֵ�����쳣����
		{
			zk_pitch[0] = zk_pitch[0] + 2 * PI;
		}
		if (fabs(zk_pitch[0])> PI/4)//����ֵ�����쳣����
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
		kft2.xk[1][0]= 0;
		NUM_acc2++;
		return 1;
}
/**********************************************************/
int process_gasensor_UE(struct AngleTrackData* iatd, struct AttTrackCfg* cfg, double ExterAngle,struct ATProcessSingleAngle* atp)
{
		double roll_acc = 0;
		double pitch_acc = 0;
		static struct  kalmanfilter kft3;
		if (!atp->bkfinit)
	{
		/*********ˮƽ�Ƕȸ����˲�����ʼ��********/
		//kalmanfilterinit(kft3, 2, 1);
		InitialMatrix_xk(&kft3);//��̬�����ʼ��
		InitialMatrix_Pxk(&kft3);
		InitialMatrix_Phi(&kft3);
		InitialMatrix_Qk(&kft3);
		InitialMatrix_Hk(&kft3);
		InitialMatrix_Rk(&kft3);
		InitialMatrix_zk(&kft3);
		double dxk0[2] = { 0.05*D2R,0.1*D2R };
		setPxk(&kft3, dxk0);
		double noise_imuy[2] = { cfg->gnoisex*D2R,cfg->gstabilityx*D2R / 3600.0 };
		setQk(&kft3, noise_imuy);
		atp->bkfinit = 1;
	}
		double dtime = iatd->imutime;
		double gyo_bias[3] = { 0 };
		double gyomean[3] = { 0 };
		Maddn(atp->gyopre, iatd->gyo3, gyomean, 3, 1);
		Mmul(gyomean, 3, 1, 0.5);
		Mequalm(iatd->gyo3, 3, 1, atp->gyopre);
		Mminn(gyomean, atp->gyobias, gyo_bias, 3, 1); //��������ȡ��ֵ										 
		atp->pitch += (iatd->imutime*(iatd->gyo3[1] - atp->gyobias[1]));//���ݻ���������
		atp->integ_pitch+= iatd->imutime*(iatd->gyo3[1] - atp->gyobias[1]);//���������ǣ��޷���������
		setPhi(&kft3, dtime);
		TUpdate(&kft3, dtime);
		pitch_acc = ExterAngle;//�ⲿ�Ƕȸ�ֵ
		double noise_pitch[1] = { 0 };
		double zk_pitch[1] = { 0 };
		zk_pitch[0] = atp->pitch - pitch_acc;
		if (zk_pitch[0] > PI)//����ֵ����ֵ����
	{
		zk_pitch[0] -= 2 * PI;
	}
	if (zk_pitch[0] <- PI)
	{
		zk_pitch[0] += 2 * PI;
	}
	if (fabs(zk_pitch[0]) >PI/4)
	{
		zk_pitch[0] =0;
	}
		setzk(&kft3, zk_pitch);
		setRk(&kft3, noise_pitch, 1);
		setHk(&kft3, 1, 0);
		MUpdate(&kft3, noise_pitch, zk_pitch);
		atp->pitch -= kft3.xk[0][0];
		atp->gyobias[1] += kft3.xk[1][0];
		outfile(2, "%f, %f, %f,%f,%f,%f,%f\n",
			kft3.xk[0], kft3.xk[1], atp->gyobias[1] * R2D, pitch_acc*R2D, 0, noise_pitch[0] * R2D, atp->integ_pitch*R2D);
		kft3.xk[0][0] = 0;
		kft3.xk[1][0] = 0;
		NUM_acc3++;
		return 1;
}
/**********************************************************/

int process_gasensor_3D(struct AngleTrackData* iatd, struct AttTrackCfg* cfg, double ExterHeading, struct ATProcessSingleAngle* atp)
{
	double roll_acc = 0;
	double pitch_acc = 0;
	static struct  kalmanfilter kft1p;//�����ǽṹ��
	static struct  kalmanfilter kft1r;//����ǽṹ��
	static struct  kalmanfilter kft1h;//����ǽṹ��
	if (!atp->bkfinit)
	{
		/*********ˮƽ�Ƕȸ����˲�����ʼ��********/
		//kalmanfilterinit(kft1r, 2, 1);//����ǽṹ���ʼ
		InitialMatrix_xk(&kft1r);
		InitialMatrix_Pxk(&kft1r);
		InitialMatrix_Phi(&kft1r);
		InitialMatrix_Qk(&kft1r);
		InitialMatrix_Hk(&kft1r);
		InitialMatrix_Rk(&kft1r);
		InitialMatrix_zk(&kft1r);
		//kalmanfilterinit(kft1p, 2, 1);//�����ǽṹ���ʼ
		InitialMatrix_xk(&kft1p);
		InitialMatrix_Pxk(&kft1p);
		InitialMatrix_Phi(&kft1p);
		InitialMatrix_Qk(&kft1p);
		InitialMatrix_Hk(&kft1p);
		InitialMatrix_Rk(&kft1p);
		InitialMatrix_zk(&kft1p);
		//kalmanfilterinit(kft1h, 2, 1);//�����ǽṹ���ʼ
		InitialMatrix_xk(&kft1h);
		InitialMatrix_Pxk(&kft1h);
		InitialMatrix_Phi(&kft1h);
		InitialMatrix_Qk(&kft1h);
		InitialMatrix_Hk(&kft1h);
		InitialMatrix_Rk(&kft1h);
		InitialMatrix_zk(&kft1h);
		double dxk0[2] = { 0.1*D2R,0.1*D2R };//�����������ʼ�Ƕ�
		double dxk1[2] = { 2*D2R,0.05*D2R };//�����ʼ�Ƕ�
		setPxk(&kft1p, dxk0);
		setPxk(&kft1r, dxk0);
		setPxk(&kft1h, dxk1);
		double noise_imuy[2] = { cfg->gnoisex*D2R,cfg->gstabilityx*D2R / 3600.0 };//0.025,8.0(IS203-----SCC3300)
		double noise_heading[2] = { cfg->gnoisex*D2R,cfg->gstabilityx*D2R / 3600.0 };//0.025,8.0(IS203-----SCC3300)
		setQk(&kft1r, noise_imuy);
		setQk(&kft1p, noise_imuy);
		setQk(&kft1h, noise_heading);
		atp->bkfinit = 1;
	}
	double dtime = iatd->imutime;
	double gyo_bias[3] = { 0 };
	double accmean[3] = { 0 }, gyomean[3] = { 0 };
	Maddn(atp->accpre, iatd->acc3, accmean, 3, 1);
	Maddn(atp->gyopre, iatd->gyo3, gyomean, 3, 1);
	Mmul(accmean, 3, 1, 0.5);
	Mmul(gyomean, 3, 1, 0.5);
	Mequalm(iatd->acc3, 3, 1, atp->accpre);
	Mequalm(iatd->gyo3, 3, 1, atp->gyopre);
	Mminn(gyomean, atp->gyobias, gyo_bias, 3, 1); //��������ȡ��ֵ		
#if 1
	Mmul(gyo_bias, 3, 1, dtime);
	qupdt(atp->qua, gyo_bias);
	double atttemp[3] = { 0.0 };
	q2mat_ned(atp->qua, atp->Cb2n);
	m2att_ned(atp->Cb2n, atttemp);
	atp->roll = atttemp[0];
	atp->pitch = atttemp[1];
	atp->heading = atttemp[2];
	//atp->pitch += iatd->imutime*(iatd->gyo3[1]-atp->gyobias[1]);//���ݻ���������
	//atp->roll += iatd->imutime*(iatd->gyo3[0]-atp->gyobias[0]);//���ݻ���������
#endif
	setPhi(&kft1r, dtime);
	setPhi(&kft1p, dtime);
	TUpdate(&kft1r, dtime);
	TUpdate(&kft1p, dtime);

	double noise_pitch[1] = { 0 };
	double noise_roll[1] = { 0 };
	double noise_heading[1] = { 0 };
	double normacc = 0;
	/************���ٶȼƿ���ƽ�����˳���Ƶ�ź�***********/
	if (cfg->ekf_strategy == 1)
	{
		double smooth_ax = 0, smooth_ay = 0, smooth_az = 0, smooth_gz = 0;
		if (NUM_acc1 < WinLen)
		{
			acc_x_win1[NUM_acc1] = iatd->acc3[0];
			acc_y_win1[NUM_acc1] = iatd->acc3[1];
			acc_z_win1[NUM_acc1] = iatd->acc3[2];
		}
		if (NUM_acc1 >= WinLen)
		{
			for (int i = 0;i < WinLen;i++)
			{
				acc_x_win1[i] = acc_x_win1[i + 1];
				acc_y_win1[i] = acc_y_win1[i + 1];
				acc_z_win1[i] = acc_z_win1[i + 1];
			}
			acc_x_win1[WinLen - 1] = iatd->acc3[0];
			acc_y_win1[WinLen - 1] = iatd->acc3[1];
			acc_z_win1[WinLen - 1] = iatd->acc3[2];
			smooth_ax = dataFilter(acc_x_win1, WinLen);
			smooth_ay = dataFilter(acc_y_win1, WinLen);
			smooth_az = dataFilter(acc_z_win1, WinLen);
			roll_acc = atan2(-smooth_ay, -smooth_az);
			pitch_acc = atan2(smooth_ax, sqrt(smooth_ay * smooth_ay + smooth_az * smooth_az));
		}
		//���ݹ�ʽ(����ż��ٶȴ�С���)
		/******************������Ϊ��ֵ*******************/
		noise_roll[0] = 1.5*D2R ;
		noise_pitch[0] =  1.5*D2R ;
		noise_heading[0] =  cfg->gnssyawvar*D2R ;
		normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
	}

	/************���ٶȼƾ�ֵ***********/
	if (cfg->ekf_strategy == 2)
	{
		pitch_acc = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));//���ٶȼ��㸩����
		roll_acc = atan2(-accmean[1], -accmean[2]);//���ٶȼƼ�������
												   /*******************��������̬����*************/
		normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
		double normacc1 = fabs((pitch_acc - static_accpitch)*R2D);
		if (normacc < 0.05)  //10mg
		{
			atp->num_accnorm++;
		}
		else
		{
			atp->num_accnorm = 0;
		}
		if (atp->num_accnorm >100)  //׼��̬����1��
		{
			noise_pitch[0] = 0.5*D2R;//0.5
			noise_roll[0] = 0.5*D2R;
			/********���Ӿ�̬�Ĵ����Ӽƿ���ƽ��*********/
		}
		else
		{
			noise_pitch[0] = (0.5 + (normacc - 0.05) * 50)*D2R;
			noise_roll[0] = (0.5 + (normacc - 0.05) * 50)*D2R;
			//noise_pitch[0] = (0.5 + normacc1* 1.5)*D2R;
		}
		if (normacc > 0.1)
		{
			//printf("���ż��ٶȴ�\n");
			//		return 1;
		}
	}
	double zk_pitch[1] = { 0 };
	double zk_roll[1] = { 0 };
	double zk_heading[1] = { 0 };
	zk_pitch[0] = atp->pitch - pitch_acc;//����������ֵ
	zk_roll[0] = atp->roll - roll_acc;//���������ֵ
	zk_heading[0] = atp->heading - ExterHeading;//���������ֵ
	if ((zk_pitch[0])>2 * PI)//����ֵ�����쳣����
	{
		zk_pitch[0] = zk_pitch[0] - 2 * PI;
	}
	if ((zk_pitch[0])<-2 * PI)//����ֵ�����쳣����
	{
		zk_pitch[0] = zk_pitch[0] + 2 * PI;
	}
	if ((zk_roll[0])>2 * PI)//����ֵ�����쳣����
	{
		zk_roll[0] = zk_roll[0] - 2 * PI;
	}
	if ((zk_roll[0])<-2 * PI)//����ֵ�����쳣����
	{
		zk_roll[0] = zk_roll[0] + 2 * PI;
	}


	setzk(&kft1r, zk_roll);
	setzk(&kft1p, zk_pitch);
	setzk(&kft1h, zk_heading);
	setRk(&kft1r, noise_roll, 1);
	setRk(&kft1p, noise_pitch, 1);
	setRk(&kft1h, noise_heading, 1);
	setHk(&kft1r, 1, 0);
	setHk(&kft1p, 1, 0);
	setHk(&kft1h, 1, 0);
	MUpdate(&kft1r, noise_roll, zk_roll);
	MUpdate(&kft1p, noise_pitch, zk_pitch);
	MUpdate(&kft1h, noise_heading, zk_heading);
	atp->roll -= kft1r.xk[0][0];//����ǽǶȲ���
	atp->gyobias[0] += kft1r.xk[1][0];//x����ƫ����
	atp->pitch -= kft1p.xk[0][0];//�����Ƿ���
	atp->gyobias[1] += kft1p.xk[1][0];//y����ƫ��������
	atp->heading -= kft1h.xk[0][0];//����Ƿ���
	atp->gyobias[2] += kft1h.xk[1][0];//z����ƫ��������
	outfile(2, "%f, %f, %f,%f,%f,%f,%f\n",
		kft1p.xk[0], kft1p.xk[1], atp->gyobias[1] * R2D, pitch_acc*R2D, normacc, noise_pitch[0] * R2D, atp->integ_pitch*R2D);
	//���ݴ�ӡ������
	//	printf("acc_angle:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
	//	atp->roll*R2D,roll_acc*R2D,atp->pitch*R2D,pitch_acc*R2D,iatd->gyo3[0],iatd->gyo3[1],iatd->gyo3[2],iatd->acc3[0],iatd->acc3[1],iatd->acc3[2],iatd->imutime,
	//	zk_roll[0]*R2D,zk_pitch[0]*R2D,kft1r.xk[0],kft1r.xk[1]);
	kft1r.xk[0][0] = 0;//״̬����
	kft1r.xk[1][0] = 0;
	kft1p.xk[0][0] = 0;//״̬����
	kft1p.xk[1][0] = 0;
	kft1h.xk[0][0] = 0;//״̬����
	kft1h.xk[1][0] = 0;
	NUM_acc1++;
	double atti[3] = { atp->roll,atp->pitch,atp->heading };
	a2mat_ned(atti, atp->Cb2n);
	m2qua_ned(atp->Cb2n, atp->qua);
	//�����ǵ�ƽ
	//atp->roll = atan2(sin(atp->roll)*cos(atp->pitch), sqrt(sin(atp->roll)*sin(atp->roll)*sin(atp->pitch)*sin(atp->pitch) + cos(atp->roll)*cos(atp->roll)));
	return 1;
}
/**********************************************************/


