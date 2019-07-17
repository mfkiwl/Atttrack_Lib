#include "DataStruct.h"
#include <math.h>

void decode_gasensor(IMUdata_t* atd, struct AngleTrackData* atdata)
{
	atdata->imutime = atd->imutimetarget;
	atdata->gyo3[0] = atd->gyox;
	atdata->gyo3[1] = atd->gyoy;
	atdata->gyo3[2] = atd->gyoz;
	atdata->acc3[0] = atd->accx;
	atdata->acc3[1] = atd->accy;
	atdata->acc3[2] = atd->accz;
}

void acc_bias(AttTrackCfg_t* cfg, IMUdata_t* atd, double accbias[])
{
	accbias[0] = atd->accx - cfg->abiasx;
	accbias[1] = atd->accy - cfg->abiasy;
	accbias[2] = atd->accz - cfg->abiasz;
}

void AngleTrackData_Rest(struct AngleTrackData* atd)
{
	atd->imutime = 0;
	for (int i = 0; i < 3; i++)
	{
		atd->gyo3[i] = 0;
		atd->acc3[i] = 0;
	}
}
void AngleTrackData_Init(struct AngleTrackData* atd)
{
	atd->imutime = 0;
	for (int i = 0; i < 3; i++)
	{
		atd->gyo3[i] = 0;
		atd->acc3[i] = 0;
	}
}

//AngleTrackData& AngleTrackData::operator=(const AngleTrackData& atdata)
//{
//	imutime = atdata.imutime;
//	for (int i = 0; i < 3; i++)
//	{
//		gyo3[i] = atdata.gyo3[i];
//		acc3[i] = atdata.acc3[i];
//	}
//	return *this;
//}

double dataFilter(double acc_win[], int count)
{
	 double MAX=0, MIN=0;
	 double sum = 0;
	 double sum1 = 0;
	MAX = acc_win[0];
	MIN = acc_win[0];
	sum = acc_win[0];
	sum1 = acc_win[0];
	for (int i = 1;i<count; i++)
	{
		if (acc_win[i] > MAX)
		{
			MAX = acc_win[i];
		}
		if (acc_win[i] < MIN)
		{
			MIN = acc_win[i];
		}
		sum += acc_win[i];
		sum1 += acc_win[i];
	}
	double equal_data = (sum - MAX - MIN) / (count-2 );
	sum1 = sum1 / count;
	//printf("data:%f,%f,%f,%f,%f,MAX=%f,MIN=%f,%f,%f\n",acc_win[0], acc_win[1], acc_win[2], acc_win[3], acc_win[4],MAX, MIN,equal_data,sum1);
	return equal_data;
}

double getstd(double acc_win[], int count)
{
	static double mean_data = 0;
	int num = count;
	static double std_data = 0;
	for (int i = 0; i < num ; i++)
	{
		mean_data += acc_win[i];
	}
	mean_data= mean_data / (num);//计算均值
	for (int i = 0; i < num; i++)
	{
		std_data += (acc_win[i] - mean_data)*(acc_win[i] - mean_data);
	}
		std_data = sqrt(std_data / (num));//计算std
		return std_data;
}