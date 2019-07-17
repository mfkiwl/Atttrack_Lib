
#pragma once
//#include "ComFunc.h"
#include "AttTrack_lib.h"
#include "config.h"
//���嵼��ϵΪNED������ϵΪFRD
//����Ƕȸ���
//class AngleTrackData
//{
//public:
//	double imutime;            //ʱ�������
//	double gyo3[3];            //XYZ���ݽ��ٶȣ�deg/s��ǰ����
//	double acc3[3];            //XYZ�Ӽ��߼��ٶȣ�m/s2,ǰ����
//public:
//	void Init();
//	void Rest();
//	AngleTrackData& operator=(const AngleTrackData& atdata);
//};
struct AngleTrackData
{
	double imutime;            //ʱ�������
	double gyo3[3];            //XYZ���ݽ��ٶȣ�deg/s��ǰ����
	double acc3[3];            //XYZ�Ӽ��߼��ٶȣ�m/s2,ǰ����
};
void AngleTrackData_Init(struct AngleTrackData* atd);
void AngleTrackData_Rest(struct AngleTrackData* atd);

double dataFilter(double acc_win[], int count);
double getstd(double acc_win[], int count);
void decode_gasensor(IMUdata_t* atd, struct AngleTrackData* atdata);
void acc_bias(AttTrackCfg_t* cfg, IMUdata_t* atd, double accbias[]);