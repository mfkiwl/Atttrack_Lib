#pragma once
#include "ComFunc.h"
#include "DataStruct.h"
//#include "config.h"
//#include <vector>
//using namespace std;
//˫λ�÷��궨��װ�������궨�Ӽ���ƫ��
//��װ�����ò�Ҫ����30deg
#define Winlen 6000
struct Cal_Install_Error
{
	//vector<double> ax1, ay1, az1, ax2, ay2, az2;  //��̬��������
	//double ax1[Winlen] ;
	//double ay1[Winlen];
	//double az1[Winlen];
	//double ax2[Winlen];
	//double ay2[Winlen];
	//double az2[Winlen];
	double max1, may1, maz1, max2, may2, maz2;
	double sax1, say1, saz1, sax2, say2, saz2;
	double sumheading_A2B, sumheading_B2A;
	int winlen;  
	int winlen_gnss;
	double installangle[2]; //ˮƽ��װ����
	//option=1��A-B����2��B-A����3���������
	//int calheadingerr_m(AttTrackData* atdata, int option);//��̬����Ǳ궨����
	//option=1��λ��1�ɼ����ݣ�2��λ��2�ɼ����ݣ�3���������
	//int calheadingerr_s(AttTrackData* atdata, int option);//��̬����Ǳ궨����
};
//Cal_Install_Error(int len = 6000); //100Hz 1min
						   // option =1��2��3 1--��һλ�ò��� 2--�ڶ�λ�ò��� 3--���㰲װ����
int Install_Angle_Process(struct  AngleTrackData iatd, struct Cal_Install_Error install_err);
int calinstallerr_init(struct Cal_Install_Error* Cal_err);
int calinstallerr(struct AngleTrackData* atdata,struct Cal_Install_Error* Cal_err, int option);
void Comp_InstallErr_Acc(double acc[3], double installroll, double installpitch);
double GetAveStd(double acc_cal[], int acc_num,int opt);