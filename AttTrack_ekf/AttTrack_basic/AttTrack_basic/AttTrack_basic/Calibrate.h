#pragma once
#include "ComFunc.h"
#include "DataStruct.h"
//#include "config.h"
//#include <vector>
//using namespace std;
//双位置法标定安装误差（出厂标定加计零偏误差）
//安装误差最好不要超过30deg
#define Winlen 6000
struct Cal_Install_Error
{
	//vector<double> ax1, ay1, az1, ax2, ay2, az2;  //静态数据向量
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
	double installangle[2]; //水平安装误差角
	//option=1：A-B方向；2：B-A方向；3：计算完成
	//int calheadingerr_m(AttTrackData* atdata, int option);//动态航向角标定方案
	//option=1：位置1采集数据；2：位置2采集数据；3：计算完成
	//int calheadingerr_s(AttTrackData* atdata, int option);//静态航向角标定方案
};
//Cal_Install_Error(int len = 6000); //100Hz 1min
						   // option =1，2，3 1--第一位置采样 2--第二位置采样 3--计算安装误差角
int Install_Angle_Process(struct  AngleTrackData iatd, struct Cal_Install_Error install_err);
int calinstallerr_init(struct Cal_Install_Error* Cal_err);
int calinstallerr(struct AngleTrackData* atdata,struct Cal_Install_Error* Cal_err, int option);
void Comp_InstallErr_Acc(double acc[3], double installroll, double installpitch);
double GetAveStd(double acc_cal[], int acc_num,int opt);