#include "GnssHeading.h"
int  GnssHeading_Process(GNSSdata_t* gnss, double *result)
{
	double gnss_vel[3] = { 0 };
	double gnss_heading = 0;
	gnss_vel[0] = gnss->gnss_v[0];//enu下速度  m/s
	gnss_vel[1] = gnss->gnss_v[1];
	gnss_vel[2] = gnss->gnss_v[2];
	if (gnss->state_pos)//gnss数据有更新
	{
	    gnss_heading = atan2(gnss_vel[1] , gnss_vel[0]);//正负180度
		gnss_heading_last = gnss_heading;
	}
	else //gnss数据无更新时，航向保持不变
	{
		gnss_heading = gnss_heading_last;
	}
	int car_flag=Car_State_Judge1(gnss);
	if (car_flag == 0)//停车航向处理
	{
		gnss_heading = gnss_heading_last;
	}
	if (car_flag == -1)//倒车时航向处理
	{
		if ((gnss_heading >= 0) && (gnss_heading <= PI / 2))//第一象限
		{
			gnss_heading = gnss_heading - PI;
		}
		if ((gnss_heading <= 0) && (gnss_heading >= -PI / 2))//第二象限
		{
			gnss_heading = (gnss_heading + PI);
		}
		if ((gnss_heading >= PI/2) && (gnss_heading <= PI))//第三象限
		{
			gnss_heading = -(gnss_heading - PI);
		}
		if ((gnss_heading >= -PI) && (gnss_heading <= -PI/2))//第四象限
		{
			gnss_heading = (gnss_heading + PI);
		}
	}
	return 1;
}


int Car_State_Judge(GNSSdata_t* gnss,double gnss_att[3])
{
	double gnss_vel[3] = { 0 };
	gnss_vel[0] = gnss->gnss_v[0];//enu下速度  m/s
	gnss_vel[1] = gnss->gnss_v[1];
	gnss_vel[2] = gnss->gnss_v[2];
	double body_vel[3] = { 0 };
	Cn2d(gnss_att,gnss_vel, body_vel);//姿态转换，n系(enu)到d系 d系（右前上）
	if (fabs(body_vel[1]) < 0.1)
	{
		return 0;//静止
	}
	if (body_vel[1] > 0.1)
	{
		return 1;//前进
	}
	if (body_vel[1] <- 0.1)
	{
		return -1;//倒车
	}
	return 2;
}
int Car_State_Judge1(GNSSdata_t* gnss)
{
	double gnss_vel[3] = { 0 };
	gnss_vel[0] = gnss->gnss_v[0];//enu下速度  m/s
	gnss_vel[1] = gnss->gnss_v[1];
	gnss_vel[2] = gnss->gnss_v[2];
	double gnss_two_heading = gnss->gnssyaw;//gnss双天线航向 
	double gnss_heading[2] = { 0 };
	gnss_heading[0] = sin(gnss_two_heading);
	gnss_heading[1] = cos(gnss_two_heading);
	double temp = gnss_heading[0] * gnss_vel[0] + gnss_heading[1] * gnss_vel[1];
	if (temp < 0)
	{
		return -1;
	}
	else
	{
		return 1;
	}
	return 2;
}
void Cn2d(double gnss_att[3], double gnss_vel[3],double body_vel[3])
{
	double C[9] = { 0 };
	double cr = cos(gnss_att[1]);
	double sr = sin(gnss_att[1]);
	double cp = cos(gnss_att[0]);
	double sp = sin(gnss_att[0]);
	double ch = cos(gnss_att[2]);
	double sh = sin(gnss_att[2]);
	C[0] = cr*ch - sr*sp*sh;
	C[1] = cr*sh + sr*ch*sp;
	C[2] = -sr*cp;
	C[3] = -sh*cp;
	C[4] = ch*cp;
	C[5] = sp;
	C[6] = sp*ch + cr*sh*sp;
	C[7] = sr*sh - cr*ch*sp;
	C[8] = cr*cp;
	for (int i = 0;i < 3;i++)
	{
		body_vel[i] = C[3*i] * gnss_vel[0] + C[3*i+1] * gnss_vel[1] + C[3*i+2] * gnss_vel[2];
	}
}