#pragma once
#include "AttTrack_lib.h"
#include <math.h>
#define PI 3.1415926
static double gnss_heading_last = 0;
int Car_State_Judge(GNSSdata_t* gnss,double gnss_att[3]);//dϵ���ٶ��жϳ���ǰ�������������߼���ά��̬��
int Car_State_Judge1(GNSSdata_t* gnss);//˫���ߺ����жϵ���
int Car_State_Judge2(GNSSdata_t* gnss);//�����ߺ����жϵ���
void Cn2d(double gnss_att[3],double gnss_vel[3],double body_vel[3]);
