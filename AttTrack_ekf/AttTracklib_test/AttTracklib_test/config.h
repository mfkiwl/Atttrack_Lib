#pragma once
#include <string.h>
#include "parson.h"
#include "AttTrack_lib.h"
struct AttTrackCfg
{
	double gbiasx;//陀螺仪静态零偏
	double gbiasy;
	double gbiasz;
	double abiasx;//加速度计静态零偏
	double abiasy;
	double abiasz;
	double gstdxthr;
	double gstdythr;
	double gstdzthr;
	double astdxthr;
	double astdythr;
	double astdzthr;
	double gnoisex;//角速度噪声
	double gnoisey;
	double gnoisez;
	double anoisex;//加速度噪声
	double anoisey;
	double anoisez;
	double gstabilityx;//陀螺仪零偏稳定性
	double gstabilityy;
	double gstabilityz;
	double astabilityx;//加速度计零偏稳定性
	double astabilityy;
	double astabilityz;
	int gshaftx;
	int gshafty;
	int gshaftz;
	int ashaftx;
	int ashafty;
	int ashaftz;
	int ekf_strategy;//ekf量测噪声策略 1.加速度平滑滤波   2.动静态区分
	double gnssyawstd_thr;
	double gnssyawvar;
	double install_roll;//横滚角安装误差
	double install_pitch;//俯仰角安装误差
	double install_heading;//航向角安装误差
};
typedef struct  AttTrackCfg AttTrackCfg_t;

int init_config();
void save_config();
void creat_config();

//
int config_set_gyo_staticbiasx(double num);
double config_get_gyo_staticbiasx();
int config_set_gyo_staticbiasy(double num);
double config_get_gyo_staticbiasy();
int config_set_gyo_staticbiasz(double num);
double config_get_gyo_staticbiasz();
//�Ӽ���ƫ
int config_set_acc_staticbiasx(double num);
double config_get_acc_staticbiasx();
int config_set_acc_staticbiasy(double num);
double config_get_acc_staticbiasy();
int config_set_acc_staticbiasz(double num);
double config_get_acc_staticbiasz();
//���ݾ�̬�ж�std��ֵ
int config_set_gyo_stdx_thr(double num);
double config_get_gyo_stdx_thr();
int config_set_gyo_stdy_thr(double num);
double config_get_gyo_stdy_thr();
int config_set_gyo_stdz_thr(double num);
double config_get_gyo_stdz_thr();
//�Ӽƾ�̬�ж�std��ֵ
int config_set_acc_stdx_thr(double num);
double config_get_acc_stdx_thr();
int config_set_acc_stdy_thr(double num);
double config_get_acc_stdy_thr();
int config_set_acc_stdz_thr(double num);
double config_get_acc_stdz_thr();
//���ݽ��ٶ���������
int config_set_gyo_noisex(double num);
double config_get_gyo_noisex();
int config_set_gyo_noisey(double num);
double config_get_gyo_noisey();
int config_set_gyo_noisez(double num);
double config_get_gyo_noisez();
//�Ӽ����ٶ���������
int config_set_acc_noisex(double num);
double config_get_acc_noisex();
int config_set_acc_noisey(double num);
double config_get_acc_noisey();
int config_set_acc_noisez(double num);
double config_get_acc_noisez();
// ������ƫ�ȶ���
int config_set_gyo_stabilityx(double num);
double config_get_gyo_stabilityx();
int config_set_gyo_stabilityy(double num);
double config_get_gyo_stabilityy();
int config_set_gyo_stabilityz(double num);
double config_get_gyo_stabilityz();
//�Ӽ���ƫ�ȶ���
int config_set_acc_stabilityx(double num);
double config_get_acc_stabilityx();
int config_set_acc_stabilityy(double num);
double config_get_acc_stabilityy();
int config_set_acc_stabilityz(double num);
double config_get_acc_stabilityz();

//������ϵת��
int config_set_gyo_shaftx(int num);
int config_get_gyo_shaftx();
int config_set_gyo_shafty(int num);
int config_get_gyo_shafty();
int config_set_gyo_shaftz(int num);
int config_get_gyo_shaftz();
int config_set_ekf_strategy(int num);
int config_get_ekf_strategy();
//�Ӽ���ϵת��
int config_set_acc_shaftx(int num);
int config_get_acc_shaftx();
int config_set_acc_shafty(int num);
int config_get_acc_shafty();
int config_set_acc_shaftz(int num);
int config_get_acc_shaftz();
//˫���ߺ���std�ж���ֵ�͹۲�����
int config_set_gnssyawstd_thr(double num);
double config_get_gnssyawstd_thr();
int config_set_gnssyaw_var(double num);
double config_get_gnssyaw_var();

//��װ���
int config_set_install_roll(double num);
double config_get_install_roll();
int config_set_install_pitch(double num);
double config_get_install_pitch();
int config_set_install_heading(double num);
double config_get_install_heading();


void generate_default_config_Angle();
void get_config(AttTrackCfg_t* cfg);


