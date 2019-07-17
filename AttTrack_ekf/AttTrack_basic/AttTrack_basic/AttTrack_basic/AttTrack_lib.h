#pragma once
#include "stdio.h"
#define  FILE_SAVE_PATH "E:\\result\\"  //���ݴ洢·��
#define  ATTITUDE_TRACK_CONFIG_PATH  "E:\\result\\AttitudeTrack.cfg"  //�����ļ��洢·��
#define nox 2  //״̬��������
#define noz 1  //״̬��������
#ifdef __cplusplus
extern "C"
{
#endif
// ���Թ۲�����
struct IMUdata
{
	double imutimetarget;   //IMU�����ʱ�������/���룬100Hz���������ʱ����
	double accx;            //���ٶȼ�X�����   g
	double accy;            //        Y         g
	double accz;            //        Z         g
	double gyox;            //����X�������     deg/s
	double gyoy;            //    Y             deg/s
	double gyoz;            //    Z             deg/s
	int state_acc;          //IMU��־λ��������ٶȼƳ�����״̬λ
};
typedef struct IMUdata IMUdata_t;
// �������̹۲�����
struct Compassdata
{
	int comass_type;        //1 ƽ���������  2 ��ά��������
	double accx;            //���ٶȼ�X�����   g
	double accy;            //        Y         g
	double accz;            //        Z         g
	double mx;              //������X�������     ��˹
	double my;              //      Y             ��˹
	double mz;              //      Z             ��˹
};
typedef struct Compassdata Compassdata_t;
//GNSS�۲�����
struct GNSSdata
{
	double gnsstimetarget;   //GNSS�۲��ʱ����������룬20Hz
	double lat;              //γ��, WGS84       deg
	double lon;              //���ȣ�WGS84       deg
	double alt;              //�̣߳�WGS84        m
	double gnss_v[3];        //gnss�ٶȣ�ecef/enu   m/s
	double heading;          //˫���ߺ���        deg
	int state_pos;           //GNSSλ�ý���״̬
	int state_yaw;           //GNSS�����״̬
	int heading_flag;        //gnss��������  0.˫���� 1.������
};
typedef struct GNSSdata GNSSdata_t;
//�����ļ���Ϣ
struct Config
{
	double abiasx;           //���ٶȼ�x�ᾲ̬��ƫ  g
	double abiasy;           //���ٶȼ�y�ᾲ̬��ƫ  g
	double abiasz;           //���ٶȼ�z�ᾲ̬��ƫ  g
	double gstdxthr;         //�����Ǿ�̬��ֵ       deg/s
	double gstdythr;         //�����Ǿ�̬��ֵ       deg/s
	double gstdzthr;         //�����Ǿ�̬��ֵ       deg/s
	double astdxthr;         //���ٶȼƾ�̬��ֵ       g
	double astdythr;         //���ٶȼƾ�̬��ֵ       g
	double astdzthr;         //���ٶȼƾ�̬��ֵ       g
	double gnoisex;          //x����ٶ�����         rad
	double gnoisey;          //y����ٶ�����         rad
	double gnoisez;          //z����ٶ�����         rad
	double gstabilityx;     //x����������ƫ�ȶ���    rad/h
	double gstabilityy;     //x����������ƫ�ȶ���    rad/h
	double gstabilityz;     //x����������ƫ�ȶ���    rad/h
	double angle_err_init;      //�Ƕ�����ʼֵ         rad
	double bias_err_init;       //��ƫ����ʼֵ         rad
	double gnoisestd;       //�㶨��������std        rad
	double gnssyawstd_thr;  //˫���ߺ������ֵ       rad
	double gnssyawvar;      //˫���ߺ������������   rad
	double install_roll;    //����ǰ�װ���         deg
	double install_pitch;   //�����ǰ�װ���         deg
	double install_heading; //����ǰ�װ���         deg
	double acc_norm;        //���ż��ٶ�
	int ekf_strategy;       //ekf������������ 1.���ٶ�ƽ���˲�   2.����̬����  3.��������
	int angle_flag;         //����Ƕ�����  1��IMU����Ƕȸ��� 2����������+�ⲿ�Ƕ�
};
typedef struct  Config Config_t;
//��̬���ٹ۲�����
struct ATdata
{
	IMUdata_t imu;           //IMU�۲�����
	GNSSdata_t gnss;         //GNSS�۲�����
	Compassdata_t compass;	 //compass�۲�����
	Config_t config;         //�����ļ��ṹ��
	int bimu_updta;          //IMU�۲����ݸ��±�־��0-δ����  1-����
	int bgnss_updata;        //GNSS�۲����ݸ��±�־�� 0-δ���� 1-����
	int bcompass_updata;     //compass�۲����ݸ��±�־ �� 0-δ���� 1-����
};
typedef struct ATdata ATdata_t;

int  AngleTrack_Process(Config_t* conf, double gyo,double IMU_time,double ExterAngle, double* result);//���������ⲿ�Ƕ��ںϣ��������Ƕ�



int AngleTrack_Process_Demo(ATdata_t *atd, Config_t* conf, double* result);
int  IMU_AccAngle_Process(IMUdata_t *imu, Config_t* conf, double *result);
int  GnssHeading_Process(GNSSdata_t* gnss, double *result);//�����߼��㺽���
int  CompassHeading_Process(Compassdata_t* compass, double *result);//�������̼��㺽���
#ifdef __cplusplus
}
#endif