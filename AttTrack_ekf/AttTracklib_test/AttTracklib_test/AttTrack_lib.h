#pragma once
#include "stdio.h"
#define  FILE_SAVE_PATH "E:\\result\\"
#define  ATTITUDE_TRACK_CONFIG_PATH  "E:\\result\\AttitudeTrack.cfg"
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
	int state_acc;          //���ٶȼƳ�����״̬λ
};
typedef struct IMUdata IMUdata_t;
// �������̹۲�����
struct Compassdata
{
	int comass_type;        //1 ƽ���������  2 ��ά��������
	double accx;            //���ٶȼ�X�����   g
	double accy;            //        Y         g
	double accz;            //        Z         g
	double mx;              //������X�������     
	double my;              //      Y             
	double mz;              //      Z          
};
typedef struct Compassdata Compassdata_t;
//GNSS�۲�����
struct GNSSdata
{
	double gnsstimetarget;   //GNSS�۲��ʱ����������룬20Hz
	double lat;              //γ��, WGS84       deg
	double lon;              //���ȣ�WGS84       deg
	double alt;              //�̣߳�WGS84        m
	double gnssyaw;          //˫���ߺ��򣬱�ƫ��(-180 - 180)      deg
	double gnss_v[3];        //gnss�ٶȣ�ecef/enu   m/s
	double speed;            //����              m/s
	double heading;          //˫���ߺ���        rad
	double speed_ver;        //�߳��ٶ�          m/s
	int state_pos;           //GNSSλ�ý�״̬
	int state_yaw;           //GNSS�����״̬
	int heading_flag;        //gnss��������  0.˫���� 1.������
};
typedef struct GNSSdata GNSSdata_t;
//��̬���ٹ۲�����
struct ATdata
{
	IMUdata_t imu;           //IMU�۲�����
	GNSSdata_t gnss;         //GNSS�۲�����
	Compassdata_t compass;	 //compass�۲�����
	int Angle_flag;          //1.����Ƕȸ��� 2.˫��Ƕȸ��� 3.��ά�Ƕȸ���
	int bgnss_updata;        //GNSS�۲����ݸ��±�־�� 0-δ���� 1-����
	int bcompass_updata;     //compass�۲����ݸ��±�־ �� 0-δ���� 1-����
};
typedef struct ATdata ATdata_t;

struct Result
{
	double roll;
	double pitch;
	double heading;
};
typedef struct Result result_t;

/******************************EKF�Ƕȸ���ͨ�ýӿ�************************************/
int  AngleTrack_Process(ATdata_t* atdata, result_t* res);//ͨ�ýӿڣ�����Ƕȿ�ѡ,��Ҫ����



/******************************���Ƕȸ��ٽӿ�************************************/
int  AngleTrack_Process_UEniaxialAngle(IMUdata_t* imu,double ExterAngle, double* result);//���������ⲿ�Ƕ��ںϣ��������Ƕ�
int  AngleTrack_Process_UniaxialAngle(IMUdata_t* imu,  double* result);//����������ٶȼ��ںϣ��������Ƕ�
/******************************˫��Ƕȸ��ٽӿ�************************************/
int  AngleTrack_Process_DualAngle(IMUdata_t* imu, double result[2]);//IMU��̬���٣���������������
/******************************��ά�Ƕȸ��ٽӿ�************************************/
int  AngleTrack_Process_3DAngle(IMUdata_t* imu,double ExterHeading,double result[3]);//IMU���ⲿ������ںϣ��������������������
int  GnssHeading_Process(GNSSdata_t* gnss, double *result);//�����߼��㺽���
int  CompassHeading_Process(Compassdata_t* compass, double *result);//�������̼��㺽���
#ifdef __cplusplus
}
#endif