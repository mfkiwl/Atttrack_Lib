#include "stdafx.h"
#include "config.h"

JSON_Object        *s_cfg_obj = NULL;
JSON_Value         *s_cfg_val = NULL;
int init_config()
{
	s_cfg_val = json_parse_file(ATTITUDE_TRACK_CONFIG_PATH);
	s_cfg_obj = json_object(s_cfg_val);

	if (s_cfg_obj == NULL)
	{
		printf("Init config file error");
		return 0;
	}

	return 1;
}

void save_config()
{
	if (s_cfg_val)
	{
		json_serialize_to_file_pretty(s_cfg_val, ATTITUDE_TRACK_CONFIG_PATH);
	}
}

void creat_config()
{
	s_cfg_val = json_value_init_object();
	s_cfg_obj = json_object(s_cfg_val);
}

int config_set_gyo_staticbiasx(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gxstaticbias", num);
	save_config();
	return 1;
}
double config_get_gyo_staticbiasx()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gxstaticbias");
}

int config_set_gyo_staticbiasy(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gystaticbias", num);
	save_config();
	return 1;
}
double config_get_gyo_staticbiasy()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gystaticbias");
}

int config_set_gyo_staticbiasz(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gzstaticbias", num);
	save_config();
	return 1;
}
double config_get_gyo_staticbiasz()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gzstaticbias");
}

int config_set_acc_staticbiasx(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.axstaticbias", num);
	save_config();
	return 1;
}
double config_get_acc_staticbiasx()
{
	return json_object_dotget_number(s_cfg_obj, "imu.axstaticbias");
}

int config_set_acc_staticbiasy(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.aystaticbias", num);
	save_config();
	return 1;
}
double config_get_acc_staticbiasy()
{
	return json_object_dotget_number(s_cfg_obj, "imu.aystaticbias");
}

int config_set_acc_staticbiasz(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.azstaticbias", num);
	save_config();
	return 1;
}
double config_get_acc_staticbiasz()
{
	return json_object_dotget_number(s_cfg_obj, "imu.azstaticbias");
}


int config_set_gyo_stdx_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gxstdthr", num);
	save_config();
	return 1;
}
double config_get_gyo_stdx_thr()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gxstdthr");
}

int config_set_gyo_stdy_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gystdthr", num);
	save_config();
	return 1;
}
double config_get_gyo_stdy_thr()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gystdthr");
}

int config_set_gyo_stdz_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gzstdthr", num);
	save_config();
	return 1;
}
double config_get_gyo_stdz_thr()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gzstdthr");
}

int config_set_acc_stdx_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.axstdthr", num);
	save_config();
	return 1;
}
double config_get_acc_stdx_thr()
{
	return json_object_dotget_number(s_cfg_obj, "imu.axstdthr");
}

int config_set_acc_stdy_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.aystdthr", num);
	save_config();
	return 1;
}
double config_get_acc_stdy_thr()
{
	return json_object_dotget_number(s_cfg_obj, "imu.aystdthr");
}

int config_set_acc_stdz_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.azstdthr", num);
	save_config();
	return 1;
}
double config_get_acc_stdz_thr()
{
	return json_object_dotget_number(s_cfg_obj, "imu.azstdthr");
}

int config_set_gyo_noisex(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gxnoise", num);
	save_config();
	return 1;
}
double config_get_gyo_noisex()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gxnoise");
}

int config_set_gyo_noisey(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gynoise", num);
	save_config();
	return 1;
}
double config_get_gyo_noisey()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gynoise");
}

int config_set_gyo_noisez(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gznoise", num);
	save_config();
	return 1;
}
double config_get_gyo_noisez()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gznoise");
}

int config_set_acc_noisex(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.axnoise", num);
	save_config();
	return 1;
}
double config_get_acc_noisex()
{
	return json_object_dotget_number(s_cfg_obj, "imu.axnoise");
}

int config_set_acc_noisey(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.aynoise", num);
	save_config();
	return 1;
}
double config_get_acc_noisey()
{
	return json_object_dotget_number(s_cfg_obj, "imu.aynoise");
}

int config_set_acc_noisez(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.aznoise", num);
	save_config();
	return 1;
}
double config_get_acc_noisez()
{
	return json_object_dotget_number(s_cfg_obj, "imu.aznoise");
}

int config_set_gyo_stabilityx(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gxstability", num);
	save_config();
	return 1;
}
double config_get_gyo_stabilityx()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gxstability");
}

int config_set_gyo_stabilityy(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gystability", num);
	save_config();
	return 1;
}
double config_get_gyo_stabilityy()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gystability");
}

int config_set_gyo_stabilityz(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gzstability", num);
	save_config();
	return 1;
}
double config_get_gyo_stabilityz()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gzstability");
}

int config_set_acc_stabilityx(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.axstability", num);
	save_config();
	return 1;
}
double config_get_acc_stabilityx()
{
	return json_object_dotget_number(s_cfg_obj, "imu.axstability");
}

int config_set_acc_stabilityy(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.aystability", num);
	save_config();
	return 1;
}
double config_get_acc_stabilityy()
{
	return json_object_dotget_number(s_cfg_obj, "imu.aystability");
}

int config_set_acc_stabilityz(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.azstability", num);
	save_config();
	return 1;
}
double config_get_acc_stabilityz()
{
	return json_object_dotget_number(s_cfg_obj, "imu.azstability");
}

int config_set_gyo_shaftx(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gxshaft", num);
	save_config();
	return 1;
}
int config_get_gyo_shaftx()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gxshaft");
}

int config_set_gyo_shafty(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gyshaft", num);
	save_config();
	return 1;
}
int config_get_gyo_shafty()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gyshaft");
}

int config_set_gyo_shaftz(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu.gzshaft", num);
	save_config();
	return 1;
}
int config_get_gyo_shaftz()
{
	return json_object_dotget_number(s_cfg_obj, "imu.gzshaft");
}

int config_set_acc_shaftx(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu.axshaft", num);
	save_config();
	return 1;
}
int config_get_acc_shaftx()
{
	return json_object_dotget_number(s_cfg_obj, "imu.axshaft");
}

int config_set_acc_shafty(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu.ayshaft", num);
	save_config();
	return 1;
}
int config_get_acc_shafty()
{
	return json_object_dotget_number(s_cfg_obj, "imu.ayshaft");
}

int config_set_acc_shaftz(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu.azshaft", num);
	save_config();
	return 1;
}
int config_get_acc_shaftz()
{
	return json_object_dotget_number(s_cfg_obj, "imu.azshaft");
}

int config_set_ekf_strategy(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu.ekf_strategy", num);
	save_config();
	return 1;
}
int config_get_ekf_strategy()
{
	return json_object_dotget_number(s_cfg_obj, "imu.ekf_strategy");
}


int config_set_right_leverx(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.rightleverx", num);
	save_config();
	return 1;
}
double config_get_right_leverx()
{
	return json_object_dotget_number(s_cfg_obj, "imu.rightleverx");
}

int config_set_right_levery(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.rightlevery", num);
	save_config();
	return 1;
}
double config_get_right_levery()
{
	return json_object_dotget_number(s_cfg_obj, "imu.rightlevery");
}

int config_set_right_leverz(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.rightleverz", num);
	save_config();
	return 1;
}
double config_get_right_leverz()
{
	return json_object_dotget_number(s_cfg_obj, "imu.rightleverz");
}

int config_set_left_leverx(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.leftleverx", num);
	save_config();
	return 1;
}
double config_get_left_leverx()
{
	return json_object_dotget_number(s_cfg_obj, "imu.leftleverx");
}

int config_set_left_levery(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.leftlevery", num);
	save_config();
	return 1;
}
double config_get_left_levery()
{
	return json_object_dotget_number(s_cfg_obj, "imu.leftlevery");
}

int config_set_left_leverz(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu.leftleverz", num);
	save_config();
	return 1;
}
double config_get_left_leverz()
{
	return json_object_dotget_number(s_cfg_obj, "imu.leftleverz");
}

int config_set_gnssyawstd_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "gnss.yawstdthr", num);
	save_config();
	return 1;
}
double config_get_gnssyawstd_thr()
{
	return json_object_dotget_number(s_cfg_obj, "gnss.yawstdthr");
}

int config_set_gnssyaw_var(double num)
{
	json_object_dotset_number(s_cfg_obj, "gnss.yawvar", num);
	save_config();
	return 1;
}
double config_get_gnssyaw_var()
{
	return json_object_dotget_number(s_cfg_obj, "gnss.yawvar");
}

int config_set_install_roll(double num)
{
	json_object_dotset_number(s_cfg_obj, "install.roll", num);
	save_config();
	return 1;
}
double config_get_install_roll()
{
	return json_object_dotget_number(s_cfg_obj, "install.roll");
}

int config_set_install_pitch(double num)
{
	json_object_dotset_number(s_cfg_obj, "install.pitch", num);
	save_config();
	return 1;
}
double config_get_install_pitch()
{
	return json_object_dotget_number(s_cfg_obj, "install.pitch");
}
int config_set_install_heading(double num)
{
	json_object_dotset_number(s_cfg_obj, "install.heading", num);
	save_config();
	return 1;
}
double config_get_install_heading()
{
	return json_object_dotget_number(s_cfg_obj, "install.heading");
}



void generate_default_config_Angle()
{
	s_cfg_val = json_value_init_object();
	s_cfg_obj = json_object(s_cfg_val);

	config_set_gyo_staticbiasx(0);
	config_set_gyo_staticbiasy(0);
	config_set_gyo_staticbiasz(0);
	config_set_acc_staticbiasx(0);
	config_set_acc_staticbiasy(0);
	config_set_acc_staticbiasz(0);

	config_set_gyo_stdx_thr(0.001);         //具体数值
	config_set_gyo_stdy_thr(0.001);
	config_set_gyo_stdz_thr(0.001);
	config_set_acc_stdx_thr(0.05);
	config_set_acc_stdy_thr(0.05);
	config_set_acc_stdz_thr(0.05);

	config_set_gyo_noisex(0.025);     //deg
	config_set_gyo_noisey(0.025);     //deg
	config_set_gyo_noisez(0.025);     //deg
	config_set_acc_noisex(0.001);     //g
	config_set_acc_noisey(0.001);     //g
	config_set_acc_noisez(0.001);     //g

	config_set_gyo_stabilityx(8.0);   //dph
	config_set_gyo_stabilityy(8.0);   //dph
	config_set_gyo_stabilityz(8.0);   //dph
	config_set_acc_stabilityx(0.00001);  //g
	config_set_acc_stabilityy(0.00001);  //g
	config_set_acc_stabilityz(0.00001);  //g

	config_set_gyo_shaftx(0);
	config_set_gyo_shafty(1);
	config_set_gyo_shaftz(2);
	config_set_acc_shaftx(0);
	config_set_acc_shafty(1);
	config_set_acc_shaftz(2);
	config_set_ekf_strategy(1);

	config_set_gnssyawstd_thr(100.1); //deg
	config_set_gnssyaw_var(30.0);      //deg

	config_set_install_roll(0);
	config_set_install_pitch(0);
	config_set_install_heading(0);
	//关闭对象
	//json_serialize_to_file_pretty(s_cfg_val, GILC_TILTMEASURE_CONFIG_PATH);
	//json_value_free(s_cfg_val);
}


void get_config(AttTrackCfg_t* cfg)
{
	cfg->gbiasx = config_get_gyo_staticbiasx();  //具体数值
	cfg->gbiasy = config_get_gyo_staticbiasy();
	cfg->gbiasz = config_get_gyo_staticbiasz();
	cfg->abiasx = config_get_acc_staticbiasx();
	cfg->abiasy = config_get_acc_staticbiasy();
	cfg->abiasz = config_get_acc_staticbiasz();

	cfg->gstdxthr = config_get_gyo_stdx_thr();   //具体数值
	cfg->gstdythr = config_get_gyo_stdy_thr();
	cfg->gstdzthr = config_get_gyo_stdz_thr();
	cfg->astdxthr = config_get_acc_stdx_thr();
	cfg->astdythr = config_get_acc_stdy_thr();
	cfg->astdzthr = config_get_acc_stdz_thr();

	cfg->gnoisex = config_get_gyo_noisex();
	cfg->gnoisey = config_get_gyo_noisey();
	cfg->gnoisez = config_get_gyo_noisez();
	cfg->anoisex = config_get_acc_noisex();
	cfg->anoisey = config_get_acc_noisey();
	cfg->anoisez = config_get_acc_noisez();

	cfg->gstabilityx = config_get_gyo_stabilityx();
	cfg->gstabilityy = config_get_gyo_stabilityy();
	cfg->gstabilityz = config_get_gyo_stabilityz();
	cfg->astabilityx = config_get_acc_stabilityx();
	cfg->astabilityy = config_get_acc_stabilityy();
	cfg->astabilityz = config_get_acc_stabilityz();

	cfg->gshaftx = config_get_gyo_shaftx();
	cfg->gshafty = config_get_gyo_shafty();
	cfg->gshaftz = config_get_gyo_shaftz();
	cfg->ashaftx = config_get_acc_shaftx();
	cfg->ashafty = config_get_acc_shafty();
	cfg->ashaftz = config_get_acc_shaftz();
	cfg->ekf_strategy = config_get_ekf_strategy();


	cfg->gnssyawstd_thr = config_get_gnssyawstd_thr();
	cfg->gnssyawvar = config_get_gnssyaw_var();


	cfg->install_roll = config_get_install_roll();
	cfg->install_pitch = config_get_install_pitch();
	cfg->install_heading = config_get_install_heading();
}

//int set_config(AttTrackCfg_t* cfg)
//{
//	config_set_install_roll(0);
//	config_set_install_pitch(0);
//	config_set_install_heading(0);
//}