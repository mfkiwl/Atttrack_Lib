#include "DATAread.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define D2R (3.14159265358979323846/180.0)
int GAsensordataread1(struct chardata *charreaddata, FILE *file_fp)
{
	char NAME[3] ;
	memset(NAME, 0x00, sizeof(NAME));
	double time1;
	static double time0 = 0;
	char name1[] = "GN";
	char name2[] = "GA";
	fscanf(file_fp, "%s", test);
	
	strncpy(NAME, test,2);//获取每一行数据的名称
	//strncpy(NAME, NAME, 4);//获取每一行数据的名称
		charreaddata->gnss_pitch = 0;
		charreaddata->two_heading = 0;
	//printf("data=%s,%s\n", test,NAME);
	int flag = 0;
	int m, i, b[20];
	char ch;
	ch = ',';
	m = strlen(test);
	/*##########################GNSS数据############################*/
	if (strcmp(name1, NAME) == 0) //GNSS data
	{
		//printf("NAME=%s\n",NAME);
		//printf("GNSS_DATA=%s\n",test);
		for (i = 0;i < m;++i)
		{
			if (test[i] == ch)
			{
				b[flag] = i + 1;
				//printf("position=%d\n",b[flag]);
				//printf("flag=%d\n",flag);

				if (flag == 0)
				{
					strncpy(gv, test + b[flag], 10);
					charreaddata->gnss_pitch = atof(gv);
				}

				if (flag == 1)
				{
					strncpy(gv, test + b[flag], 10);

					charreaddata->two_heading = atof(gv);
				}

				//printf("two_ant_gnss_heading=%f\n", charreaddata->two_heading);
				flag += 1;
				//charreaddata->gpsUpdate = 1;
			}
		}
		return 1;
	}
	//else
	//{
	//	charreaddata->gnss_pitch = 0;
	//	charreaddata->two_heading = 0;
	//}
	/*##########################GAsensor数据############################*/
	if (strcmp(NAME, name2) == 0)   //GAsensor data
	{
		//printf("NAME=%s,name=%s\n",NAME,name2);

		for (i = 0;i < m;++i)
		{
			if (test[i] == ch)
			{
				b[flag] = i + 1;
				if (flag == 0)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->gyro_data[1] = -atof(gyro);
				}
				if (flag == 1)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->acc_data[0] = atof(gyro);
				}
				if (flag == 2)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->acc_data[1] = atof(gyro);
				}
				if (flag == 3)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->acc_data[2] = atof(gyro);
				}
				if (flag == 4)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->ga_time = atof(gyro);
				}
				if (flag == 5)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->imu_Ts = atof(gyro)/1000;
				}

				flag += 1;
			}
		}
		return 0;
	}

}



void IS203dataread(struct chardata *charreaddata, FILE *file_fp)
{
	char NAME[3];
	memset(NAME, 0x00, sizeof(NAME));
	double time1;
	static double time0 = 0;
	char name1[] = "GN";
	char name2[] = "IM";
	fscanf(file_fp, "%s", test);
	//printf("NAME=%s\n",test);
	strncpy(NAME, test,2);//获取每一行数据的名称
	int flag = 0;
	int m, i, b[20];
	char ch;
	ch = ',';
	m = strlen(test);
	/*##########################GNSS数据############################*/
	if (strcmp(name1, NAME) == 0) //GNSS data
	{
		//printf("NAME=%s\n",NAME);
		//printf("GNSS_DATA=%s\n",test);
		for (i = 0;i < m;++i)
		{
			if (test[i] == ch)
			{
				b[flag] = i + 1;
				//printf("position=%d\n",b[flag]);
				//printf("flag=%d\n",flag);
				if (flag == 1)
				{
					strncpy(gv, test + b[flag], 10);
					charreaddata->gnss_pos[0] = atof(gv);
				}
				if (flag == 2)
				{
					strncpy(gv, test + b[flag], 10);
					charreaddata->gnss_pos[1] = atof(gv);
				}
				if (flag == 3)
				{
					strncpy(gv, test + b[flag], 10);
					charreaddata->gnss_pos[2] = atof(gv);
				}
				if (flag == 4)
				{
					strncpy(gv, test + b[flag], 10);
					charreaddata->two_heading = atof(gv);
				}

				/*			int gps_sta;
				if (flag == 14)
				{
				strncpy(gv, test + b[flag], 3);
				gps_sta = atof(gv);
				}

				if (gps_sta == 11)
				{
				charreaddata->gps_pos_status = 4;
				charreaddata->gps_yaw_status = 4;
				}
				if (gps_sta == 6)
				{
				charreaddata->gps_pos_status = 5;
				charreaddata->gps_yaw_status = 5;
				}
				if (gps_sta == 5)
				{
				charreaddata->gps_pos_status = 2;
				charreaddata->gps_yaw_status = 2;
				}
				if (gps_sta == 4)
				{
				charreaddata->gps_pos_status = 1;
				charreaddata->gps_yaw_status = 1;
				}*/
				//printf("two_ant_gnss_heading=%f\n",two_ant_gnss_heading);
				flag += 1;
				charreaddata->gpsUpdate = 1;

			}
		}
	}
	else
	{
		charreaddata->gpsUpdate = 0;
		//charreaddata->gps_yaw_status = 0;
		charreaddata->gyro_data[2] = 0;
	}
	/*##########################IMU数据############################*/
	if (strcmp(NAME, name2) == 0)   //IMU data
	{
		//printf("NAME=%s,name=%s\n",NAME,name2);

		for (i = 0;i < m;++i)
		{
			if (test[i] == ch)
			{
				b[flag] = i + 1;
				if (flag == 0)
				{

					strncpy(ti, test + b[flag], 14);
					time1 = atof(ti);
					charreaddata->imu_Ts = time1;//imu时间间隔
												 //time0 = time1;
				}
				if (flag == 6)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->gyro_data[2] = atof(gyro);
				}
				if (flag == 5)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->gyro_data[1] = atof(gyro);
				}
				if (flag == 4)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->gyro_data[0] = atof(gyro);
				}
				if (flag == 1)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->acc_data[0] = atof(gyro);
				}
				if (flag == 2)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->acc_data[1] = atof(gyro);
				}
				if (flag == 3)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->acc_data[2] = atof(gyro);
				}

				flag += 1;
			}
		}
	}

}



void GAdataread(struct chardata *charreaddata, FILE *file_fp)
{
	char NAME[3];
	memset(NAME, 0x00, sizeof(NAME));
	double time1;
	static double time0 = 0;
	char name1[] = "GN";
	char name2[] = "IM";
	fscanf(file_fp, "%s", test);
	//printf("NAME=%s\n",test);
	strncpy(NAME, test, 2);//获取每一行数据的名称
	int flag = 0;
	int m, i, b[20];
	char ch;
	ch = ',';
	m = strlen(test);
	/*##########################IMU数据############################*/

		for (i = 0;i < m;++i)
		{
			if (test[i] == ch)
			{
				b[flag] = i + 1;
				if (flag == 4)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->gyro_data[1] = atof(gyro);
				}
				if (flag == 5)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->acc_data[0] = atof(gyro);
				}
				if (flag == 6)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->acc_data[1] = atof(gyro);
				}
				if (flag == 7)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->acc_data[2] = atof(gyro);
				}
				if (flag == 8)
				{
					strncpy(gyro, test + b[flag], 8);
					charreaddata->imu_Ts = atof(gyro);
				}
				flag += 1;
			}
		}

}

void IS203dataread1(struct chardata *charreaddata, FILE *file_fp)
{
	char NAME[3];
	memset(NAME, 0x00, sizeof(NAME));
	double time1;
	static double time0 = 0;
	char name1[] = "GN";
	char name2[] = "IM";
	fscanf(file_fp, "%s", test);
	//printf("NAME=%s\n",test);
	strncpy(NAME, test, 2);//获取每一行数据的名称
	int flag = 0;
	int m, i, b[20];
	char ch;
	ch = ',';
	m = strlen(test);
	/*##########################IMU数据############################*/

	for (i = 0;i < m;++i)
	{
		if (test[i] == ch)
		{
			b[flag] = i + 1;
			if (flag == 3)
			{
				strncpy(gyro, test + b[flag], 8);
				charreaddata->gyro_data[0] = atof(gyro);
			}
			if (flag == 4)
			{
				strncpy(gyro, test + b[flag], 8);
				charreaddata->gyro_data[1] = atof(gyro);
			}
			if (flag == 5)
			{
				strncpy(gyro, test + b[flag], 8);
				charreaddata->gyro_data[2] = atof(gyro);
			}
			if (flag ==6)
			{
				strncpy(gyro, test + b[flag], 8);
				charreaddata->acc_data[0] = atof(gyro);
			}
			if (flag == 7)
			{
				strncpy(gyro, test + b[flag], 8);
				charreaddata->acc_data[1] = atof(gyro);
			}
			if (flag == 8)
			{
				strncpy(gyro, test + b[flag], 8);
				charreaddata->acc_data[2] = atof(gyro);
			}
			//if (flag == 8)
			//{
			//	strncpy(gyro, test + b[flag], 8);
			//	charreaddata->imu_Ts = atof(gyro);
			//}
			flag += 1;
		}
	}

}



double getmean(double inputdata[], int NUM)
{
	int i;
	double sum = 0.0;
	double aver = 0.0;
	for (i = 0;i <= NUM;i++)
	{
		sum += inputdata[i];
		//printf("sum=%f\n",sum);
	}
	if (NUM == 0)
	{
		aver = 0;
	}
	else
	{
		aver = sum / (NUM);
	}

	//printf("aver=%f,%d\n", aver,NUM);
	return aver;
}

void IMUstructinit(struct chardata *charreaddata)
{
	charreaddata->acc_data[0] = 0;
	charreaddata->acc_data[1] = 0;
	charreaddata->acc_data[2] = 0;
	charreaddata->gyro_data[0] = 0;
	charreaddata->gyro_data[1] = 0;
	charreaddata->gyro_data[2] = 0;
}


void GAsensordataread(struct chardata *charreaddata, FILE *file_fp)
{
	/*##########################GA数据############################*/
	int flag = 0;
	char ch;
	int m, i, b[20];
	ch = ',';
	fgets(test, 1024, file_fp);
	//printf("data:%s\n", test);
	m = strlen(test);
	for (i = 0;i < m;++i)
	{
		if (test[i] == ch)
		{
			b[flag] = i + 1;
			strncpy(imuchar, test + 5, 8);
			charreaddata->gyro_data[1] = atof(imuchar) *D2R / 1000;
			if (flag == 0)
			{
				strncpy(imuchar, test + b[flag] + 7, 8);
				charreaddata->acc_data[0] = atof(imuchar) / 1000;
			}
			if (flag == 1)
			{
				strncpy(imuchar, test + b[flag] + 7, 8);
				charreaddata->acc_data[1] = atof(imuchar) / 1000;
			}
			if (flag == 2)
			{
				strncpy(imuchar, test + b[flag] + 7, 8);
				charreaddata->acc_data[2] = atof(imuchar) / 1000;
			}

			flag += 1;
		}
		//printf("%d,%d,%d,%d\n", b[0], b[1], b[2],m);
	}

}


void GAdataread1(struct chardata *charreaddata, FILE *file_fp)
{
	char NAME[3];
	memset(NAME, 0x00, sizeof(NAME));
	double time1;
	static double time0 = 0;
	char name1[] = "GN";
	char name2[] = "IM";
	fscanf(file_fp, "%s", test);
	//printf("NAME=%s\n",test);
	strncpy(NAME, test, 2);//获取每一行数据的名称
	int flag = 0;
	int m, i, b[20];
	char ch;
	ch = ',';
	m = strlen(test);
	/*##########################IMU数据############################*/

	for (i = 0;i < m;++i)
	{
		if (test[i] == ch)
		{

			b[flag] = i + 1;
			if (flag == 0)
			{
				strncpy(gyro, test + b[flag], 8);
				charreaddata->gyro_data[1] = atof(gyro)/50;
			}
			if (flag == 1)
			{
				strncpy(gyro, test + b[flag], 8);
				charreaddata->acc_data[0] = atof(gyro)/5886;
			}
			if (flag == 2)
			{
				strncpy(gyro, test + b[flag], 8);
				charreaddata->acc_data[1] = atof(gyro) / 5886;
			}
			if (flag == 3)
			{
				strncpy(gyro, test + b[flag], 8);
				charreaddata->acc_data[2] = atof(gyro) / 5886;
			}
			flag += 1;
		}
	}

}


void BDFdataread(struct chardata *charreaddata, FILE *file_fp)
{
	/*##########################BDF06数据############################*/
	int flag = 0;
	char ch;
	int m, i, b[20];
	ch = ',';
	fscanf(file_fp, "%s", test);
	//printf("%s\n", test);
	m = strlen(test);
	for (i = 0;i < m;++i)
	{
		if (test[i] == ch)
		{
			b[flag] = i + 1;
			strncpy(imuchar, test, 8);
			charreaddata->acc_data[0] = atof(imuchar);
			if (flag == 0)
			{
				strncpy(imuchar, test + b[flag], 8);
				charreaddata->acc_data[1] = atof(imuchar);
			}
			if (flag == 1)
			{
				strncpy(imuchar, test + b[flag], 8);
				charreaddata->acc_data[2] = atof(imuchar);
			}
			if (flag == 5)
			{
				strncpy(imuchar, test + b[flag], 8);
				charreaddata->bdf_data[0] = atof(imuchar);
			}
			if (flag == 6)
			{
				strncpy(imuchar, test + b[flag], 8);
				charreaddata->bdf_data[1] = atof(imuchar);
			}
			if (flag == 7)
			{
				strncpy(imuchar, test + b[flag], 8);
				charreaddata->bdf_data[2] = atof(imuchar);
			}
			if (flag == 2)
			{
				strncpy(imuchar, test + b[flag], 8);
				charreaddata->gyro_data[0] = atof(imuchar)*D2R;
			}
			if (flag == 3)
			{
				strncpy(imuchar, test + b[flag], 8);
				charreaddata->gyro_data[1] = atof(imuchar)*D2R;
			}
			if (flag == 4)
			{
				strncpy(imuchar, test + b[flag], 8);
				charreaddata->gyro_data[2] = atof(imuchar)*D2R;
			}


			flag += 1;
		}
	}

}