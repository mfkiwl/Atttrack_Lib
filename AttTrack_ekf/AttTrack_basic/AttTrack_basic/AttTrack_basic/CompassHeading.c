#include "CompassHeading.h"
int  CompassHeading_Process(Compassdata_t* compass, double *result)
{
	if (compass->comass_type == 1)//ƽ���������
	{
		double compass_mag[3] = { 0 };
		compass_mag[0] = compass->mx;//��������ϵѡNED
		compass_mag[1] = compass->my;
		compass_mag[2] = compass->mz;
		double compass_heading = 0;
		compass_heading = atan2(compass_mag[1], compass_mag[0]);//����180��  �ű������
		*result = compass_heading;
	}
	if (compass->comass_type == 2)//��ά��������
	{
		double compass_mag[3] = { 0 };
		double compass_acc[3] = { 0 };
		compass_mag[0] = compass->mx;//��������ϵѡNED
		compass_mag[1] = compass->my;
		compass_mag[2] = compass->mz;
		compass_acc[0] = compass->accx;//��������ϵѡNED
		compass_acc[1] = compass->accy;
		compass_acc[2] = compass->accz;
		double pitch = 0;
		double roll = 0;
		double heading = 0;
		double compass_heading = 0;
		pitch = atan2(compass_acc[0], sqrt(compass_acc[1] * compass_acc[1] + compass_acc[2] * compass_acc[2]));
		roll = atan2(-compass_acc[1], -compass_acc[2]);
		heading= atan2(compass_mag[1], compass_mag[0]);//����180��
		compass_heading = atan2((compass_mag[1] * cos(roll) - compass_mag[2] * sin(roll)), (compass_mag[0] * cos(pitch) + compass_mag[1] * sin(pitch)*sin(roll) + compass_mag[2] * sin(pitch)*cos(roll)));
		*result=compass_heading;
	}
	return 1;
}