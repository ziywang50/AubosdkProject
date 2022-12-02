#include "stdafx.h"
#include "example.h"
#include <armadillo>
#include "yolo.h"

#define M_PI 3.14159265358979323846
#define ROBOT_ADDR "192.168.214.6"
#define ROBOT_PORT 8899

//aubo sdk下载下来就是乱码
//���ƹ��û��ģ�����
const char* USER_DI_00 = "U_DI_00";
const char* USER_DI_01 = "U_DI_01";
const char* USER_DI_02 = "U_DI_02";
const char* USER_DI_03 = "U_DI_03";
const char* USER_DI_04 = "U_DI_04";
const char* USER_DI_05 = "U_DI_05";
const char* USER_DI_06 = "U_DI_06";
const char* USER_DI_07 = "U_DI_07";
const char* USER_DI_10 = "U_DI_10";
const char* USER_DI_11 = "U_DI_11";
const char* USER_DI_12 = "U_DI_12";
const char* USER_DI_13 = "U_DI_13";
const char* USER_DI_14 = "U_DI_14";
const char* USER_DI_15 = "U_DI_15";
const char* USER_DI_16 = "U_DI_16";
const char* USER_DI_17 = "U_DI_17";

//���ƹ��û��ģ�����
const char* USER_DO_00 = "U_DO_00";
const char* USER_DO_01 = "U_DO_01";
const char* USER_DO_02 = "U_DO_02";
const char* USER_DO_03 = "U_DO_03";
const char* USER_DO_04 = "U_DO_04";
const char* USER_DO_05 = "U_DO_05";
const char* USER_DO_06 = "U_DO_06";
const char* USER_DO_07 = "U_DO_07";
const char* USER_DO_10 = "U_DO_10";
const char* USER_DO_11 = "U_DO_11";
const char* USER_DO_12 = "U_DO_12";
const char* USER_DO_13 = "U_DO_13";
const char* USER_DO_14 = "U_DO_14";
const char* USER_DO_15 = "U_DO_15";
const char* USER_DO_16 = "U_DO_16";
const char* USER_DO_17 = "U_DO_17";

const char* TOOL_IO_0 = "T_DI/O_00";
const char* TOOL_IO_1 = "T_DI/O_01"	;
const char* TOOL_IO_2 = "T_DI/O_02"	;
const char* TOOL_IO_3 = "T_DI/O_03"	;


void printRoadPoint(const aubo_robot_namespace::wayPoint_S  *wayPoint)
{
	std::cout<<"pos.x="<<wayPoint->cartPos.position.x<<std::endl;
	std::cout<<"pos.y="<<wayPoint->cartPos.position.y<<std::endl;
	std::cout<<"pos.z="<<wayPoint->cartPos.position.z<<std::endl;

	std::cout<<"ori.w="<<wayPoint->orientation.w<<std::endl;
	std::cout<<"ori.x="<<wayPoint->orientation.x<<std::endl;
	std::cout<<"ori.y="<<wayPoint->orientation.y<<std::endl;
	std::cout<<"ori.z="<<wayPoint->orientation.z<<std::endl;

	std::cout<<"joint_1="<<wayPoint->jointpos[0]*180.0/M_PI<<std::endl;
	std::cout<<"joint_2="<<wayPoint->jointpos[1]*180.0/M_PI<<std::endl;
	std::cout<<"joint_3="<<wayPoint->jointpos[2]*180.0/M_PI<<std::endl;
	std::cout<<"joint_4="<<wayPoint->jointpos[3]*180.0/M_PI<<std::endl;
	std::cout<<"joint_5="<<wayPoint->jointpos[4]*180.0/M_PI<<std::endl;
	std::cout<<"joint_6="<<wayPoint->jointpos[5]*180.0/M_PI<<std::endl;
}

void callback_RealTimeRoadPoint(const aubo_robot_namespace::wayPoint_S  *wayPoint, void *arg)
{
	printRoadPoint(wayPoint);
}

/************************************************************************/
/* 
   pos Ŀ��λ��x,y,z ��λ��
   joint6Angle 6��Ƕ�(��)
*/
/************************************************************************/
bool move_to(RSHD rshd, const Pos *pos, double joint6Angle)
{
	bool result = false;

	//���Ȼ�ȡ��ǰ·����Ϣ
	aubo_robot_namespace::wayPoint_S wayPoint;

	//���λ����Ϣ
	aubo_robot_namespace::wayPoint_S targetPoint;

	//Ŀ��λ�ö�Ӧ�Ĺؽڽ�
	double targetRadian[ARM_DOF] = {0};
	
	if (RS_SUCC == rs_get_current_waypoint(rshd, &wayPoint))
	{
		//�ο���ǰ��̬���õ������ؽڽ�
		if (RS_SUCC == rs_inverse_kin(rshd, wayPoint.jointpos, pos, &wayPoint.orientation, &targetPoint))
		{
			//���õ�Ŀ��λ��,��6�ؽڽǶ�����Ϊ�û������ĽǶȣ�������+-175�ȣ�
			targetRadian[0] = targetPoint.jointpos[0];
			targetRadian[1] = targetPoint.jointpos[1];
			targetRadian[2] = targetPoint.jointpos[2];
			targetRadian[3] = targetPoint.jointpos[3];
			targetRadian[4] = targetPoint.jointpos[4];
			//6��ʹ���û������ĹؽڽǶ�
			targetRadian[5] = joint6Angle/180*M_PI;

			//�ᶯ��Ŀ��λ��
			if (RS_SUCC == rs_move_joint(rshd, targetRadian))
			{
				std::cout<<"����Ŀ��λ��"<<std::endl;

				//��ȡ��ǰ�ؽڽǣ�������֤
				rs_get_current_waypoint(rshd, &wayPoint);

				printRoadPoint(&wayPoint);
			}
			else
			{
				std::cerr<<"move joint error"<<std::endl;
			}
		}
		else
		{
			std::cerr<<"ik failed"<<std::endl;
		}

	}
	else
	{
		std::cerr<<"get current waypoint error"<<std::endl;
	}

	return result;
}

/********************************************************************
	function:	example_login
	purpose :	��½��е��
	param   :	rshd ��������ľ��
				addr ��е�۷�������ַ
				port ��е�۷������˿�
	return  :	true �ɹ� false ʧ��
*********************************************************************/
bool example_login(RSHD &rshd, const char * addr, int port)
{
	bool result = false;

	rshd = RS_FAILED;

	//��ʼ���ӿڿ�
	if (rs_initialize() == RS_SUCC)
	{
		//����������
		if (rs_create_context(&rshd)  == RS_SUCC )
		{
			//��½��е�۷�����
			if (rs_login(rshd, addr, port) == RS_SUCC)
			{
				result = true;
				//��½�ɹ�
				std::cout<<"login succ"<<std::endl;
			}
			else
			{
				//��½ʧ��
				std::cerr<<"login failed"<<std::endl;				
			}
		}
		else
		{
			//����������ʧ��
			std::cerr<<"rs_create_context error"<<std::endl;
		}
	}
	else
	{
		//��ʼ���ӿڿ�ʧ��
		std::cerr<<"rs_initialize error"<<std::endl;
	}

	return result;
}

/********************************************************************
	function:	example_logout
	purpose :	�˳���½
	param   :	rshd �����ľ��
					
	return  :	true �ɹ� false ʧ��
*********************************************************************/
bool example_logout(RSHD rshd)
{
	return rs_logout(rshd)==RS_SUCC ? true : false;
}

/********************************************************************
	function:	example_robotStartup
	purpose :	�����е��(����������ʵ��е�ۣ�
	param   :	rshd �����ľ��
					
	return  :	true �ɹ� false ʧ��
*********************************************************************/
bool example_robotStartup(RSHD rshd)
{
	bool result = false;

	//���ߵĶ���ѧ�������˶�ѧ����
	ToolDynamicsParam tool_dynamics = {0};
	//��е����ײ�ȼ�
	uint8 colli_class = 6;
	//��е������Ƿ��ȡ��̬��Ĭ�Ͽ����
	bool read_pos = true;
	//��е�۾�̬��ײ��⣨Ĭ�Ͽ����
	bool static_colli_detect = true;
	//��е�������ٶȣ�ϵͳ�Զ����ƣ�Ĭ��Ϊ30000)
	int board_maxacc = 30000;
	//��е�۷������״̬
	ROBOT_SERVICE_STATE state = ROBOT_SERVICE_READY;

	if (rs_robot_startup(rshd, &tool_dynamics, colli_class, read_pos, static_colli_detect, board_maxacc, &state)
		== RS_SUCC)
	{
		result = true;
		std::cout<<"call robot startup succ, robot state:"<<state<<std::endl;
	}
	else
	{
		std::cerr<<"robot startup failed"<<std::endl;
	}

	return result;
}

/********************************************************************
	function:	example_robotShutdown
	purpose :	�رջ�е�ۣ�����������ʵ��е�ۣ�
	param   :	rshd �����ľ��
					
	return  :	true �ɹ� false ʧ��
*********************************************************************/
bool example_robotShutdown(RSHD rshd)
{
	return rs_robot_shutdown(rshd)==RS_SUCC ? true : false;
}

/********************************************************************
	function:	example_moveJ
	purpose :	��е���ᶯ����
	param   :	rshd �����ľ��
					
	return  :	true �ɹ� false ʧ��
*********************************************************************/
bool example_moveJ(RSHD rshd)
{
	bool result = false;

	RobotRecongnitionParam param;
	rs_get_robot_recognition_param(rshd, 1, &param);

	//��λ��Ϊ��е�۵ĳ�ʼλ�ã��ṩ6���ؽڽǵĹؽ���Ϣ����λ�����ȣ���
	double initPos[6]={
		-49.82/180*M_PI,
		29.86/180*M_PI,
		-63.45/180*M_PI,
		-38.09/180*M_PI,
		-92.40/180*M_PI,
		97.68/180*M_PI};

	//�����˶�����ʼλ��
	if (rs_move_joint(rshd, initPos) == RS_SUCC)
	{
		result = true;
		std::cout<<"movej succ"<<std::endl;
	}
	else
	{
		std::cerr<<"movej failed!"<<std::endl;
	}

	return result;
}

/********************************************************************
	function:	example_moveL
	purpose :	��е�۱��ֵ�ǰ��ֱ̬���˶�����
	param   :	rshd �����ľ��
					
	return  :	true �ɹ� false ʧ��
*********************************************************************/
bool example_moveL(RSHD rshd, aubo_robot_namespace::Pos pos)
{
	bool result = false;

	//�����ƶ�����ʼλ��
	//example_moveJ(rshd);

	//��ȡ��ǰ·����Ϣ
	aubo_robot_namespace::wayPoint_S wayPoint;

	//���λ����Ϣ
	aubo_robot_namespace::wayPoint_S targetPoint;

	//Ŀ��λ�ö�Ӧ�Ĺؽڽ�
	double targetRadian[ARM_DOF] = {0};

	//Ŀ��λ��
	//Pos pos = {-0.489605, 0.455672, 0.448430};

	if (RS_SUCC == rs_get_current_waypoint(rshd, &wayPoint))
	{
		//�ο���ǰ��̬���õ������ؽڽ�
		if (RS_SUCC == rs_inverse_kin(rshd, wayPoint.jointpos, &pos, &wayPoint.orientation, &targetPoint))
		{
			//���õ�Ŀ��λ��,��6�ؽڽǶ�����Ϊ�û������ĽǶȣ�������+-175�ȣ�
			targetRadian[0] = targetPoint.jointpos[0];
			targetRadian[1] = targetPoint.jointpos[1];
			targetRadian[2] = targetPoint.jointpos[2];
			targetRadian[3] = targetPoint.jointpos[3];
			targetRadian[4] = targetPoint.jointpos[4];
			targetRadian[5] = targetPoint.jointpos[5];

			//�ᶯ��Ŀ��λ��
			if (RS_SUCC == rs_move_line(rshd, targetRadian))
			{
				std::cout<<"at target"<<std::endl;
			}
			else
			{
				std::cerr<<"move joint error"<<std::endl;
			}
		}
		else
		{
			std::cerr<<"ik failed. Unable to move to positon"<<std::endl;
		}

	}
	else
	{
		std::cerr<<"get current waypoint error"<<std::endl;
	}


	return result;
}

/********************************************************************
	function:	example_moveP
	purpose :	��е�۹켣�˶�����
	param   :	rshd �����ľ��
					
	return  :	void
*********************************************************************/
void example_moveP(RSHD rshd)
{
	/** ģ��ҵ�� **/
	/** �ӿڵ���: ��ʼ���˶����� ***/
	rs_init_global_move_profile(rshd);

	/** �ӿڵ���: ���ùؽ����˶��������ٶ� ***/
	aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
	jointMaxAcc.jointPara[0] = 50.0/180.0*M_PI;
	jointMaxAcc.jointPara[1] = 50.0/180.0*M_PI;
	jointMaxAcc.jointPara[2] = 50.0/180.0*M_PI;
	jointMaxAcc.jointPara[3] = 50.0/180.0*M_PI;
	jointMaxAcc.jointPara[4] = 50.0/180.0*M_PI;
	jointMaxAcc.jointPara[5] = 50.0/180.0*M_PI;   //�ӿ�Ҫ��λ�ǻ���
	rs_set_global_joint_maxacc(rshd, &jointMaxAcc);

	/** �ӿڵ���: ���ùؽ����˶�������ٶ� ***/
	aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
	jointMaxVelc.jointPara[0] = 50.0/180.0*M_PI;
	jointMaxVelc.jointPara[1] = 50.0/180.0*M_PI;
	jointMaxVelc.jointPara[2] = 50.0/180.0*M_PI;
	jointMaxVelc.jointPara[3] = 50.0/180.0*M_PI;
	jointMaxVelc.jointPara[4] = 50.0/180.0*M_PI;
	jointMaxVelc.jointPara[5] = 50.0/180.0*M_PI;   //�ӿ�Ҫ��λ�ǻ���
	rs_set_global_joint_maxvelc(rshd, &jointMaxVelc);


	/** �ӿڵ���: ��ʼ���˶����� ***/
	rs_init_global_move_profile(rshd);

	/** �ӿڵ���: ����ĩ�����˶��������ٶ� ����ֱ���˶�����ĩ�����˶�***/
	double endMoveMaxAcc;
	endMoveMaxAcc = 0.2;   //��λ��ÿ��
	rs_set_global_end_max_line_acc(rshd, endMoveMaxAcc);
	rs_set_global_end_max_angle_acc(rshd, endMoveMaxAcc);


	/** �ӿڵ���: ����ĩ�����˶�������ٶ� ֱ���˶�����ĩ�����˶�***/
	double endMoveMaxVelc;
	endMoveMaxVelc = 0.2;   //��λ��ÿ��
	rs_set_global_end_max_line_velc(rshd, endMoveMaxVelc);
	rs_set_global_end_max_angle_velc(rshd, endMoveMaxVelc);

	double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};

	for(int i=0;i<2;i++)
	{
		//׼����  �ؽ��˶����ڹؽ����˶�
		rs_init_global_move_profile(rshd);
		rs_set_global_joint_maxacc(rshd, &jointMaxAcc);
		rs_set_global_joint_maxvelc(rshd, &jointMaxVelc);

		//�ؽ��˶���׼����
		jointAngle[0] = -0.000003;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;

		int ret = rs_move_joint(rshd, jointAngle);
		if(ret != RS_SUCC)
		{
			std::cerr<<"JointMoveʧ��.��ret:"<<ret<<std::endl;
		}

		//Բ��
		rs_init_global_move_profile(rshd);

		rs_set_global_end_max_line_acc(rshd, endMoveMaxAcc);
		rs_set_global_end_max_angle_acc(rshd, endMoveMaxAcc);
		rs_set_global_end_max_line_velc(rshd, endMoveMaxVelc);
		rs_set_global_end_max_angle_velc(rshd, endMoveMaxVelc);

		jointAngle[0] = -0.000003;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		jointAngle[0] = 0.200000;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570794;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		jointAngle[0] = 0.600000;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		rs_set_circular_loop_times(rshd, 0);
		if(RS_SUCC !=rs_move_track(rshd, ARC_CIR))
		{
			std::cerr<<"TrackMove failed.��ret:"<<ret<<std::endl;
		}

		//׼����
		rs_init_global_move_profile(rshd);
		rs_set_global_joint_maxacc(rshd, &jointMaxAcc);
		rs_set_global_joint_maxvelc(rshd, &jointMaxVelc);

		jointAngle[0] = -0.000003;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;

		//�ؽ��˶���׼����
		ret = rs_move_joint(rshd, jointAngle);
		if(RS_SUCC != ret)
		{
			std::cerr<<"JointMoveʧ��.��ret:"<<ret<<std::endl;
		}

		//Բ
		rs_init_global_move_profile(rshd);

		rs_set_global_end_max_line_acc(rshd, endMoveMaxAcc);
		rs_set_global_end_max_angle_acc(rshd, endMoveMaxAcc);
		rs_set_global_end_max_line_velc(rshd, endMoveMaxVelc);
		rs_set_global_end_max_angle_velc(rshd, endMoveMaxVelc);

		jointAngle[0] = -0.000003;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		jointAngle[0] = -0.211675;
		jointAngle[1] = -0.325189;
		jointAngle[2] = -1.466753;
		jointAngle[3] = 0.429232;
		jointAngle[4] = -1.570794;
		jointAngle[5] = -0.211680;
		rs_add_waypoint(rshd, jointAngle);

		jointAngle[0] = -0.037186;
		jointAngle[1] = -0.224307;
		jointAngle[2] = -1.398285;
		jointAngle[3] = 0.396819;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.037191;
		rs_add_waypoint(rshd, jointAngle);

		//Բ��Ȧ��
		rs_set_circular_loop_times(rshd, 1);
		ret = rs_move_track(rshd, ARC_CIR);
		if(RS_SUCC != ret)
		{
			std::cerr<<"TrackMove failed.��ret:"<<ret<<std::endl;
		}


		//׼����
		rs_init_global_move_profile(rshd);

		rs_set_global_joint_maxacc(rshd, &jointMaxAcc);
		rs_set_global_joint_maxvelc(rshd, &jointMaxVelc);

		jointAngle[0] = -0.000003;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;

		//�ؽ��˶���׼����
		if(RS_SUCC != rs_move_joint(rshd, jointAngle))
		{
			std::cerr<<"JointMoveʧ��.��ret:"<<ret<<std::endl;
		}

		//MoveP
		rs_init_global_move_profile(rshd);

		rs_set_global_end_max_line_acc(rshd, endMoveMaxAcc);
		rs_set_global_end_max_angle_acc(rshd, endMoveMaxAcc);
		rs_set_global_end_max_line_velc(rshd, endMoveMaxVelc);
		rs_set_global_end_max_angle_velc(rshd, endMoveMaxVelc);


		jointAngle[0] = -0.000003;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		jointAngle[0] = 0.100000;
		jointAngle[1] = -0.147267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570794;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		jointAngle[0] = 0.200000;
		jointAngle[1] = -0.167267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570794;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		//���ڰ뾶
		rs_set_blend_radius(rshd, 0.03);
		rs_set_circular_loop_times(rshd, 1);
		if(RS_SUCC !=rs_move_track(rshd, CARTESIAN_MOVEP))
		{
			std::cerr<<"TrackMove failed.��ret:"<<ret<<std::endl;
		}
	}
}

/********************************************************************
	function:	example_ik_fk
	purpose :	��е����������
	param   :	
					
	return  :	
*********************************************************************/
void example_ik_fk(RSHD rshd)
{
	aubo_robot_namespace::wayPoint_S wayPoint;

	ik_solutions solutions;

	double jointAngle[aubo_robot_namespace::ARM_DOF] = {-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008};
	
	//����
	if (RS_SUCC == rs_forward_kin(rshd, jointAngle, &wayPoint))
	{
		std::cout<<"fk succ"<<std::endl;

		printRoadPoint(&wayPoint);
	}

	//���
	double startPointJointAngle[aubo_robot_namespace::ARM_DOF] = {0.0/180.0*M_PI,  0.0/180.0*M_PI,  0.0/180.0*M_PI, 0.0/180.0*M_PI, 0.0/180.0*M_PI,0.0/180.0*M_PI};

	aubo_robot_namespace::Pos targetPosition;
	targetPosition.x =-0.48;
	targetPosition.y =-0.15;
	targetPosition.z = 0.44;

	aubo_robot_namespace::Rpy rpy;
	aubo_robot_namespace::Ori targetOri;

	rpy.rx = 180.0/180.0*M_PI;
	rpy.ry = 0.0/180.0*M_PI;
	rpy.rz = -90.0/180.0*M_PI;

	rs_rpy_to_quaternion(rshd, &rpy, &targetOri);

	if (RS_SUCC == rs_inverse_kin(rshd, startPointJointAngle, &targetPosition, &targetOri, &wayPoint))
	{
		std::cout<<"ik succ"<<std::endl;
		printRoadPoint(&wayPoint);
	}
	else
	{
		std::cerr<<"ik failed"<<std::endl;
	}

	if (RS_SUCC == rs_inverse_kin_closed_form(rshd, &targetPosition, &targetOri, &solutions))
	{
		std::cout<<"ik succ"<<std::endl;
		for (int i=0;i<solutions.solution_count;i++)
		{
			printRoadPoint(&solutions.waypoint[i]);
		}
	}
	else
	{
		std::cerr<<"ik failed"<<std::endl;
	}
}

/********************************************************************
	function:	example_boardIO
	purpose :	��е�ۿ��ƹ�IO����(����������ʵ��е�ۣ�
	param   :	rshd �����ľ��
					
	return  :	void
*********************************************************************/
void example_boardIO(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_00, IO_STATUS_VALID))
	{
		std::cout<<"set "<<USER_DO_00<<" succ"<<std::endl;

		if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_00, &status))
		{
			std::cout<<"get "<<USER_DO_00<<"="<<status<<std::endl;
		}
		else
		{
			std::cerr<<"get "<<USER_DO_00<<" failed"<<std::endl;
		}
	}
	else
	{
		std::cerr<<"set "<<USER_DO_00<<" failed"<<std::endl;
	}
}

//��е�۹��߶�IO����(����������ʵ��е�ۣ�
void example_ToolIO(RSHD rshd)
{
	double status = 0;

	//��������tool_io_0Ϊ�������
	if (RS_SUCC == rs_set_tool_io_type(rshd, TOOL_DIGITAL_IO_0, IO_OUT))
	{
		//����tool_io_0�������Ϊ��Ч
		if (RS_SUCC == rs_set_tool_do_status(rshd, TOOL_IO_0, IO_STATUS_VALID))
		{
			std::cout<<"set "<<TOOL_IO_0<<" succ"<<std::endl;
		}
		else
		{
			std::cerr<<"set "<<TOOL_IO_0<<" failed"<<std::endl;
		}
		
		//��ȡtool_io_0���������״̬
		if (RS_SUCC == rs_get_tool_io_status(rshd, TOOL_IO_0, &status))
		{
			std::cout<<"get "<<TOOL_IO_0<<"="<<status<<std::endl;
		}
		else
		{
			std::cerr<<"get "<<TOOL_IO_0<<" failed"<<std::endl;
		}
	}
}

/********************************************************************
	function:	example_callbackRobotRoadPoint
	purpose :	ʵʱ·����Ϣ�ص���������
	param   :	rshd �����ľ��
					
	return  :	true �ɹ� false ʧ��
*********************************************************************/
bool example_callbackRobotRoadPoint(RSHD rshd)
{
	bool result = false;

	//����ʵʱ·����Ϣ����
	if (RS_SUCC == rs_enable_push_realtime_roadpoint(rshd, true))
	{
		if (RS_SUCC == rs_setcallback_realtime_roadpoint(rshd, callback_RealTimeRoadPoint, NULL))
		{
			result = true;
		}
		else
		{
			std::cerr<<"call rs_setcallback_realtime_roadpoint failed"<<std::endl;
		}
	}
	else
		std::cerr<<"call rs_enable_push_realtime_roadpoint failed!"<<std::endl;
	return result;
}

void example_get_diagnosis(RSHD rshd)
{
	RobotDiagnosis *info = new RobotDiagnosis;  

	while(1)
	{
		memset((void*)info, 0, sizeof(RobotDiagnosis));  

		rs_get_diagnosis_info(rshd, info);

		std::cout<<"info.macTargetPosBufferSize="<<info->macTargetPosBufferSize<<" info.macTargetPosDataSize="<<info->macTargetPosDataSize<<std::endl;

		Sleep(500);
	}
}

aubo_robot_namespace::Pos convertcoordinates(RSHD rshd, std::vector<Output> result) {
	//find the yolo box with maximum confidence
	int maxconfindex = 0;
	float maxconf = result[0].confidence;
	for (int i = 0; i < result.size(); i++) {
		if (result[i].confidence >= maxconf) {
				maxconf = result[i].confidence;
				maxconfindex = i;
		}
	}
	int x_mid = result[maxconfindex].box.x + 0.5 * result[maxconfindex].box.width;
	int y_mid = result[maxconfindex].box.y + 0.5 * result[maxconfindex].box.height;
	//double zw = 0.42;
	//center coordinates for x, y
	double u = x_mid;
	double v = y_mid;
	//intrinsic parameters K
	arma::mat K = { {481.7598513211307, 0, 313.4083141573491}, {0, 478.726944051684, 223.362322547715}, {0, 0, 1} };
	//Put rotation result from easy_handeye here
	const Ori orientation = { 0.805, -0.530, 0.181, -0.191 };
	Rpy rpyangle;
	//convert quaternion to rpy
	int j = rs_quaternion_to_rpy(rshd, &orientation, &rpyangle);
	arma::mat rotx = { {1, 0, 0}, {0, cos(rpyangle.rx), -sin(rpyangle.rx)}, {0, sin(rpyangle.rx), cos(rpyangle.rx)} };
	arma::mat roty = { {cos(rpyangle.ry), 0, sin(rpyangle.ry)}, {0, 1, 0}, {-sin(rpyangle.ry), 0, cos(rpyangle.ry)} };
	arma::mat rotz = { {cos(rpyangle.rz), -sin(rpyangle.rz), 0}, {sin(rpyangle.rz), cos(rpyangle.rz), 0}, {0, 0, 1} };
	//not known rotation direction
	//multiply euler angles
	arma::mat R = rotz * roty * rotx;
	arma::mat imagec = { {u}, {v}, {1.0} };
	/*arma::mat invr = inv(R);
	arma::mat invk = inv(K);
	arma::mat inrk = invr * invk;
	arma::vec m1 = inrk * imagec;*/
	//T translation vector. Put translation result from easy_handeye here
	arma::mat T = { {-0.857}, {-0.14}, {0.364} };
	arma::mat down = { {0, 0, 0, 1} };
	arma::mat rt = join_rows(R, T);
	arma::mat O = join_cols(rt, down);
	O = inv(O);
	R = { {O(0, 0), O(0, 1), O(0, 2)}, {O(1, 0), O(1, 1), O(1, 2)}, {O(2, 0), O(2, 1), O(2, 2) } };
	T = { {O(0, 3)}, {O(1,3)}, {O(2, 3)} };
	arma::mat mat1 = inv(R) * inv(K) * imagec;
	arma::mat mat2 = inv(R) * T;
	std::cout << mat2(2);
	//zc is the z value of the robot's end effector on camera coordinate system.
	//double zc = (zw + mat2(2)) / mat1(2);
	double zc = 0.5;
	arma::vec world = zc * mat1 - mat2;
	aubo_robot_namespace::Pos pos1 = { world(0), world(1), world(2) };
	return pos1;
}