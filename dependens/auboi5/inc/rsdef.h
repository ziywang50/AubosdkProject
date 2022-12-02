#ifndef RSDEF_H
#define RSDEF_H
#include "rstype.h"
#include "AuboRobotMetaType.h"
#include <vector>

#define TRUE             1
#define FALSE            0
#define RS_SUCC          0
#define RS_FAILED       -1
#define MAX_RS_INSTANCE  32
#define POS_SIZE         3
#define ORI_SIZE         4
#define INERTIA_SIZE     6

#define RS_UNUSED(x)     (void)x;
//#define _DEBUG

using namespace  aubo_robot_namespace;

typedef CoordCalibrateByJointAngleAndTool CoordCalibrate;

typedef struct{
    double rotateAxis[3];
}Move_Rotate_Axis;

typedef struct{
	wayPoint_S waypoint[8];
	int solution_count;
}ik_solutions;

#ifdef __cplusplus
extern "C" {
#endif

//library initialize and uninitialize
/**
 * @brief ��ʼ����е�ۿ��ƿ�
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_initialize(void);

/**
 * @brief ����ʼ����е�ۿ��ƿ�
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_uninitialize(void);

//robot service context
/**
 * @brief ������е�ۿ��������ľ��
 * @param rshd
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_create_context(RSHD *rshd/*returned context handle*/);

/**
 * @brief ע����е�ۿ��������ľ��
 * @param rshd
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_destory_context(RSHD rshd);

//login logout
/**
 * @brief ���ӻ�е�۷�����
 * @param rshd е�ۿ��������ľ��
 * @param addr ��е�۷�������IP��ַ
 * @param port ��е�۷������Ķ˿ں�
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_login(RSHD rshd, const char * addr, int port);

/**
 * @brief �Ͽ���е�۷���������
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_logout(RSHD rshd);

/**
 * @brief ��ȡ��ǰ������״̬
 * @param rshd е�ۿ��������ľ��
 * @param status true ���� false ����
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_login_status(RSHD rshd, bool *status);

//set move profile
/**
 * @brief ��ʼ��ȫ�ֵ��˶�����
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_init_global_move_profile(RSHD rshd);

//joint max acc,velc
/**
 * @brief ���������ؽڵ������ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param max_acc �����ؽڵ������ٶȣ���λ(rad/ss)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_global_joint_maxacc(RSHD rshd, const JointVelcAccParam  *max_acc);

/**
 * @brief ���������ؽڵ�����ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param max_velc �����ؽڵ�����ٶȣ���λ(rad/s)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_global_joint_maxvelc(RSHD rshd, const JointVelcAccParam *max_velc);

/**
 * @brief ��ȡ�����ؽڵ������ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param max_acc ���������ؽڵ������ٶȵ�λ(rad/s^2)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_global_joint_maxacc(RSHD rshd, JointVelcAccParam  *max_acc);

/**
 * @brief ���������ؽڵ�����ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param max_velc ���������ؽڵ����Ӷȵ�λ(rad/s)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_global_joint_maxvelc(RSHD rshd, JointVelcAccParam *max_velc);

//end line max acc,velc
/**
 * @brief ���û�е��ĩ������߼��ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param max_acc ĩ���������ٶȣ���λ(m/s^2)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_global_end_max_line_acc(RSHD rshd, double max_acc);

/**
 * @brief ���û�е��ĩ��������ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param max_velc ĩ��������ٶȣ���λ(m/s)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_global_end_max_line_velc(RSHD rshd, double max_velc);

/**
 * @brief ��ȡ��е��ĩ������߼��ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param max_acc ��е��ĩ������߼��ٶȣ���λ(m/s^2)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_global_end_max_line_acc(RSHD rshd, double *max_acc);

/**
 * @brief ��ȡ��е��ĩ��������ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param max_velc ��е��ĩ��������ٶȣ���λ(m/s)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_global_end_max_line_velc(RSHD rshd, double *max_velc);

//end line max acc,velc
/**
 * @brief ���û�е��ĩ�����Ǽ��ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param max_acc ĩ�����Ǽ��ٶȣ���λ(rad/s^2)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_global_end_max_angle_acc(RSHD rshd, double max_acc);

/**
 * @brief ���û�е��ĩ�������ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param max_velc ĩ������ٶȣ���λ(rad/s)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_global_end_max_angle_velc(RSHD rshd, double max_velc);

/**
 * @brief ��ȡ��е��ĩ�����Ǽ��ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param max_acc ��е��ĩ�����Ǽ��ٶȣ���λ(m/s^2)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_global_end_max_angle_acc(RSHD rshd, double *max_acc);

/**
 * @brief ��ȡ��е��ĩ�����Ǽ��ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param max_velc ��е��ĩ�������ٶȣ���λ(m/s)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_global_end_max_angle_velc(RSHD rshd, double *max_velc);

//robot move
/**
 * @brief ��е���ᶯ
 * @param rshd е�ۿ��������ľ��
 * @param joint_radian �����ؽڵĹؽڽǣ���λ(rad)
 * @param isblock    isblock==true  ������������е���˶�ֱ������Ŀ��λ�û��߳��ֹ��Ϻ󷵻ء�
 *                   isblock==false ������������������أ��˶�ָ��ͳɹ��ͷ��أ��������غ��е�ۿ�ʼ�˶���
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_move_joint(RSHD rshd, double joint_radian[ARM_DOF], bool isblock = true);

/**
 * @brief ����ģʽ��е���ᶯ
 * @param rshd е�ۿ��������ľ��
 * @param joint_radian �����ؽڵĹؽڽǣ���λ(rad)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_follow_mode_move_joint(RSHD rshd, double joint_radian[ARM_DOF]);

/**
 * @brief ��е�۱��ֵ�ǰ��ֱ̬���˶�
 * @param rshd е�ۿ��������ľ��
 * @param joint_radian �����ؽڵĹؽڽǣ���λ(rad)
 * @param isblock    isblock==true  ������������е���˶�ֱ������Ŀ��λ�û��߳��ֹ��Ϻ󷵻ء�
 *                   isblock==false ������������������أ��˶�ָ��ͳɹ��ͷ��أ��������غ��е�ۿ�ʼ�˶���
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_move_line(RSHD rshd, double joint_radian[ARM_DOF], bool isblock = true);

/**
 * @brief ���ֵ�ǰλ�ñ任��̬����ת�˶�
 * @param rshd е�ۿ��������ľ��
 * @param user_coord �û�����ϵ
 * @param rotate_axis :ת��(x,y,z) ���磺(1,0,0)��ʾ��Y��ת��
 * @param rotate_angle ��ת�Ƕ� ��λ��rad��
 * @param isblock    isblock==true  ������������е���˶�ֱ������Ŀ��λ�û��߳��ֹ��Ϻ󷵻ء�
 *                   isblock==false ������������������أ��˶�ָ��ͳɹ��ͷ��أ��������غ��е�ۿ�ʼ�˶���
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_move_rotate(RSHD rshd, const CoordCalibrate *user_coord, const Move_Rotate_Axis *rotate_axis, double rotate_angle,  bool isblock = true);

/**
 * @brief ��������Ѿ����õ�ȫ��·��
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_remove_all_waypoint(RSHD rshd);

/**
 * @brief ���ȫ��·�����ڹ켣�˶�
 * @param rshd е�ۿ��������ľ��
 * @param joint_radian �����ؽڵĹؽڽǣ���λ(rad)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_add_waypoint(RSHD rshd, double joint_radian[ARM_DOF]);

/**
 * @brief ���ý��ڰ뾶
 * @param rshd е�ۿ��������ľ��
 * @param radius ���ڰ뾶����λ(m)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_blend_radius(RSHD rshd, double radius);

/**
 * @brief ����Բ�˶�Ȧ��
 * @param rshd е�ۿ��������ľ��
 * @param times ��times����0ʱ����е�۽���Բ�˶�times��
 *              ��times����0ʱ����е�۽���Բ���켣�˶�
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_circular_loop_times(RSHD rshd, int times);

/**
 * @brief �����û�����ϵ
 * @param rshd е�ۿ��������ľ��
 * @param user_coord �û�����ϵ
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_user_coord(RSHD rshd, const CoordCalibrate *user_coord);

/**
 * @brief ���û�������ϵ
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_base_coord(RSHD rshd);

/**
 * @brief ����û�����ϵ���������Ƿ����
 * @param rshd е�ۿ��������ľ��
 * @param user_coord �û�����ϵ
 * @return RS_SUCC �ɹ� ����ʧ�� �������: 0 ���������: ����
 */
int rs_check_user_coord(RSHD rshd, const CoordCalibrate *user_coord);

/**
 * @brief ���û��ڻ�����ϵ�˶�ƫ����
 * @param rshd е�ۿ��������ľ��
 * @param relative ���λ��(x, y, z) ��λ(m)
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_relative_offset_on_base(RSHD rshd, const MoveRelative *relative);

/**
 * @brief ���û����û���ϵ�˶�ƫ����
 * @param rshd е�ۿ��������ľ��
 * @param relative ���λ��(x, y, z) ��λ(m)
 * @param user_coord �û�����ϵ
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_relative_offset_on_user(RSHD rshd, const MoveRelative *relative, const CoordCalibrate *user_coord);

/**
 * @brief ȡ����ǰ��λ����
 * @param rshd
 * @return
 */
int rs_set_no_arrival_ahead(RSHD rshd);

/**
 * @brief ���þ���ģʽ�µ���ǰ��λ����
 * @param rshd
 * @param distance����ǰ��λ���� ��λ���ף�
 * @return
 */
int rs_set_arrival_ahead_distance(RSHD rshd, double distance);

/**
 * @brief ����ʱ��ģʽ�µ���ǰ��λʱ��
 * @param rshd
 * @param sec ��ǰ��λʱ�䡡��λ���룩
 * @return
 */
int rs_set_arrival_ahead_time(RSHD rshd, double sec);

/**
 * @brief ���þ���ģʽ�½��ڰ뾶����
 * @param rshd
 * @param radius�����ڰ뾶���� ��λ���ף�
 * @return
 */
int rs_set_arrival_ahead_blend(RSHD rshd, double radius);

/**
 * @brief �켣�˶�
 * @param rshd е�ۿ��������ľ��
 * @param sub_move_mode �켣����:
 *                      2:Բ��
 *                      3:�켣
 * @param isblock    isblock==true  ������������е���˶�ֱ������Ŀ��λ�û��߳��ֹ��Ϻ󷵻ء�
 *                   isblock==false ������������������أ��˶�ָ��ͳɹ��ͷ��أ��������غ��е�ۿ�ʼ�˶���
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_move_track(RSHD rshd, move_track sub_move_mode, bool isblock = true);

/**
 * @brief ���ֵ�ǰλ��ͨ��ֱ���˶��ķ�ʽ�˶���Ŀ��λ��,����Ŀ��λ����ͨ����Ե�ǰλ�õ�ƫ�Ƹ���
 * @param rshd е�ۿ��������ľ��
 * @param target �����û�ƽ���ʾ��Ŀ��λ��
 * @param tool   ���߲���
 * @param isblock    isblock==true  ������������е���˶�ֱ������Ŀ��λ�û��߳��ֹ��Ϻ󷵻ء�
 *                   isblock==false ������������������أ��˶�ָ��ͳɹ��ͷ��أ��������غ��е�ۿ�ʼ�˶���
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_move_line_to(RSHD rshd, const Pos *target, const ToolInEndDesc *tool, bool isblock=true);

/**
 * @brief ���ֵ�ǰλ��ͨ���ؽ��˶��ķ�ʽ�˶���Ŀ��λ��
 * @param rshd е�ۿ��������ľ��
 * @param target �����û�ƽ���ʾ��Ŀ��λ��
 * @param tool ���߲���
 * @param isblock    isblock==true  ������������е���˶�ֱ������Ŀ��λ�û��߳��ֹ��Ϻ󷵻ء�
 *                   isblock==false ������������������أ��˶�ָ��ͳɹ��ͷ��أ��������غ��е�ۿ�ʼ�˶���
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_move_joint_to(RSHD rshd, const Pos *target, const ToolInEndDesc *tool, bool isblock=true);

/**
 * @brief ����ʾ������ϵ
 * @param rshd е�ۿ��������ľ��
 * @param user_coord ʾ������ϵ
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_teach_coord(RSHD rshd, const CoordCalibrate *teach_coord);


/**
 * @brief ��ʼʾ��
 * @param rshd е�ۿ��������ľ��
 * @param mode ʾ�̹ؽ�:JOINT1,JOINT2,JOINT3, JOINT4,JOINT5,JOINT6,   λ��ʾ��:MOV_X,MOV_Y,MOV_Z   ��̬ʾ��:ROT_X,ROT_Y,ROT_Z
 * @param dir  �˶�����   ������true  ������false
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_teach_move_start(RSHD rshd, teach_mode mode, bool dir);

/**
 * @brief ����ʾ��
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_teach_move_stop(RSHD rshd);

/**
 * @brief ����������ϵķ����߹켣�˶�����
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_clear_offline_track(RSHD rshd);

/**
 * @brief ���������ӷ����߹켣�˶�·��
 * @param rshd е�ۿ��������ľ��
 * @param waypoints ·������ (·�����С�ڵ���3000)
 * @param waypoint_count ·�������С
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_append_offline_track_waypoint(RSHD rshd, const JointParam waypoints[], int waypoint_count);

/**
 * @brief ���������ӷ����߹켣�˶�·���ļ�
 * @param rshd е�ۿ��������ľ��
 * @param filename ·���ļ�ȫ·��,·���ļ���ÿһ�а��������ؽڵĹؽڽ�(����),�ö��Ÿ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_append_offline_track_file(RSHD rshd, const char* filename);

/**
 * @brief ֪ͨ��������������߹켣�˶�
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_startup_offline_track(RSHD rshd);

/**
 * @brief ֪ͨ������ֹͣ�����߹켣�˶�
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_stop_offline_track(RSHD rshd);

/**
 * @brief ֪ͨ����������TCP2CANBUS͸��ģʽ
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_enter_tcp2canbus_mode(RSHD rshd);

/**
 * @brief ֪ͨ�������˳�TCP2CANBUS͸��ģʽ
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_leave_tcp2canbus_mode(RSHD rshd);

/**
 * @brief ͸���˶�·�㵽CANBUS
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_waypoint_to_canbus(RSHD rshd, double joint_radian[ARM_DOF]);

/**
 * @brief ͸���˶�·�㵽CANBUS
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_waypoints_to_canbus(RSHD rshd, double joint_radian[][ARM_DOF], int waypint_count);


/**
 * @brief ���⡡���������˺���Ϊ���⺯������֪�ؽڽ����Ӧλ�õ�λ�ú���̬��
 * @param rshd е�ۿ��������ľ��
 * @param joint_radian �����ؽڵĹؽڽǣ���λ(rad)
 * @param waypoint �����ؽڽ�,λ��,��̬
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_forward_kin(RSHD rshd, const double joint_radian[ARM_DOF], wayPoint_S *waypoint);

/**
 * @brief ��� �˺���Ϊ��е����⺯��������λ����Ϣ(x,y,z)�Ͷ�Ӧλ�õĲο���̬(w,x,y,z)�õ���Ӧλ�õĹؽڽ���Ϣ��
 * @param rshd е�ۿ��������ľ��
 * @param joint_radian �ο��ؽڽǣ�ͨ��Ϊ��ǰ��е��λ�ã���λ(rad)
 * @param pos Ŀ��·���λ�� ��λ:��
 * @param ori Ŀ��·��Ĳο���̬
 * @param waypoint Ŀ��·����Ϣ
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_inverse_kin(RSHD rshd, double joint_radian[ARM_DOF], const Pos *pos, const Ori *ori, wayPoint_S *waypoint);

/**
 * @brief ��� �˺���Ϊ��е����⺯��������λ����Ϣ(x,y,z)�Ͷ�Ӧλ�õĲο���̬(w,x,y,z)�õ���Ӧλ�õĹؽڽ���Ϣ��
 * @param rshd е�ۿ��������ľ��
 * @param pos Ŀ��·���λ�� ��λ:��
 * @param ori Ŀ��·��Ĳο���̬
 * @param ik_solutions Ŀ��·����Ϣ��������⣩
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_inverse_kin_closed_form(RSHD rshd, const Pos *pos, const Ori *ori, ik_solutions *solutions);

/**
 * @brief �û�����ϵת��������ϵ
 *        ����:  �����������Ļ��ڻ�����ϵ�µ�λ�ú���̬��ת�ɡ�����ĩ�˻����û�����ϵ�µ�λ�ú���̬��
 *
 *      ����չ1:  ���������Ŀ��Կ�����һ������Ĺ��ߣ������ߵ�λ��Ϊ(0,0,0)
 * ��������������  ��˵�����Ϊ(0,0,0)ʱ���൱�ڽ����������Ļ��ڻ�����ϵ�µ�λ�ú���̬��ת�ɡ����������Ļ����û�����ϵ�µ�λ�ú���̬��
 *
 * ����������չ2:  �û�����ϵҲ����ѡ��ɻ�����ϵ����������userCoord.coordType = BaseCoordinate
 *               ��˵��û�ƽ��Ϊ������ϵʱ���൱�ڽ����������Ļ��ڻ�����ϵ�µ�λ�ú���̬��ת�ɡ�����ĩ�˻��ڻ�����ϵ�µ�λ�ú���̬��
 *               ���ڻ�����ϵ�ӹ��ߡ�
 * @param rshd е�ۿ��������ľ��
 * @param pos_onbase ���ڻ�����ϵ�ķ���������λ����Ϣ��x,y,z��  ��λ(m)
 * @param ori_onbase ���ڻ�����ϵ����̬��Ϣ(w, x, y, z)
 * @param user_coord �û�����ϵ
 * @param tool_pos   ������Ϣ
 * @param pos_onuser �����û�����ϵ�Ĺ���ĩ��λ����Ϣ,�������
 * @param ori_onuser �����û�����ϵ�Ĺ���ĩ����̬��Ϣ,�������
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_base_to_user(RSHD rshd, const Pos *pos_onbase, const Ori *ori_onbase, const CoordCalibrate *user_coord, const ToolInEndDesc *tool_pos, Pos *pos_onuser, Ori *ori_onuser);

/**
 * @brief �û�����ϵת������ϵ
 * @param rshd е�ۿ��������ľ��
 * @param pos_onuser �����û�����ϵ�Ĺ���ĩ��λ����Ϣ
 * @param ori_onuser �����û�����ϵ�Ĺ���ĩ����̬��Ϣ
 * @param user_coord �û�����ϵ
 * @param tool_pos   ������Ϣ
 * @param pos_onbase ���ڻ�����ϵ�ķ���������λ����Ϣ
 * @param ori_onbase ���ڻ�����ϵ����̬��Ϣ
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_user_to_base(RSHD rshd, const Pos *pos_onuser, const Ori *ori_onuser, const CoordCalibrate *user_coord, const ToolInEndDesc *tool_pos, Pos *pos_onbase, Ori *ori_onbase);

/**
 * @brief ��������ϵת������õ�����ĩ�˵��λ�ú���̬
 * @param rshd е�ۿ��������ľ��
 * @param flange_center_pos_onbase ���ڻ�����ϵ�Ĺ���ĩ��λ����Ϣ
 * @param flange_center_ori_onbase ���ڻ�����ϵ�Ĺ���ĩ����̬��Ϣ
 * @param tool ������Ϣ
 * @param tool_end_pos_onbase ���ڻ�����ϵ�Ĺ���ĩ��λ����Ϣ
 * @param tool_end_ori_onbase ���ڻ�����ϵ�Ĺ���ĩ����̬��Ϣ
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_base_to_base_additional_tool(RSHD rshd, const Pos *flange_center_pos_onbase, const Ori *flange_center_ori_onbase, const ToolInEndDesc *tool, Pos *tool_end_pos_onbase, Ori *tool_end_ori_onbase);

/**
 * @brief ŷ����ת��Ԫ��
 * @param rshd е�ۿ��������ľ��
 * @param rpy ��̬��ŷ���Ǳ�ʾ����
 * @param ori ��̬����Ԫ�ر�ʾ����
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_rpy_to_quaternion(RSHD rshd, const Rpy *rpy,  Ori *ori);

/**
 * @brief ��Ԫ��תŷ����
 * @param rshd е�ۿ��������ľ��
 * @param ori  ��̬����Ԫ�ر�ʾ����
 * @param rpy  ��̬��ŷ���Ǳ�ʾ����
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_quaternion_to_rpy(RSHD rshd, const Ori *ori , Rpy *rpy);

/**
 * @brief ���ù��ߵ��˶�ѧ����
 * @param rshd е�ۿ��������ľ��
 * @param tool ���ߵ��˶�ѧ����
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_tool_end_param(RSHD rshd,const ToolInEndDesc *tool);

//end tool parameters
/**
 * @brief �����޹��ߵĶ���ѧ����
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_none_tool_dynamics_param(RSHD rshd);

/**
 * @brief ���ù��ߵĶ���ѧ����
 * @param rshd е�ۿ��������ľ��
 * @param tool ���ߵĶ���ѧ����
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_tool_dynamics_param(RSHD rshd, const ToolDynamicsParam *tool);

/**
 * @brief ��ȡ���ߵĶ���ѧ����
 * @param rshd е�ۿ��������ľ��
 * @param tool ���ߵĶ���ѧ����
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_tool_dynamics_param(RSHD rshd, ToolDynamicsParam *tool);

/**
 * @brief �����޹����˶�ѧ����
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_none_tool_kinematics_param(RSHD rshd);

/**
 * @brief ���ù��ߵ��˶�ѧ����
 * @param rshd е�ۿ��������ľ��
 * @param tool ���ߵ��˶�ѧ����
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_tool_kinematics_param(RSHD rshd, const ToolKinematicsParam *tool);

/**
 * @brief ��ȡ���ߵ��˶�ѧ����
 * @param rshd е�ۿ��������ľ��
 * @param tool ���ߵ��˶�ѧ����
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_tool_kinematics_param(RSHD rshd, ToolKinematicsParam *tool);


//robot control
/**
 * @brief �����е��
 * @param rshd е�ۿ��������ľ��
 * @param tool_dynamics ����ѧ����
 * @param colli_class ��ײ�ȼ�
 * @param read_pos �Ƿ������ȡλ��
 * @param static_colli_detect �Ƿ�������⾲̬��ײ
 * @param board_maxacc �ӿڰ�����������ٶ�
 * @param state ��е�����״̬
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_robot_startup(RSHD rshd, const ToolDynamicsParam *tool_dynamics, uint8 colli_class, bool read_pos, bool static_colli_detect, int board_maxacc, ROBOT_SERVICE_STATE *state);

/**
 * @brief �رջ�е��
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_robot_shutdown(RSHD rshd);

/**
 * @brief ֹͣ��е���˶�
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_move_stop(RSHD rshd);

/**
 * @brief ֹͣ��е���˶�
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_move_fast_stop(RSHD rshd);

/**
 * @brief ��ͣ��е���˶�
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_move_pause(RSHD rshd);

/**
 * @brief ��ͣ��ظ���е���˶�
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_move_continue(RSHD rshd);

/**
 * @brief ��е����ײ��ָ�
 * @param rshd е�ۿ��������ľ��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_collision_recover(RSHD rshd);

/**
 * @brief ��ȡ��е�۵�ǰ״̬
 * @param rshd е�ۿ��������ľ��
 * @param state ��е�۵�ǰ״̬
 *              ��е�۵�ǰֹͣ:RobotStatus.Stopped
 *              ��е�۵�ǰ����:RobotStatus.Running
 *              ��е�۵�ǰ��ͣ:RobotStatus.Paused
 *              ��е�۵�ǰ�ָ�:RobotStatus.Resumed
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_robot_state(RSHD rshd, RobotState *state);

//tool interface


//robot parameters
/**
 * @brief ���û�е�۷���������ģʽ
 * @param rshd е�ۿ��������ľ��
 * @param mode  ��е�۷���������ģʽ
 *              ��е�۷���ģʽ:RobotRunningMode.RobotModeSimulator
 *              ��е����ʵģʽ:RobotRunningMode.RobotModeReal
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_work_mode(RSHD rshd, RobotWorkMode  mode);

/**
 * @brief ��ȡ��е�۷�������ǰ����ģʽ
 * @param rshd е�ۿ��������ľ��
 * @param mode  ��е�۷���������ģʽ
 *              ��е�۷���ģʽ:RobotRunningMode.RobotModeSimulator
 *              ��е����ʵģʽ:RobotRunningMode.RobotModeReal
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_work_mode(RSHD rshd, RobotWorkMode *mode);

/**
 * @brief ��ȡ��������
 * @param rshd е�ۿ��������ľ��
 * @param gravity ��������
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_gravity_component(RSHD rshd, RobotGravityComponent *gravity);

/**
 * @brief ���û�е����ײ�ȼ�
 * @param rshd е�ۿ��������ľ��
 * @param grade ��ײ�ȼ� ��Χ��0��10��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_collision_class(RSHD rshd, int grade);

/**
 * @brief ��ȡ�豸��Ϣ
 * @param rshd е�ۿ��������ľ��
 * @param dev �豸��Ϣ
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_device_info(RSHD rshd, RobotDevInfo *dev);

/**
 * @brief ��ȡ��ǰ�Ƿ��Ѿ�������ʵ��е��
 * @param rshd е�ۿ��������ľ��
 * @param exist true������ false��������
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_is_have_real_robot(RSHD rshd , bool *exist);

/**
 * @brief ��ǰ��е���Ƿ�����������ģʽ
 * @param rshd е�ۿ��������ľ��
 * @param isonline true���� false������
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_is_online_mode(RSHD rshd, bool *isonline);

/**
 * @brief ��ǰ��е���Ƿ�������������ģʽ
 * @param rshd е�ۿ��������ľ��
 * @param ismaster true����ģʽ false����ģʽ
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_is_online_master_mode(RSHD rshd, bool *ismaster);

/**
 * @brief ��ȡ��е�۵�ǰ״̬��Ϣ
 * @param rshd е�ۿ��������ľ��
 * @param jointStatus ���������ؽ�״̬����������������ѹ���¶�
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_joint_status(RSHD rshd, JointStatus jointStatus[ARM_DOF]);

/**
 * @brief ��ȡ��е�۵�ǰλ����Ϣ
 * @param rshd е�ۿ��������ľ��
 * @param waypoint �ؽ�λ����Ϣ
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_current_waypoint(RSHD rshd, wayPoint_S *waypoint);

/**
 * @brief ��ȡ��е�������Ϣ
 * @param rshd е�ۿ��������ľ��
 * @param info ��е�������Ϣ
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_diagnosis_info(RSHD rshd, RobotDiagnosis *info);

/**
 * @brief ��ȡsocket����״̬
 * @param rshd е�ۿ��������ľ��
 * @param connected true�������� false��δ����
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_socket_status(RSHD rshd, bool *connected);

/**
 * @brief ��ȡ��е��ĩ���ٶ�
 * @param rshd е�ۿ��������ľ��
 * @param endspeed ĩ���ٶ� ��λ��m/s��
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_robot_end_speed(RSHD rshd, float *endspeed);

//IO interaface
/**
 * @brief ��ȡ�ӿڰ�ָ��IO���ϵ�������Ϣ
 * @param rshd е�ۿ��������ľ��
 * @param type IO����
 * @param config IO������Ϣ����
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_board_io_config(RSHD rshd, RobotIoType type, std::vector<RobotIoDesc> *config);

/**
 * @brief ���ݽӿڰ�IO���ͺ͵�ַ����IO״̬
 * @param rshd е�ۿ��������ľ��
 * @param type IO����
 * @param name IO����
 * @param val  IO״̬
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_board_io_status_by_name(RSHD rshd, RobotIoType type, const char *name, double val);

/**
 * @brief ���ݽӿڰ�IO���ͺ͵�ַ����IO״̬
 * @param rshd е�ۿ��������ľ��
 * @param type IO����
 * @param addr IO״̬
 * @param val  IO״̬
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_board_io_status_by_addr(RSHD rshd, RobotIoType type, int addr, double val);

/**
 * @brief ���ݽӿڰ�IO���ͺ͵�ַ��ȡIO״̬
 * @param rshd е�ۿ��������ľ��
 * @param type IO����
 * @param name IO����
 * @param val  IO״̬
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_board_io_status_by_name(RSHD rshd, RobotIoType type, const char *name, double *val);

/**
 * @brief ���ݽӿڰ�IO���ͺ͵�ַ��ȡIO״̬
 * @param rshd е�ۿ��������ľ��
 * @param type IO����
 * @param addr IO��ַ
 * @param val
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_board_io_status_by_addr(RSHD rshd, RobotIoType type, int addr, double *val);

//tool device interface
/**
 * @brief ���ù��߶˵�Դ��ѹ����
 * @param rshd е�ۿ��������ľ��
 * @param type ower_type:��Դ����
 *              0:.OUT_0V
 *              1:.OUT_12V
 *              2:.OUT_24V
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_tool_power_type(RSHD rshd, ToolPowerType type);

/**
 * @brief ��ȡ���߶˵�Դ��ѹ����
 * @param rshd е�ۿ��������ľ��
 * @param type ower_type:��Դ����
 *              0:.OUT_0V
 *              1:.OUT_12V
 *              2:.OUT_24V
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_tool_power_type(RSHD rshd, ToolPowerType *type);

/**
 * @brief ���ù��߶�������IO������
 * @param rshd е�ۿ��������ľ��
 * @param addr ��ַ
 * @param type ����  0:���� 1:���
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_tool_io_type(RSHD rshd, ToolDigitalIOAddr addr, ToolIOType type);

/**
 * @brief ��ȡ���߶˵�ѹ��ֵ
 * @param rshd е�ۿ��������ľ��
 * @param voltage ��ѹ��ֵ����λ�����أ�
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_tool_power_voltage(RSHD rshd, double *voltage);

/**
 * @brief ��ȡ���߶�IO״̬
 * @param rshd е�ۿ��������ľ��
 * @param name IO����
 * @param val ���߶�IO״̬
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_get_tool_io_status(RSHD rshd, const char *name, double *val);

/**
 * @brief ���ù��߶�IO״̬
 * @param rshd е�ۿ��������ľ��
 * @param name IO����
 * @param status ���߶�IO״̬
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_tool_do_status(RSHD rshd, const char *name, IO_STATUS status);

/**
 * @brief ��ͣ����
 * @param rshd
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_client_status_pause(RSHD rshd);

/**
 * @brief ��������
 * @param rshd
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_set_client_status_continue(RSHD rshd);

/**
 * @brief ����ֹͣ
 * @param rshd
 * @return
 */
int rs_set_client_status_stop(RSHD rshd);

/**
 * @brief �������
 * @param rshd
 * @return
 */
int rs_set_client_status_run(RSHD rshd);

/**
 * @brief �·���е�ۿ���ָ��
 * @param rshd
 * @param cmd
 * @return
 */
int rs_robot_control(RSHD rshd, RobotControlCommand cmd);

/**
 * @brief ���û�е�۱�ʶ����
 * @param param
 * @return
 */
int rs_set_robot_recognition_param(RSHD rshd, const RobotRecongnitionParam *param);


/**
 * @brief ��ȡ��е�۱�ʶ����
 * @param type
 * @param param
 * @return
 */
int rs_get_robot_recognition_param(RSHD rshd, int type, RobotRecongnitionParam *param);

//callback
/**
 * @brief ע�����ڻ�ȡʵʱ·��Ļص�����
 * @param rshd е�ۿ��������ľ��
 * @param ptr ��ȡʵʱ·����Ϣ�ĺ���ָ��
 * @param arg �������ϵͳ�����κδ����ֻ�ǽ����˻��棬���ص���������ʱ�ò�����ͨ���ص������Ĳ�������
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_setcallback_realtime_roadpoint(RSHD rshd, const RealTimeRoadPointCallback ptr, void  *arg);

/**
 * @brief ע�����ڻ�ȡ�ؽ�״̬�Ļص�����
 * @param rshd е�ۿ��������ľ��
 * @param ptr ��ȡʵʱ�ؽ�״̬��Ϣ�ĺ���ָ��
 * @param arg �������ϵͳ�����κδ����ֻ�ǽ����˻��棬���ص���������ʱ�ò�����ͨ���ص������Ĳ�������
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_setcallback_realtime_joint_status(RSHD rshd, const RealTimeJointStatusCallback ptr, void  *arg);

/**
 * @brief ע�����ڻ�ȡʵʱĩ���ٶȵĻص�����
 * @param rshd е�ۿ��������ľ��
 * @param ptr ��ȡʵʱĩ���ٶȵĺ���ָ��
 * @param arg ������ϵͳ�����κδ����ֻ�ǽ����˻��棬���ص���������ʱ�ò�����ͨ���ص������Ĳ�������
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_setcallback_realtime_end_speed(RSHD rshd, const RealTimeEndSpeedCallback ptr, void  *arg);

/**
 * @brief ע�����ڻ�ȡ��е���¼���Ϣ�Ļص�����
 * @param rshd е�ۿ��������ľ��
 * @param ptr ��ȡ��е���¼���Ϣ�ĺ���ָ��
 * @param arg ������ϵͳ�����κδ����ֻ�ǽ����˻��棬���ص���������ʱ�ò�����ͨ���ص������Ĳ�������
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_setcallback_robot_event(RSHD rshd, const RobotEventCallback ptr, void  *arg);


//enable push information
/**
 * @brief �����Ƿ�����ʵʱ·����Ϣ����
 * @param rshd е�ۿ��������ľ��
 * @param enable true��ʾ���� false��ʾ������
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_enable_push_realtime_roadpoint(RSHD rshd, bool enable);

/**
 * @brief �����Ƿ�����ʵʱ�ؽ�״̬����
 * @param rshd е�ۿ��������ľ��
 * @param enable true��ʾ���� false��ʾ������
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_enable_push_realtime_joint_status(RSHD rshd, bool enable);

/**
 * @brief �����Ƿ�����ʵʱĩ���ٶ�����
 * @param rshd е�ۿ��������ľ��
 * @param enable true��ʾ���� false��ʾ������
 * @return RS_SUCC �ɹ� ����ʧ��
 */
int rs_enable_push_realtime_end_speed(RSHD rshd, bool enable);


#ifdef __cplusplus
}
#endif


#endif // RSDEF_H
