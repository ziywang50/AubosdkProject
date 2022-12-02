#pragma once

#include <string>
#include <armadillo>
#include "rsdef.h"
#include "yolo.h"

using namespace std;

//��ӡ·����Ϣ
void printRoadPoint(const aubo_robot_namespace::wayPoint_S  *wayPoint);

void callback_RealTimeRoadPoint(const aubo_robot_namespace::wayPoint_S  *wayPoint, void *arg);

//��½��е��
bool example_login(RSHD &rshd, const char * addr, int port);

//�˳���½
bool example_logout(RSHD rshd);

//�����е��(����������ʵ��е�ۣ�
bool example_robotStartup(RSHD rshd);

//�رջ�е�ۣ�����������ʵ��е�ۣ�
bool example_robotShutdown(RSHD rshd);

//��е���ᶯ����
bool example_moveJ(RSHD rshd);

//��е�۱��ֵ�ǰ��ֱ̬���˶�����
bool example_moveL(RSHD rshd, aubo_robot_namespace::Pos pos);

//��е�۹켣�˶�����
void example_moveP(RSHD rshd);

//��е����������
void example_ik_fk(RSHD rshd);

//��е�ۿ��ƹ�IO����(����������ʵ��е�ۣ�
void example_boardIO(RSHD rshd);

//��е�۹��߶�IO����(����������ʵ��е�ۣ�
void example_ToolIO(RSHD rshd);

//ʵʱ·����Ϣ�ص���������
bool example_callbackRobotRoadPoint(RSHD rshd);

//�����Ϣ
void example_get_diagnosis(RSHD rshd);

aubo_robot_namespace::Pos convertcoordinates(RSHD rshd, std::vector<Output> result);