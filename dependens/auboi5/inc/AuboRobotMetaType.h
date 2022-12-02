#ifndef AUBOROBOTMETATYPE_H
#define AUBOROBOTMETATYPE_H

#include <iostream>
#include <stdint.h>

/* General types */
typedef  uint8_t     boolean;
typedef  int8_t      int8;
typedef  int16_t     int16;
typedef  int32_t     int32;
typedef  uint8_t     uint8;
typedef  uint16_t    uint16;
typedef  uint32_t    uint32;
typedef  int64_t     int64;
typedef  uint64_t    uint64;
typedef  float       float32;
typedef  double      float64;

#ifdef __cplusplus
extern "C" {
#endif


/** �����ռ� **/
namespace  aubo_robot_namespace
{

enum {ARM_DOF = 6};          /** ��е�۹ؽ��� **/

#pragma pack(1)

/** ��е�۹ؽڰ汾��Ϣ��**/
typedef struct 
{
    //Ӳ���汾��Ϣ
    char hw_version[8];
    //�̼��汾��Ϣ
    char sw_version[16];
}JointVersion;

typedef struct 
{
    char productID[16];
}JointProductID;

/**
 *���ýṹ�������豸��Ϣ
 **/
typedef struct 
{
    //�豸�ͺš�оƬ�ͺţ���λ����վ��0x01  �ӿڰ�0x02
    uint8 type;
    //�豸�汾�ţ�V1.0
    char revision[16];
    //����ID��"OUR "��ASCII��0x4F 55 52 00
    char manu_id[16];
    //��е������
    char joint_type[16];
    //��е�۹ؽڼ����߶���Ϣ
    JointVersion joint_ver[8];
    //�豸�����ַ�����0x00����
    char desc[64];
    //�ؽ�ID��Ϣ
    JointProductID jointProductID[8];

    //���豸�汾�� - �ַ�����ʾ���硰V1.0.0
    char slave_version[16];
    //IO��չ��汾�� -�ַ�����־���硰V1.0.0
    char extio_version[16];

}RobotDevInfo;

/**
  * �ýṹ��������е�۵Ĺؽ�״̬
  *
  */
typedef struct 
{
    int    jointCurrentI;       /**< Current of driver   �ؽڵ���*/
    int    jointSpeedMoto;      /**< Speed of driver �����ؽ��ٶ�*/
    float  jointPosJ;           /**< Current position in radian ���ؽڽ�*/
    float  jointCurVol;         /**< Rated voltage of motor. Unit: mV ���ؽڵ�ѹ*/
    float  jointCurTemp;        /**< Current temprature of joint ����������ǰ�¶�*/
    int    jointTagCurrentI;    /**< Target current of motor ���������� �����Ŀ�����*/
    float  jointTagSpeedMoto;   /**< Target speed of motor �������������� ���Ŀ���ٶ�*/
    float  jointTagPosJ;        /**< Target position of joint in radian Ŀ��ؽڽǡ�*/
    uint16 jointErrorNum;       /**< Joint error of joint num�������������ؽڴ����� */
}JointStatus;


/**
 * event define   ������е���¼�����
 *
 * ��е�۵ĺܶ���Ϣ�����ϣ�֪ͨ����ͨ���¼�֪ͨ���ͻ��ģ�������ʹ��SDKʱ��
 * ���ע������¼��Ļص�������
 */
typedef enum{
    RobotEvent_armCanbusError,           //��е��CAN���ߴ���
    RobotEvent_remoteHalt,               //Զ�̹ػ���������TODO
    RobotEvent_remoteEmergencyStop,      //��е��Զ�̼�ͣ
    RobotEvent_jointError,               //�ؽڴ���

    RobotEvent_forceControl,             //������
    RobotEvent_exitForceControl,         //�˳�������

    RobotEvent_softEmergency,            //���ͣ
    RobotEvent_exitSoftEmergency,        //�˳����ͣ

    RobotEvent_collision,                //��ײ
    RobotEvent_collisionStatusChanged,   //��ײ״̬�ı�
    RobotEvent_tcpParametersSucc,        //���߶���ѧ�������óɹ�
    RobotEvent_powerChanged,             //��е�۵�Դ����״̬�ı�
    RobotEvent_ArmPowerOff,              //��е�۵�Դ�ر�
    RobotEvent_mountingPoseChanged,      //��װλ�÷����ı�
    RobotEvent_encoderError,             //����������

    RobotEvent_encoderLinesError,        //������������һ��
    RobotEvent_singularityOverspeed,     //����㳬��
    RobotEvent_currentAlarm,             //��е�۵����쳣
    RobotEvent_toolioError,             //��е�۹��߶˴���
    RobotEvent_robotStartupPhase,       //��е������׶�
    RobotEvent_robotStartupDoneResult,  //��е�������ɽ��
    RobotEvent_robotShutdownDone,       //��е�۹ػ����
    RobotEvent_atTrackTargetPos,        //��е�۹켣�˶���λ�ź�֪ͨ

    RobotSetPowerOnDone,                //���õ�Դ״̬���
    RobotReleaseBrakeDone,              //��е��ɲ���ͷ����
    RobotEvent_robotControllerStateChaned,  //��е�ۿ���״̬�ı�
    RobotEvent_robotControllerError,        //��е�ۿ��ƴ���----һ�����㷨�滮��������ʱ����
    RobotEvent_socketDisconnected,          //socket�Ͽ�����

    RobotEvent_robotControlException,
    RobotEvent_trackPlayInterrupte,

    RobotEvent_staticCollisionStatusChanged,
    RobotEvent_MountingPoseWarning,
    RobotEvent_MacDataInterruptWarning,
    RobotEvent_ToolIoError,
    RobotEvent_InterfacBoardSafeIoEvent,

    RobotEvent_RobotHandShakeSucc,
    RobotEvent_RobotHandShakeFailed,

    RobotEvent_RobotErrorInfoNotify,


    RobotEvent_exceptEvent = 100,

    //unknown event
    robot_event_unknown,

    //user event
    RobotEvent_User = 1000,                            // first user event id
    RobotEvent_MaxUser = 65535                         // last user event id

}RobotEventType;


typedef enum
{
    RobotControllerErr_MotionCfgErr,    //only this is recoverable.
    RobotControllerErr_OverspeedProtect,
    RobotControllerErr_IkFailure,
    RobotControllerErr_OnlineTrajErr,
    RobotControllerErr_OfflineTrajErr,
    RobotControllerErr_StatusException,
}RobotControllerErrorCode;


typedef enum {
    RUN_TO_READY_POSITION,
    RUN_PROJECT,
    PAUSE_PROJECT,
    CONTINUE_PROJECT,
    SLOWLY_STOP_PROJECT,
    LOAD_PROJECT,
    ENTER_SAFEGUARD_MODE_BY_DI_EXTERNAL_SAFEGUARD_STOP,
    RELEASE_SAFEGUARD_MODE_IN_AUTOMATIC_MODE,
    RELEASE_SAFEGUARD_MODE_IN_MANUAL_MODE,
    MANUALLY_RELEASE_SAFEGUARD_MODE_PROMPT,
    ENTER_SAFEGUARD_MODE_BY_TRI_STATE_SWITCH,
    RELEASE_SAFEGUARD_MODE_BY_TRI_STATE_SWITCH,
    ENTER_REDUCE_MODE,
    RELEASE_REDUCE_MODE,
}InterfaceBoardSafeIoEventCode;


typedef enum {
    RobotToolNoError      = 0, //�޴���
    RobotToolOverVoltage  = 1, //��ѹ
    RobotToolUnderVoltage = 2, //Ƿѹ
    RobotToolOVerTemp     = 3, //����
    RobotToolCanBusError  = 4  //CAN���ߴ���
}RobotToolErrorCode;


/** �¼����� **/
typedef struct{
    RobotEventType  eventType;       //�¼����ͺ�
    int             eventCode;       //
    std::string     eventContent;    //�¼�����
}RobotEventInfo;




/****��е����������x y z *****/
typedef struct 
{
    float x;
    float y;
    float z;
}RobotGravityComponent;

//��չ�豸�����Ϣ
typedef struct 
{
    //��е��״̬
    //1- �ӿڰ��ϵ�
    //2- �Լ����
    //3- MAC����
    //4- �ȴ���е���ϵ�
    //5- ��ʼ����е��
    //6- ��е����������
    //7- �ػ�
    uint8_t robot_state;
    //��е�۵�ǰ����״̬
    //1- ����
    //2- ������
    //3- �ٶȻ�
    //4- λ�û�
    uint8_t robot_status;
    //Ŀ��λ��
    float target_pos[6];
    //�����ٶ�rad/s
    float theoretical_speed[6];
    //���ۼ��ٶ�rad/ss
    float theoretical_acc[6];
    //���۵��� /10 A
    int32_t theoretical_current[6];
}RobotExtDiagnosis;

typedef struct 
{
    uint16  robotReducedConfigJointSpeed[6];    //�������� �ؽ��ٶ�����
    uint32  robotReducedConfigTcpSpeed;         //�������� TCP�ٶ�����
    uint32  robotReducedConfigTcpForce;         //�������� TCP�����ݶ�Ϊ��ײ�ȼ���
    uint32  robotReducedConfigMomentum;         //�������� ����
    uint32  robotReducedConfigPower;            //�������� ����
    uint8   robotSafeguradResetConfig;          //������ ������
    uint8   robotOperationalModeConfig;         //����ģʽ����
}RobotSafetyConfig;


typedef struct 
{
    uint8 orpePause;               //��λ����ͣ״̬
    uint8 orpeStop;                //��λ��ֹͣ״̬
    uint8 orpeError[16];           //��λ������
    uint8 systemEmergencyStop;     //���ϵͳ���ֹͣ����ź�
    uint8 reducedModeError;        //�����������
    uint8 safetyguardResetSucc;    //�������óɹ�
}OrpeSafetyStatus;


typedef struct 
{
    uint8  OriginPoseState;
    float   OriginPose[6];
}OriginPose;

/*****����IO����������********/

typedef struct 
{
    //����IO����
    uint8 ioData;
    //ģ��IO����
    float aiData[2];
    //ϵͳ��ѹ
    float systemVoltage;
    //ϵͳ�¶�
    float systemTemperature;
    //����״̬
    uint8 errorStatus;
}RobotToolStatus;

typedef struct
{
    //���߶�����
    //[0]:���߶˵�ѹ����
    //[1]:�ɣ�����
    uint8 config[4];
}RobotToolConfig;


typedef enum{
    RobotModeSimulator, //��е�۷���ģʽ
    RobotModeReal       //��е����ʵģʽ
}RobotWorkMode;

#pragma pack()

/****��е�������Ϣ****/
typedef struct
{
	//CANͨ��״̬:0x01~0x80���ؽ�CANͨ�Ŵ���ÿ���ؽ�ռ��1bit��
	//0x00���޴��� 0xff��CAN���ߴ��ڴ���
	uint8 armCanbusStatus;
	//��е��48V��Դ��ǰ����
	float armPowerCurrent;
	//��е��48V��Դ��ǰ��ѹ
	float armPowerVoltage;
	//��е��48V��Դ״̬�������أ�
	bool  armPowerStatus;
	//�������¶�
	char  contorllerTemp;
	//������ʪ��
	uint8 contorllerHumidity;
	//Զ�̹ػ��ź�
	bool  remoteHalt;
	//��е�����ͣ
	bool  softEmergency;
	//Զ�̼�ͣ�ź�
	bool  remoteEmergency;
	//��ײ���λ
	bool  robotCollision;
	//��е�۽�������ģʽ��־λ
	bool  forceControlMode;
	//ɲ��״̬
	bool brakeStuats;
	//ĩ���ٶ�
	float robotEndSpeed;
	//�����ٶ�
	int robotMaxAcc;
	//��λ�����״̬λ
	bool orpeStatus;
	//λ�˶�ȡʹ��λ
	bool enableReadPose;
	//��װλ��״̬
	bool robotMountingPoseChanged;
	//�ű���������״̬
	bool encoderErrorStatus;
	//��ֹ��ײ��⿪��
	bool staticCollisionDetect;
	//�ؽ���ײ��� ÿ���ؽ�ռ��1bit 0-����ײ 1-������ײ
	uint8 jointCollisionDetect;
	//����������һ�´��� 0-�޴��� 1-�д���
	bool encoderLinesError;
	//joint error status
	bool jointErrorStatus;
	//��е���������پ���
	bool singularityOverSpeedAlarm;
	//��е�۵������󾯸�
	bool robotCurrentAlarm;
	//tool error
	uint8 toolIoError;
	//��е�۰�װλ�ô�λ��ֻ������ģʽ�������ã�
	bool robotMountingPoseWarning;
	//mac����������
	uint16 macTargetPosBufferSize;
	//mac��������Ч���ݳ���
	uint16 macTargetPosDataSize;
	//mac�����ж�
	uint8  macDataInterruptWarning;

}RobotDiagnosis;


/**
 * ����IO������
 **/
typedef enum
{
    RobotBoardControllerDI,    //�ӿڰ������DI(����������)������ֻ��(һ��ϵͳ�ڲ�ʹ��)
    RobotBoardControllerDO,    //�ӿڰ������DO(���������)     ֻ��(һ��ϵͳ�ڲ�ʹ��)
    RobotBoardControllerAI,    //�ӿڰ������AI(ģ��������)��   ֻ��(һ��ϵͳ�ڲ�ʹ��)
    RobotBoardControllerAO,    //�ӿڰ������AO(ģ�������)������ֻ��(һ��ϵͳ�ڲ�ʹ��)

    RobotBoardUserDI,          //�ӿڰ��û�DI(����������)�����ɶ���д
    RobotBoardUserDO,          //�ӿڰ��û�DO(���������)   �ɶ���д
    RobotBoardUserAI,          //�ӿڰ��û�AI(ģ��������)   �ɶ���д
    RobotBoardUserAO,          //�ӿڰ��û�AO(ģ�������)   �ɶ���д

    RobotToolDI,               //���߶�DI
    RobotToolDO,               //���߶�DO
    RobotToolAI,               //���߶�AI
    RobotToolAO,               //���߶�AO

}RobotIoType;


typedef enum
{
    RobotToolIoTypeDI=RobotToolDI,      //���߶�DI
    RobotToolIoTypeDO=RobotToolDO       //���߶�DO
}RobotToolIoType;


/**
  * �ۺ�����һ��IO
  **/
typedef struct 
{
    char        ioId[32];      //IO-ID Ŀǰδʹ��
    RobotIoType ioType;        //IO����
    char        ioName[32];    //IO����
    int         ioAddr;        //IO��ַ
    double      ioValue;       //IO״̬
}RobotIoDesc;

/**
  * ���ߵĵ�Դ����
  **/
typedef enum
{
    OUT_0V  = 0,
    OUT_12V = 1,
    OUT_24V = 2
}ToolPowerType;

typedef  enum              //�ɣ�״̬
{
    IO_STATUS_INVALID = 0, //��Ч
    IO_STATUS_VALID        //��Ч
}IO_STATUS;


typedef enum
{
    TOOL_DIGITAL_IO_0 = 0,
    TOOL_DIGITAL_IO_1 = 1,
    TOOL_DIGITAL_IO_2 = 2,
    TOOL_DIGITAL_IO_3 = 3

}ToolDigitalIOAddr;


typedef enum          //�ɣ�����
{
    IO_IN = 0,        //����
    IO_OUT            //���
}ToolIOType;

//�ӿڰ�����������
typedef struct
{
    uint8 addr ;
    uint8 value;
    uint8 type;
}RobotDiagnosisIODesc;

//�ӿڰ�ģ��������
typedef struct
{
    uint8  addr ;
    float  value;
    uint8 type;
}RobotAnalogIODesc;

typedef enum {
    RobotMoveStop     = 0,
    RobotMovePause    = 1,
    RobotMoveContinue = 2,
}RobotMoveControlCommand;


enum RobotControlCommand{
    RobotRelease        = 0,    //�ͷ�ɲ��
    RobotBrake          = 1,    //ɲ��
    OverspeedWarning    = 2,    //�϶�ʾ���ٶȹ��챨��
    OverspeedRecover    = 3,    //����϶����ٱ���
    DisableForceControl = 4,    //ʧ������
    EnableForceControl  = 5,    //ʹ������
    OrpeOpen            = 6,    //����λ�����
    OrpeClose           = 7,    //�ر���λ�����
    EnableReadPose      = 8,    //�򿪶�ȡλ��
    DisableReadPose     = 9,    //�رն�ȡλ��
    MountingPoseChanged    = 10,//��װλ���Ѹı�
    MountingPoseUnChanged  = 11,//��װλ��δ�ı�
    EnableStaticCollisionDetect  = 12,   //�򿪾�ֹ��ײ���
    DisableStaticCollisionDetect = 13,   //�رվ�ֹ��ײ���
    ClearSingularityOverSpeedAlarm = 14, //�����е���������پ���
    ClearRobotCurrentAlarm = 15          //�����е�۵������󾯸�
};


enum Robot_Dyn_identify_traj
{
    Dyn_identify_traj_none = 0,
    Dyn_identify_traj_robot, //submode: 0/1 <-> internal/hybrid
    Dyn_identify_traj_tool,  //submode: 0/1 <-> tool only/tool+friction
    Dyn_identify_traj_tool_abort
};



/**
 * @brief ��е�۳�ʼ���׶�
 */
enum ROBOT_INIT_PHASE{
    ROBOT_INIT_PHASE_READY=0,
    ROBOT_INIT_PHASE_HANDSHAKE,
    ROBOT_INIT_PHASE_SET_POWER,
    ROBOT_INIT_PHASE_SET_BRAKE,
    ROBOT_INIT_PHASE_SET_COLLSION_CLASS,
    ROBOT_INIT_PHASE_SET_OTHER_CMD,
    ROBOT_INIT_PHASE_WORKING
};

/**
 * @brief ��е��������״̬
 */
enum ROBOT_SERVICE_STATE{
    ROBOT_SERVICE_READY=0,
    ROBOT_SERVICE_STARTING,
    ROBOT_SERVICE_WORKING,
    ROBOT_SERVICE_CLOSING,
    ROBOT_SERVICE_CLOSED,
    ROBOT_SETVICE_FAULT_POWER,
    ROBOT_SETVICE_FAULT_BRAKE,
    ROBOT_SETVICE_FAULT_NO_ROBOT
};


/** ��е��״̬ö�١�**/
enum RobotState
{
    RobotStopped = 0,
    RobotRunning,
    RobotPaused,
    RobotResumed
};

/** �˶�ģʽö�١�**/
enum move_mode
{
    NO_MOVEMODE = 0,
    MODEJ,
    MODEL,
    MODEP
};

/** �˶��켣ö�١�**/
enum move_track
{
    NO_TRACK = 0,

    //for moveJ and moveL
    TRACKING,

    //cartesian motion for movep
    ARC_CIR,
    CARTESIAN_MOVEP,
    CARTESIAN_CUBICSPLINE,
    CARTESIAN_UBSPLINEINTP,

    //joint motion  for movep
    JIONT_CUBICSPLINE,
    JOINT_UBSPLINEINTP,
};

typedef struct{
	uint8 type;   //��е�۱�ʶ��������
	uint8 length; //�������ݳ���
	uint8 data[256];//����ʵ������
}RobotRecongnitionParam;


/** ����ϵö�١�**/
enum coordinate_refer
{
    BaseCoordinate = 0,
    EndCoordinate,
    WorldCoordinate
};


/** ʾ��ģʽö�١�**/
enum teach_mode
{
    NO_TEACH = 0,
    JOINT1,
    JOINT2,
    JOINT3,
    JOINT4,
    JOINT5,
    JOINT6,
    MOV_X,
    MOV_Y,
    MOV_Z,
    ROT_X,
    ROT_Y,
    ROT_Z
};


/** ·��λ����Ϣ�ı�ʾ������**/
struct Pos
{
    double x;
    double y;
    double z;
};

/** ·��λ����Ϣ�ı�ʾ������**/
union cartesianPos_U
{
    Pos position;
    double positionVector[3];
};


/** ��̬����Ԫ�ر�ʾ������**/
struct Ori
{
    double w;
    double x;
    double y;
    double z;
};

/** ��̬��ŷ���Ǳ�ʾ������**/
struct Rpy
{
    double rx;
    double ry;
    double rz;
};


/** ������е�۵�·����Ϣ��**/
typedef struct
{
    cartesianPos_U cartPos;     //��е�۵�λ����Ϣ��X,Y,Z

    Ori orientation;            //��е����̬��Ϣ����ͨ����Ԫ�ر�ʾ,����ͨ�����ߺ���ʵ����ŷ���ǻ�ת

    double jointpos[ARM_DOF];   //��е�۹ؽڽ���Ϣ
}wayPoint_S;


/**
  * �����ؽڵ��ٶȺͼ��ٶ�
  */
typedef struct
{
    double jointPara[ARM_DOF];
}JointVelcAccParam;


typedef struct
{
    double jointPos[ARM_DOF];
}JointParam;


/**
 *  �����˶������е�ƫ������
 */
typedef struct
{
    bool  ena;                       //�Ƿ�ʹ��ƫ��

    float relativePosition[3];       //ƫ���� x,y,z

	Ori   relativeOri;

}MoveRelative;




/** �ýṹ���������ߵĲ���
  * ���߱궨��������ĩ������ϵ��λ��Ҳ����̬��
  */
typedef struct
{
    Pos        toolInEndPosition;     //�������ĩ������ϵ��λ��

    Ori        toolInEndOrientation;  //�������ĩ������ϵ����̬
}ToolInEndDesc;


typedef ToolInEndDesc ToolKinematicsParam;



enum ToolKinematicsOriCalibrateMathod{
    ToolKinematicsOriCalibrateMathod_Invalid = -1,

    ToolKinematicsOriCalibrateMathod_xOxy,       // ԭ�㡢x�������ᡢx��y��ƽ��ĵ�һ����������һ��
    ToolKinematicsOriCalibrateMathod_yOyz,       // ԭ�㡢y�������ᡢy��z��ƽ��ĵ�һ����������һ��
    ToolKinematicsOriCalibrateMathod_zOzx,       // ԭ�㡢z�������ᡢz��x��ƽ��ĵ�һ����������һ��

    ToolKinematicsOriCalibrateMathod_TxRBz_TxyPBzAndTyABnz,  //����x��ƽ�з����ڻ�����ϵz��; ����xOyƽ��ƽ���ڻ�����ϵz�ᡢ����y���������ϵ��z��н�Ϊ���
    ToolKinematicsOriCalibrateMathod_TyRBz_TyzPBzAndTzABnz,  //����y��ƽ�з����ڻ�����ϵz��; ����yOzƽ��ƽ���ڻ�����ϵz�ᡢ����z���������ϵ��z��н�Ϊ���
    ToolKinematicsOriCalibrateMathod_TzRBz_TzxPBzAndTxABnz,  //����z��ƽ�з����ڻ�����ϵz��; ����zOxƽ��ƽ���ڻ�����ϵz�ᡢ����x���������ϵ��z��н�Ϊ���

    ToolKinematicsOriCalibrateMathodCount
};


/** �ýṹ���������߹�����**/
typedef struct
{
    double xx;
    double xy;
    double xz;
    double yy;
    double yz;
    double zz;
}ToolInertia;



/**
 * �ýṹ���������ߵġ�����ѧ����
 *
 * ע�⣺�ڸ�����е�۵Ĺ���ʱ�����ߵĶ���ѧ�������˶�ѧ��������Ҫһ�����õġ�
 **/
typedef struct
{
    double positionX;    //�������ĵ�X����

    double positionY;    //�������ĵ�Y����

    double positionZ;    //�������ĵ�Z����

    double payload;      //��������

    ToolInertia toolInertia;  //���߹���

}ToolDynamicsParam;



/** ����ϵ�궨����ö�� **/
enum CoordCalibrateMathod
{
    Origin_AnyPointOnPositiveXAxis_AnyPointOnPositiveYAxis,            // ԭ�㡢x�������ᡢy��������
    Origin_AnyPointOnPositiveYAxis_AnyPointOnPositiveZAxis,            // ԭ�㡢y�������ᡢz��������
    Origin_AnyPointOnPositiveZAxis_AnyPointOnPositiveXAxis,            // ԭ�㡢z�������ᡢx��������
    Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane,  // ԭ�㡢x�������ᡢx��y��ƽ��ĵ�һ����������һ��
    Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOZPlane,  // ԭ�㡢x�������ᡢx��z��ƽ��ĵ�һ����������һ��
    Origin_AnyPointOnPositiveYAxis_AnyPointOnFirstQuadrantOfYOZPlane,  // ԭ�㡢y�������ᡢy��z��ƽ��ĵ�һ����������һ��
    Origin_AnyPointOnPositiveYAxis_AnyPointOnFirstQuadrantOfYOXPlane,  // ԭ�㡢y�������ᡢy��x��ƽ��ĵ�һ����������һ��
    Origin_AnyPointOnPositiveZAxis_AnyPointOnFirstQuadrantOfZOXPlane,  // ԭ�㡢z�������ᡢz��x��ƽ��ĵ�һ����������һ��
    Origin_AnyPointOnPositiveZAxis_AnyPointOnFirstQuadrantOfZOYPlane,  // ԭ�㡢z�������ᡢz��y��ƽ��ĵ�һ����������һ��

    CoordTypeCount
};


/**
 * �ýṹ������һ������ϵ��ϵͳ���ݸýṹ���ܹ�Ψһ��ȷ��һ������ϵ��
 *
 * ����ϵ�֣������ͣ�������ϵ(BaseCoordinate)��ĩ������ϵ(EndCoordinate)���û�����ϵ(WorldCoordinate);
 *
 *����������ϵ�ǡ����ݻ�е�۵�������������ϵ
 *��ĩ������ϵ�ǡ����ݷ��������Ľ���������ϵ
 *���û�����ϵ�ǡ��û������Լ�����������ʵ����Ҫ�õ�������ϵ,ϵͳ�����û��ṩ�ģ�����ͱ궨����ȷ���û�����ϵ��X��,Y��,Z�ᡣ
 * �������������� ��ʵ��Ӧ���б궨����ϵʱ���õ����ߣ�ϵͳΪ��׼ȷ�ĵõ�����ϵ�궨�ģ����㣬�����û���Ҫ�ṩ������Ϣ��
 * �������������� ���û��ʹ�ù��߿��Խ���������(toolDesc)����Ϊ����
 *
 * ʹ��˵����
 * ����1:��coordType==BaseCoordinate����coordType==EndCoordinateʱ������3������(methods,wayPointArray,toolDesc)ϵͳ���������
 * ��������Ϊϵͳ�����������coordType�Ѿ�����ȷ��������ϵ�ˡ�
 *
 * ����2:�������ϵ�궨��ʱ��û��ʹ�ù��ߣ����������е�λ�ú���̬��ϢӦ����Ϊ����
 */
typedef struct
{
    coordinate_refer    coordType;       //����ϵ���ͣ���coordType==BaseCoordinate����coordType==EndCoordinate�ǣ�����3��������������

    CoordCalibrateMathod methods;        // ����ϵ�궨����

    JointParam       wayPointArray[3];   //���ڱ궨����ϵ�ģ����㣨�ؽڽǣ�����Ӧ�ڻ�е�۷��������ĵ���ڻ�����ϵ

    ToolInEndDesc    toolDesc;           //�궨��ʱ��ʹ�õĹ�������

}CoordCalibrateByJointAngleAndTool;



/**
 *  �ӿڵ��óɹ��ķ���ֵ
 *  �ӿڷ���ֵ����������ֵ,�ɹ�����InterfaceCallSuccCode��ʧ�ܷ��ش���ţ�
 *
 * ��ע�������Ҳ����������
 */
enum
{
    InterfaceCallSuccCode = 0,          //�ӿڵ��óɹ��ķ���ֵ
};

typedef enum
{
    ErrnoSucc = aubo_robot_namespace::InterfaceCallSuccCode,  /** �ɹ���**/

    ErrCode_Base = 10000,
    ErrCode_Failed,       /** ͨ��ʧ�ܡ�**/
    ErrCode_ParamError,   /** ��������**/
    ErrCode_ConnectSocketFailed,        /** Socket����ʧ�ܡ�**/
    ErrCode_SocketDisconnect,           /** Socket�Ͽ����ӡ�**/
    ErrCode_CreateRequestFailed,        /** ��������ʧ�ܡ�**/
    ErrCode_RequestRelatedVariableError,/** ������ص��ڲ����������**/
    ErrCode_RequestTimeout,             /** ����ʱ��**/
    ErrCode_SendRequestFailed,          /** ����������Ϣʧ�ܡ�**/
    ErrCode_ResponseInfoIsNULL ,        /** ��Ӧ��ϢΪ�ա�**/
    ErrCode_ResolveResponseFailed ,     /** ������Ӧʧ�ܡ�**/
    ErrCode_FkFailed,                   /** ��������**/
    ErrCode_IkFailed,                   /** �������**/
    ErrCode_ToolCalibrateError,              /** ���߱궨�����д�**/
    ErrCode_ToolCalibrateParamError,         /** ���߱궨�����д�**/
    ErrCode_CoordinateSystemCalibrateError,  /** ����ϵ�궨ʧ�ܡ�**/
    ErrCode_BaseToUserConvertFailed,         /** ������ϵת�û�����ʧ�ܡ�**/
    ErrCode_UserToBaseConvertFailed,         /** �û�����ϵת������ʧ�ܡ�**/

    //move
    ErrCode_MotionRelatedVariableError,      /** �˶���ص��ڲ����������**/
    ErrCode_MotionRequestFailed,             /** �˶�����ʧ��**/
    ErrCode_CreateMotionRequestFailed,       /** �����˶�����ʧ��**/
    ErrCode_MotionInterruptedByEvent,        /** �˶����¼��жϡ�**/
    ErrCode_MotionWaypointVetorSizeError,    /** �˶���ص�·�������ĳ��Ȳ����Ϲ涨��**/
    ErrCode_ResponseReturnError,             /** ��������Ӧ���ش���**/
    ErrCode_RealRobotNoExist,                /** ��ʵ��е�۲����ڣ���Ϊ��Щ�ӿ�ֻ�������ǻ�е�۴��ڵ�����²ſ��Ա����á�**/


    ErrCode_Count = ErrCode_RealRobotNoExist-ErrCode_Base+2,

}RobotErrorCode;


}
#ifdef __cplusplus
}
#endif



/**
 * @brief ��ȡʵʱ�ؽ�״̬�Ļص���������.
 * @param jointStatus����ǰ�Ĺؽ�״̬;
 * @param size������������һ��������jointStatus���ĳ���;
 * @param arg���������������������ʹ������ע��ص������д��ݵĵڶ�������;
 */
typedef void (*RealTimeJointStatusCallback)(const aubo_robot_namespace::JointStatus *jointStatus, int size, void *arg);


/**
 * @brief ��ȡʵʱ·����Ϣ�Ļص���������.
 * @param wayPoint��   ��ǰ��·����Ϣ;
 * @param arg���������������������ʹ������ע��ص������д��ݵĵڶ�������;
 */
typedef void (*RealTimeRoadPointCallback)  (const aubo_robot_namespace::wayPoint_S  *wayPoint, void *arg);


/**
 *@brief ��ȡʵʱĩ���ٶȵĻص���������
 *@param speed   ��ǰ��ĩ���ٶ�;
 *@param arg���������������ʹ������ע��ص������д��ݵĵڶ�������;
 */
typedef void (*RealTimeEndSpeedCallback)  (double speed, void *arg);


/**
 * @brief  ��ȡ��е���¼���Ϣ�Ļص���������
 * @param arg�����������ʹ������ע��ص������д��ݵĵڶ�������;
 */
typedef void (*RobotEventCallback)         (const aubo_robot_namespace::RobotEventInfo *eventInfo, void *arg);






#endif // AUBOROBOTMETATYPE_H
