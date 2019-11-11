#ifndef PYBOT_H
#define PYBOT_H
// #include <set>
using namespace std;
#define CLASS_HEADER_FILL(header)\
{\
    header.stamp = ros::Time()::now();\
    header.frame_id = this->frame_id;\
}

#define ANGULAR_FACTOR 0.1

#define LIDAR_DIR_FRONT 0
#define LIDAR_DIR_BACK 1
#define LIDAR_DIR_LEFT 2
#define LIDAR_DIR_RIGHT 3

#define LIDAR_LEVEL_H 1
#define LIDAR_LEVEL_M 2
#define LIDAR_LEVEL_L 3

#define DEVICE_BEEPER 0
#define DEVICE_LED_G 1
#define DEVICE_LED_Y 2
#define DEVICE_LED_R 3

#define DEVICE_ON 0
#define DEVICE_OFF 1

#define DEVICE_BEEPER_3L_2S 2
#define DEVICE_BEEPER_2S 3
#define DEVICE_BEEPER_L 4

#define DEVICE_LED_SLOW 2
#define DEVICE_LED_MEDI 3
#define DEVICE_LED_FAST 4

#define SETNODE_PASS_SLIDE 1
#define SETNODE_PASS_EXACT 0
#define SETNODE_HEADWAY_HEAD 1
#define SETNODE_HEADWAY_TAIL 0

namespace tircgo
{
    const int MSG_QUE_SIZE = 100;
    const float JOY_CD = 0.1;
    /* UART */
    enum class Tracking_status: int16_t
    {
        TRACKING_STATUS_NONE = 0,
        TRACKING_STATUS_NORMAL = 1,
        TRACKING_STATUS_OUT = -2,
        TRACKING_STATUS_OBSTACLE = -3,
        TRACKING_STATUS_ARRIVAL = 127,
    };
    enum class Opcode: char
    {
        /* self-defined */
        OPCODE_NONE = '0',
        OPCODE_RT_UP = '1',
        OPCODE_RT_DOWN = '2',
        OPCODE_ND_UP = '3',
        OPCODE_ND_DOWN = '4',
        
        /* tircgo1107 protocol */
        // status asking
        // OPCODE_ALIVE = 'A',
        // OPCODE_SYS_STATUS = 'B',
        // OPCODE_WORK_STATUS = 'C',
        // OPCODE_LIDAR_STATUS = 'D',

        // hardware control
        OPCODE_SHUTDOWN = 'E',
        OPCODE_DRIVE = 'F',
        OPCODE_SIGNAL = 'G', // for LED and beeper
        OPCODE_CALIB = 'H',
        OPCODE_TRAIN_BEGIN = 'I',
        OPCODE_SETNODE = 'J',
        OPCODE_TRAIN_FINISH = 'K',
        OPCODE_WORK_BEGIN = 'L',
        OPCODE_WORK_FINISH = 'M',
    };
    /* motor limits */
    const double MOTOR_LINEAR_LIMIT = 60;
    const double MOTOR_ANGULAR_LIMIT = 10;
    enum class Errcode: int16_t
    {
        ERRCODE_NONE = 0,
        ERRCODE_OK = 1,
        ERRCODE_NOT_OK = -1,
        ERRCODE_NETWROK = -2,
        ERRCODE_POINTS_TOO_CLOSE = -3,
        ERRCODE_LOST = -4,
        ERRCODE_INVAILD_OPCODE = -5,
    };
    /* end UART */
    
    /* Joystick */
    // buttons
    const int
        JOYBUTTON_X = 0,
        JOYBUTTON_A = 1,
        JOYBUTTON_B = 2,
        JOYBUTTON_Y = 3,
        JOYBUTTON_LB = 4,
        JOYBUTTON_RB = 5,
        JOYBUTTON_LT = 6,
        JOYBUTTON_RT = 7,
        JOYBUTTON_BACK = 8,
        JOYBUTTON_START = 9,
        JOYBUTTON_MODE = 10,
        JOYBUTTON_VIBRA = 11;
    // axes, L > 0, R < 0, U > 0, D < 0
    const int
        JOYAXES_STICKLEFT_LR = 0,
        JOYAXES_STICKLEFT_UD = 1,
        JOYAXES_STICKRIGHT_LR = 2,
        JOYAXES_STICKRIGHT_UD = 3,
        JOYAXES_CROSS_LR = 4,
        JOYAXES_CROSS_UD = 5;
    const char ROBOTINVOKE_TOPIC[] = "robot_invoke";
    const char ROBOTSTATUS_TOPIC[] = "robot_status";
    const char JOYSTICKIO_TOPIC[] = "joy";
    /* Wifi */
    const char ROBOT_WIFI_TOPIC[] = "wifi_topic"; // implement
    const char ROBOT_WIFI_SEND_SRV[] = "robot_wifi_send";
    const char ROBOT_WIFI_NODEOCP_OUTER[] = "robot_wifi_nodeocp_outer"; // robot ask other robots
    const char ROBOT_WIFI_NODEOCP_INNER[] = "robot_wifi_nodeocp_inner"; // robot answer other robots
    const char ROBOT_WIFI_NODECOST_OUTER[] = "robot_wifi_nodecost_outer"; // robot ask other robots
    const char ROBOT_WIFI_NODECOST_INNER[] = "robot_wifi_nodecost_inner"; // robot answer other robots
    const char ROBOT_WIFI_TASK_CONFIRM_INNER[] = "robot_wifi_taskconfirm_inner";
    // wifi purpose
    const string
        WIFI_PUR_WS = "W",
        WIFI_PUR_ROBOT = "R",
        WIFI_PUR_ANS = "A",
        WIFI_PUR_NODEOCP = "N",
        WIFI_PUR_COST = "C";
    const char
        WIFI_ERR_NONE = '0',
        WIFI_ERR_TIMEOUT = 'T',
        WIFI_ERR_WS = 'W',
        WIFI_ERR_ROBOT = 'R',
        WIFI_ERR_ANS = 'A',
        WIFI_ERR_NODE = 'N',
        WIFI_ERR_COST = 'C',
        WIFI_ERR_NETWORK = 'w';
    const int WIFI_BUFF_SIZE = 50;
    /* Debug*/
    const char MONITOR_TOPIC[] = "controller/monitor_topic";
    const char ROBOT_ASKDATA_TOPIC[] = "cost_srv";
}
#endif