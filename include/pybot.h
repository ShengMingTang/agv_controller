#ifndef PYBOT_H
#define PYBOT_H
// #include <set>
using namespace std;

namespace  pybot
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
    // Lidar_dir
    const size_t 
        LIDAR_DIR_FRONT = 0,
        LIDAR_DIR_BACK = 1,
        LIDAR_DIR_LEFT = 2,
        LIDAR_DIR_RIGHT = 3;
    // Lidar_level
    const int16_t 
        LIDAR_LEVEL_H = 1,
        LIDAR_LEVEL_M = 2,
        LIDAR_LEVEL_L = 3;
    enum class Mode: int
    {
        MODE_IDLE = 1,
        MODE_HOMING = 2,
        MODE_TRAINING = 3,
        MODE_WORKING = 4,
    };
    enum class Opcode: char
    {
        // uart-defined
        OPCODE_HOMEING = 'A',
        OPCODE_ORIGIN = 'B',
        OPCODE_CALIB_BEGIN = 'C',
        OPCODE_CALIB_FINISH = 'D',
        OPCODE_TRAIN_BEGIN = 'E',
        OPCODE_SETNODE = 'F',
        OPCODE_TRAIN_FINISH = 'G',
        OPCODE_WORK_BEGIN = 'H',
        OPCODE_WORK_FINISH = 'I',
        OPCODE_POWEROFF = 'K',
        OPCODE_DRIVE = 'M',
        /* self-defined */
        OPCODE_NONE = '0',
        OPCODE_RT_UP = '1',
        OPCODE_RT_DOWN = '2',
        OPCODE_ND_UP = '3',
        OPCODE_ND_DOWN = '4',
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
}
#endif