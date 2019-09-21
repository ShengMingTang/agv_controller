#ifndef PYBOT_H
#define PYBOT_H
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include "tircgo_uart/RobotInvoke.h" // srv header
#include "tircgo_uart/RobotStatus.h" // topic header for subscribing to the robot status

#include <string>
namespace  pybot
{
    using namespace std;
    const int MSG_QUE_SIZE = 100;
    enum class Mode: int16_t
    {
        MODE_IDLE = 1,
        MODE_HOMING = 2,
        MODE_TRAINING = 3,
        MODE_WORKING = 4,
    };
    /* UART */
    enum class Tracking_status: int16_t
    {
        TRACKING_STATUS_NONE = 0,
        TRACKING_STATUS_NORMAL = 1,
        TRACKING_STATUS_OUT = -2,
        TRACKING_STATUS_OBSTACLE = -3,
        TRACKING_STATUS_ARRIVAL = 127,
    };
    const size_t LIDAR_DIR_FRONT = 0, LIDAR_DIR_BACK = 1, LIDAR_DIR_LEFT = 2, LIDAR_DIR_RIGHT = 3;
    const int16_t LIDAR_LEVEL_H = 1, LIDAR_LEVEL_M = 2, LIDAR_LEVEL_L = 3;
    enum class Opcode: char
    {
        OPCODE_HOMEING = 'A',
        OPCODE_CLEAR = 'B',
        OPCODE_CALIB_BEGIN = 'C',
        OPCODE_CALIB_FINISH = 'D',
        OPCODE_TRAIN_BEGIN = 'E',
        OPCODE_SETNODE = 'F',
        OPCODE_TRAIN_FINISH = 'G',
        OPCODE_WORK_BEGIN = 'H',
        OPCODE_BRAKE = 'I',
        OPCODE_POWEROFF = 'K',
    };
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
    /* topics */
    const char ROBOTINVOKE_TOPIC[] = "robot_incoke_topic";
    const char ROBOTSTATUS_TOPIC[] = "robot_status_topic";
    const char ROBOTVELCMD_TOPIC[] = "manual_cmd_vel";
    /* motor limits */
    const double MOTOR_LINEAR_LIMIT = 60;
    const double MOTOR_ANGULAR_LIMIT = 10;
    /* Joystick */
    const string JOYSTICKIO_TOPIC("joystickio_topic");
    enum class JoystickOp: int16_t
    {
        //implement
    };
    /* Wifi */
    const string WIFI_TOPIC("wifi_topic"); // implement
}
#endif