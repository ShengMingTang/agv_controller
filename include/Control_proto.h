#ifndef TIRCGO_PROTOCOL_H
#define TIRCGO_PROTOCOL_H
using namespace std;
#define CLASS_HEADER_FILL(header)\
{\
    header.stamp = ros::Time()::now();\
    header.frame_id = this->frame_id;\
}

#define MSG_QUE_SIZE 1
#define ANGULAR_FACTOR 0.1
/* motor limits */
#define MOTOR_LINEAR_LIMIT 60
#define MOTOR_ANGULAR_LIMIT 10
/* lidar */
#define LIDAR_DIR_FRONT 0
#define LIDAR_DIR_BACK 1
#define LIDAR_DIR_LEFT 2
#define LIDAR_DIR_RIGHT 3
/* lidar level */
#define LIDAR_LEVEL_FAR 0
#define LIDAR_LEVEL_MED 1
#define LIDAR_LEVEL_CLOSE 2

/* device control */
#define DEVICE_BEEPER 0
#define DEVICE_LED_G 1
#define DEVICE_LED_Y 2
#define DEVICE_LED_R 3

#define DEVICE_ON 0
#define DEVICE_OFF 1

#define DEVICE_BEEPER_3L_2S 2
#define DEVICE_BEEPER_2S 3
#define DEVICE_BEEPER_L 4

#define DEVICE_LED_ON 0
#define DEVICE_LED_OFF 1
#define DEVICE_LED_SLOW 2
#define DEVICE_LED_MEDI 3
#define DEVICE_LED_FAST 4

/* setnode control */
#define SETNODE_PASS_SLIDE 1
#define SETNODE_PASS_EXACT 0
#define SETNODE_HEADWAY_HEAD 1
#define SETNODE_HEADWAY_TAIL 0

/* UART */
#define TRACKING_STATUS_NONE 0
#define TRACKING_STATUS_NORMAL 1
#define TRACKING_STATUS_OUT -2
#define TRACKING_STATUS_OBSTACLE -3
#define TRACKING_STATUS_ARRIVAL 127
/* uart error code */
#define ERRCODE_NONE 0
#define ERRCODE_OK 1
#define ERRCODE_NOT_OK -1
#define ERRCODE_NETWROK -2
#define ERRCODE_POINTS_TOO_CLOSE -3
#define ERRCODE_LOST -4
#define ERRCODE_INVAILD_OPCODE -5
/* uart related topic */
#define ROBOTINVOKE_TOPIC "robot_invoke"
#define ROBOTSTATUS_TOPIC "robot_status"

/* Joystick */
#define JOY_CD 0.05
/* buttons */
#define JOYBUTTON_X 0
#define JOYBUTTON_A 1
#define JOYBUTTON_B 2
#define JOYBUTTON_Y 3
#define JOYBUTTON_LB 4
#define JOYBUTTON_RB 5
#define JOYBUTTON_LT 6
#define JOYBUTTON_RT 7
#define JOYBUTTON_BACK 8
#define JOYBUTTON_START 9
#define JOYBUTTON_STICK_LEFT 10
#define JOYBUTTON_STICK_RIGHT 11
/* axes, L > 0, R < 0, U > 0, D < 0 */
#define JOYAXES_STICKLEFT_LR 0
#define JOYAXES_STICKLEFT_UD 1
#define JOYAXES_STICKRIGHT_LR 2
#define JOYAXES_STICKRIGHT_UD 3
#define JOYAXES_CROSS_LR 4
#define JOYAXES_CROSS_UD 5
/* joystick related topic */
#define JOYSTICKIO_TOPIC "joy" 

/* controller-defined opcode */
#define OPCODE_NONE '0' 
#define OPCODE_RT_UP '1' 
#define OPCODE_RT_DOWN '2' 
#define OPCODE_ND_UP '3' 
#define OPCODE_ND_DOWN '4' 
#define OPCODE_DISPLAY '5'
#define OPCODE_AUTO_BEGIN '6'
#define OPCODE_AUTO_FINISH '7'
#define OPCODE_DELAY '8'

/* hardware control */
#define OPCODE_SHUTDOWN 'E' 
#define OPCODE_DRIVE 'F' 
#define OPCODE_SIGNAL 'G'  // for LED and beeper
#define OPCODE_CALIB 'H' 
#define OPCODE_TRAIN_BEGIN 'I' 
#define OPCODE_SETNODE 'J' 
#define OPCODE_TRAIN_FINISH 'K' 
#define OPCODE_WORK_BEGIN 'L' 
#define OPCODE_WORK_FINISH 'M' 

/* Wifi */
#define ROBOT_WIFI_TOPIC "wifi_topic"  // implement
#define ROBOT_WIFI_SEND_SRV "robot_wifi_send" 
#define ROBOT_WIFI_NODEOCP_OUTER "robot_wifi_nodeocp_outer"  // robot ask other robots
#define ROBOT_WIFI_NODEOCP_INNER "robot_wifi_nodeocp_inner"  // robot answer other robots
#define ROBOT_WIFI_NODECOST_OUTER "robot_wifi_nodecost_outer"  // robot ask other robots
#define ROBOT_WIFI_NODECOST_INNER "robot_wifi_nodecost_inner"  // robot answer other robots
#define ROBOT_WIFI_TASK_CONFIRM_INNER "robot_wifi_taskconfirm_inner" 
#define ROBOT_WIFI_ASKDATA_INNER "robot_wifi_askdata_inner" 

/* wifi purpose */
#define WIFI_PUR_WS "W"
#define WIFI_PUR_ROBOT "R"
#define WIFI_PUR_ANS "A"
#define WIFI_PUR_NODEOCP "N"
#define WIFI_PUR_COST "C"
/* wifi error code */
#define WIFI_ERRCODE_NONE "0"
#define WIFI_ERRCODE_TIMEOUT "T"
#define WIFI_ERRCODE_WS "W"
#define WIFI_ERRCODE_ROBOT "R"
#define WIFI_ERRCODE_ANS "A"
#define WIFI_ERRCODE_NODE "N"
#define WIFI_ERRCODE_COST "C"
#define WIFI_ERRCODE_NETWORK "w"

/* basic info publish topic (in string) */
#define MONITOR_TOPIC "controller/monitor_topic" 

/* scheduler */
#define ROBOT_SCHEDULER_CONTROLLER "robot_scheduler_controller" // scheduler send to controller
#define ROBOT_CONTROLLER_SCHEDULER "robot_controller_scheduler" // controller sned to scheduler

/* controller talk */
#define ROBOT_CONTROLLER_TALK_OUTER "robot_wifi_controller_talk_outer"
#define ROBOT_CONTROLLER_TALK_INNER "robot_wifi_controller_talk_inner"

#define ROBOT_CONTROLLER_AGENT "to_agent"
#define ROBOT_AGENT_CONTROLLER "from_agent"
namespace tircgo
{
}
#endif