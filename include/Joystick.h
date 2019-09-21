// #ifndef JOYSTICK_H
// #define JOYSTICK_H
// #include <ros/ros.h>
// #include <queue>
// #include <string>
// #include <functional>
// #include "pybot.h"
// using namespace std;
// using namespace std::placeholders;
// using namespace pybot;
// using namespace tircgo_uart;
// namespace pybot
// {
//     class Joystick
//     {
//     public:
//         Joystick();
//         ~Joystick();
//         JoystickOp pop();
//     private:
//         ros::NodeHandle n;
//         ros::Subscriber sub;
//         queue<JoystickOp> que;
//         void callback(const JoystickIO::ConstPtr& _msg); // implement
//     };
// }

// #endif