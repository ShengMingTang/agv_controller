// #ifndef WIFI_H
// #define WIFI_H
// #include <ros/ros.h>
// #include <queue>
// #include "pybot.h"
// #include "wifi/WifiIO.h"
// #include "wifi/WifiSend.h"
// using namespace std;
// using namespace pybot;
// namespace pybot
// {
//     class Wifi
//     {
//     public:
//         Wifi(const string& _parent_frame_id);
//         ~Wifi();
//         void send(wifi::WifiIO _msg);
//         bool is_msg_valid(wifi::WifiIO::ConstPtr _msg);
//         wifi::WifiIO::ConstPtr pop();
//     private:
//         void recv_callback(wifi::WifiIO::ConstPtr _msg);
//         ros::NodeHandle n;
//         const string frame_id;
//         queue<wifi::WifiIO::ConstPtr> que;
//         ros::ServiceClient send_clt; //send wifi
//         ros::Subscriber recv_sub; // receiver
//     };
// }
// #endif