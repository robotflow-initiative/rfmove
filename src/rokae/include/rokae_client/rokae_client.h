//
// Created by yongxi on 6/27/21.
//

#ifndef MOVEIT_NO_ROS_ROKAE_CLIENT_H
#define MOVEIT_NO_ROS_ROKAE_CLIENT_H

#include "moveit/robot_trajectory/robot_trajectory.h"
#include <httplib.h>

typedef std::pair<std::string, std::string> httpParam;

class RokaeClient {
public:
    explicit RokaeClient(const std::string& host = "http://127.0.0.1", int port = 8080);
    ~RokaeClient();
    void hello();
    void postTrajectory(const robot_trajectory::RobotTrajectory&);
private:
    httplib::Client* client;
};

#endif //MOVEIT_NO_ROS_ROKAE_CLIENT_H
