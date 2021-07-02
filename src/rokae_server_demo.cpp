//
// Created by yongxi on 6/28/21.
//
#include <httplib.h>
#include <rokae_server/rokae_server.h>

int main() {
    RokaeServer server;
    server.setInfoCallback([](RobotTrajectoryInfoPtr info) {
       std::cout << "Info:" << std::endl;
       std::cout << "..." << "group_name: " << info->group_name << std::endl;
       std::cout << "..." << "waypoint_count: " << info->waypoint_count << std::endl;
       std::cout << "..." << "joint_names:" << std::endl;
       for(const std::string& joint_name : info->joint_names) {
           std::cout << "......" << joint_name << std::endl;
       }
    });
    server.setWaypointCallback([](RobotTrajectoryWaypointPtr waypoint) {
        std::cout << "Waypoint:" << std::endl;
        std::cout << "...position: " << waypoint-> position.size() << std::endl;
        std::cout << "...velocity: " << waypoint->velocity.size() << std::endl;
    });
    server.setEndCallback([](const std::string& str){
        std::cout << "End:" << std::endl;
        std::cout << "..." << str << std::endl;
    });
    server.start();
    return 0;
}