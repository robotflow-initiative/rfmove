//
// Created by yongxi on 6/27/21.
//

#include <rokae_client/rokae_client.h>
#include <iostream>
#include <json.hpp>

using json = nlohmann::json;

RokaeClient::RokaeClient(const std::string &host, int port) {
    std::cout << "Create Client to " << host << ':' << port << std::endl;
    /// @bug A segment fault would raise if we use initialization list. Maybe we need a newer gcc.
    client = new httplib::Client("http://127.0.0.1:8080");
}

RokaeClient::~RokaeClient() {
    delete client;
}

void RokaeClient::hello() {
    httplib::Result res = client -> Get("/hi");
    if (res->status != 200) {
        std::cerr << "Error when get response from '/hi' :" << res->status << '-' << res.error() << std::endl;
    } else {
        std::cout << res.value().body << std::endl;
    }
}

static inline void outputRes(httplib::Result& res, const std::string& path) {
    if(res->status != 200) {
        std::cerr << "Error from " << path << ": " << res.error() << std::endl;
    }
    if (!res -> body.empty()) {
        std::cout << res -> body << std::endl;
    }
}

void RokaeClient::postTrajectory(const robot_trajectory::RobotTrajectory & trajectory) {
    const std::string& group_name = trajectory.getGroupName();
    const moveit::core::JointModelGroup* joint_model_group = trajectory.getRobotModel()->getJointModelGroup(group_name);
    size_t waypoint_count = trajectory.getWayPointCount();

    // Post group name and waypoints count through /trajectory/info
    json info;
    info["group_name"] = group_name;
    info["waypoint_count"] = waypoint_count;

    for (auto const& joint_name : joint_model_group->getJointModelNames()) {
        info["joint_names"].push_back(joint_name);
    }

    httplib::Result res = client ->Post("/trajectory/info", info.dump(2), "application/json");
    outputRes(res, "/trajectory/info");


    // Post waypoints through /trajectory/waypoint
    for (size_t i = 0; i < waypoint_count; ++i) {
        const robot_state::RobotState& state = trajectory.getWayPoint(i);
        json waypoint;

        for(const auto&joint_name : joint_model_group->getJointModelNames()) {
            waypoint["velocities"].push_back(*state.getJointVelocities(joint_name));
            waypoint["positions"].push_back(*state.getJointPositions(joint_name));
            waypoint["effort"].push_back(*state.getJointEffort(joint_name));
        }

        res = client ->Post("/trajectory/waypoint", waypoint.dump(2), "application/json");
        outputRes(res, "/trajectory/waypoint");
    }

    // Post trajectory end signal
    res = client->Post("/trajectory/end", group_name, "text/plain");
    outputRes(res, "/trajectory/end");
}
