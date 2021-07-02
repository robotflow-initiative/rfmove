//
// Created by yongxi on 2021/6/29.
//

#ifndef MOVEIT_NO_ROS_ROKAE_TRAJECTORY_H
#define MOVEIT_NO_ROS_ROKAE_TRAJECTORY_H

#include <vector>
#include <string>

struct RobotTrajectoryWaypoint {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
};
typedef std::unique_ptr<RobotTrajectoryWaypoint> RobotTrajectoryWaypointPtr;

struct RobotTrajectoryInfo {
    std::string group_name;
    int waypoint_count;
    std::vector<std::string> joint_names;
};
typedef std::unique_ptr<RobotTrajectoryInfo> RobotTrajectoryInfoPtr;

struct RobotTrajectory {
    RobotTrajectoryInfo info;
    std::vector<RobotTrajectoryWaypoint> waypoints;
};
typedef std::unique_ptr<RobotTrajectory> RobotTrajectoryPtr;

#endif //MOVEIT_NO_ROS_ROKAE_TRAJECTORY_H
