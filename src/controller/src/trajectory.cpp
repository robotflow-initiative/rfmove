//
// Created by yongxi on 2021/7/6.
//

#include "controller/trajectory.h"

SplineTrajectory::SplineTrajectory(robot_trajectory::RobotTrajectoryPtr trajectory) {
    group_name_ = trajectory->getGroupName();
    const robot_model::JointModelGroup* joint_model_group = trajectory->getGroup();
    joint_names_ = joint_model_group->getJointModelNames();
    size_t waypoint_count = trajectory->getWayPointCount();
    trajectory->getAverageSegmentDuration();
    for(size_t i=0; i<waypoint_count; ++i) {
        trajectory->getWayPoint(i);
    }
}