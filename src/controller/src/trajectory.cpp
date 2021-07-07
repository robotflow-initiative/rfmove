//
// Created by yongxi on 2021/7/6.
//

#include "controller/trajectory.h"
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

SplineTrajectory::SplineTrajectory(robot_trajectory::RobotTrajectoryPtr trajectory,
                                   bool computeTimeStamps,
                                   Parameterization param) {
    if(computeTimeStamps) {
        SplineTrajectory::computeTimeStamps(trajectory, param);
    } else if(trajectory->getAverageSegmentDuration() == 0.0) {
        ROS_ERROR("Trajectory has no time stamp.");
    }
    group_name_ = trajectory->getGroupName();
    const robot_model::JointModelGroup* joint_model_group = trajectory->getGroup();
    joint_names_ = joint_model_group->getJointModelNames();
    size_t waypoint_count = trajectory->getWayPointCount();
    if(waypoint_count < 2) {
        ROS_ERROR_STREAM("Not enough trajectory waypoints to create spline segments."
                         " Required 2 at least but only " << waypoint_count << " given.");
        return;
    }

    for(size_t i=0; i<waypoint_count - 1; ++i) {
        const moveit::core::RobotState& start_state = trajectory->getWayPoint(i);
        const moveit::core::RobotState& end_state = trajectory->getWayPoint(i+1);
        segments_.push_back(trajectory_interface::QuinticSplineSegment<double>(trajectory->getWayPointDurationFromStart(i),
                                                                               start_state,
                                                                               trajectory->getWayPointDurationFromStart(i+1),
                                                                               end_state));
    }
}

void SplineTrajectory::computeTimeStamps(robot_trajectory::RobotTrajectoryPtr trajectory, Parameterization param) {
    switch (param) {
        case Parameterization::SPLINE: {
            trajectory_processing::IterativeSplineParameterization parameterization;
            parameterization.computeTimeStamps(*trajectory);
            break;
        }
        case Parameterization::TIME: {
            trajectory_processing::IterativeParabolicTimeParameterization parameterization_time;
            parameterization_time.computeTimeStamps(*trajectory);
            break;
        }
        default:
            ROS_ERROR_STREAM("Unknown trajectory time stamps parameterization: " << static_cast<int>(param));
    }
}