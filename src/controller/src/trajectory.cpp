//
// Created by yongxi on 2021/7/6.
//

#include "controller/trajectory.h"
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <algorithm> // find in vector
#include <trajectory_interface/trajectory_interface.h>

SplineTrajectory::SplineTrajectory(robot_trajectory::RobotTrajectoryPtr robot_trajectory,
                                   bool computeTimeStamps,
                                   Parameterization param) {
    if(computeTimeStamps) {
        SplineTrajectory::computeTimeStamps(robot_trajectory, param);
    } else if(robot_trajectory->getAverageSegmentDuration() == 0.0) {
        ROS_ERROR("Trajectory has no time stamp.");
    }
    group_name_ = robot_trajectory->getGroupName();
    const robot_model::JointModelGroup* joint_model_group = robot_trajectory->getGroup();
    joint_names_ = joint_model_group->getJointModelNames();
    //size_t waypoint_count = trajectory->getWayPointCount();
    //if(waypoint_count < 2) {
    //    ROS_ERROR_STREAM("Not enough trajectory waypoints to create spline segments."
    //                     " Required 2 at least but only " << waypoint_count << " given.");
    //    return;
    // }

    // Convert RobotTrajectory to trajectory_msgs/JointTrajectory
    robot_trajectory->getRobotTrajectoryMsg(robot_trajectory_msg_);


    // Convert trajectory_msgs/JointTrajectory to controller trajectory
    trajectory_ = joint_trajectory_controller::initJointTrajectory<Trajectory>(robot_trajectory_msg_.joint_trajectory,
                                                                               ros::Time(0));
    /*
    for(size_t i=0; i<waypoint_count - 1; ++i) {
        const moveit::core::RobotState& start_state = trajectory->getWayPoint(i);
        const moveit::core::RobotState& end_state = trajectory->getWayPoint(i+1);
        segments_.push_back(trajectory_interface::QuinticSplineSegment<double>(trajectory->getWayPointDurationFromStart(i),
                                                                               start_state,
                                                                               trajectory->getWayPointDurationFromStart(i+1),
                                                                               end_state));
    }
     */
}

int SplineTrajectory::sample(const std::string& joint_name, trajectory_interface::PosVelAccState<double>& sample, double duration) {
    int index = jointIndex(joint_name);
    if(index < 0) {
        return 0;
    }
    Segment::State sampled_state;
    sample = trajectory_interface::PosVelAccState<double>();
    TrajectoryPerJoint& joint_trajectory = trajectory_.at(index);
    double start_time = joint_trajectory.front().startTime();
    double end_time = joint_trajectory.back().endTime();
    //double target_time = start_time;
    int sample_count = 0;
    //std::cout << "sample start" << std::endl;
    for(double target_time = start_time; target_time <= end_time; target_time+=duration) {
        trajectory_interface::sample(joint_trajectory, target_time, sampled_state);
        //if(sampled_state.position.size() != 1) {
        //    std::cout << "sampled state position size " << sampled_state.position.size() << std::endl;
        //}
        sample.position.push_back(sampled_state.position[0]);
        //if(sampled_state.velocity.size() != 1) {
        //    std::cout << "sampled state velocity size " << sampled_state.velocity.size() << std::endl;
        //}
        sample.velocity.push_back(sampled_state.velocity[0]);
        //if(sampled_state.acceleration.size() != 1) {
        //    std::cout << "sampled state acceleration size " << sampled_state.acceleration.size() << std::endl;
        //}
        sample.acceleration.push_back(sampled_state.acceleration[0]);
        ++sample_count;
    }
    //std::cout << "sample end" << std::endl;
    return sample_count;
}

double SplineTrajectory::duration() {
    if(trajectory_.empty()) {
        ROS_WARN("Empty spline trajectory.");
        return 0;
    }
    TrajectoryPerJoint& any_joint_trajectory = trajectory_.at(0);
    return any_joint_trajectory.back().endTime() - any_joint_trajectory.front().startTime();
}

double SplineTrajectory::startTime() {
    if(trajectory_.empty()) {
        ROS_WARN("Empty spline trajectory.");
        return 0;
    }
    TrajectoryPerJoint& any_joint_trajectory = trajectory_.at(0);
    return any_joint_trajectory.front().startTime();
}

double SplineTrajectory::endTime() {
    if(trajectory_.empty()) {
        ROS_WARN("Empty spline trajectory.");
        return 0;
    }
    TrajectoryPerJoint& any_joint_trajectory = trajectory_.at(0);
    return any_joint_trajectory.back().endTime();
}

int SplineTrajectory::jointIndex(const std::string& joint_name) {
    int index = 0;
    for(const auto& joint : robot_trajectory_msg_.joint_trajectory.joint_names) {
        if(joint==joint_name) {
            return index;
        }
        ++index;
    }
    ROS_WARN_STREAM("Can not find joint " << joint_name << " in spline trajectory.");
    return -1;
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