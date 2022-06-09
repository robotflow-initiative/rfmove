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

    /**
     * @note Do not use joint model names from jointModelGroup as joint names directly.
     * That is because RobotTrajectoryMsg already remove all the fixed joint while jointModelGroup contains them.
     */
    //joint_names_ = joint_model_group->getJointModelNames();

    // Convert RobotTrajectory to trajectory_msgs/JointTrajectory
    robot_trajectory_ = robot_trajectory;
    robot_trajectory->getRobotTrajectoryMsg(robot_trajectory_msg_);


    // Convert trajectory_msgs/JointTrajectory to controller trajectory
    trajectory_ = joint_trajectory_controller::initJointTrajectory<Trajectory>(robot_trajectory_msg_.joint_trajectory,
                                                                               ros::Time(0));
    joint_names_ = robot_trajectory_msg_.joint_trajectory.joint_names;
    // Get tip link information
    joint_model_group->getEndEffectorTips(tip_names_);
    if(!tip_names_.empty()) {
        tip_link_ = joint_model_group->getLinkModel(tip_names_[0]);
    } else {
        std::cerr << "Error when create SplineTrajectory: " << group_name_ << " has no tip link info." << std::endl;
        return;
    }

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
//添加std::vector<double>& sample_by_interval_times
//去掉sample = trajectory_interface::PosVelAccState<double>()
int SplineTrajectory::sample_by_interval(const std::string& joint_name, trajectory_interface::PosVelAccState<double>& sample, double duration) {
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
        //sample_by_interval_times.push_back(target_time);

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

int SplineTrajectory::sample_at_time(trajectory_interface::PosVelAccState<double> &result, double time_point) {
    result.position.resize(trajectory_.size());
    result.velocity.resize(trajectory_.size());
    result.acceleration.resize(trajectory_.size());
    Segment::State sampled_state;
    size_t i = 0;
    for(auto joint_trajectory : trajectory_) {
        trajectory_interface::sample(joint_trajectory, time_point, sampled_state);
        result.position[i] = sampled_state.position[0];
        result.velocity[i] = sampled_state.velocity[0];
        result.acceleration[i] = sampled_state.acceleration[0];
        ++i;
    }
    return trajectory_.size();
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

int SplineTrajectory::getTipTransforms(std::vector<Eigen::Affine3d>& result) {
    result.resize(robot_trajectory_->getWayPointCount());
    for (size_t i=0; i < robot_trajectory_->getWayPointCount(); ++i) {
        result[i] = robot_trajectory_->getWayPoint(i).getGlobalLinkTransform(tip_link_);
    }
    return robot_trajectory_->getWayPointCount();
}

const std::string& SplineTrajectory::getGroupName() {
    return group_name_;
}

const std::vector<std::string> & SplineTrajectory::getJointNames() {
    return joint_names_;
}