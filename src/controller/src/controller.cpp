//
// Created by yongxi on 2021/7/11.
//

#include "controller/controller.h"
#include <chrono>
#include <thread>

Controller::Controller(HardwareInterface *hardware, double interval)
    : interval_chrono_(interval)
    , half_interval_chrono_(interval/2)
{
    hardware_ = hardware;
    interval_ = interval;
    half_interval_ = interval/2;
}

Controller::~Controller() {
    for (const auto& handler : handlers_ ){
        delete handler.second;
    }
    for (const auto& group_handler : group_handlers_) {
        delete group_handler.second;
    }
    delete hardware_;
}

bool Controller::loadJointHandler(const std::string &joint_name) {
    if(handlers_.find(joint_name) != handlers_.end()) {
        return true;
    } else {
        JointHandler* handler = hardware_->getJointHandler(joint_name);
        if(handler != nullptr) {
            handlers_[joint_name] = handler;
            return true;
        } else {
            return false;
        }
    }
}

bool Controller::loadJointGroupHandler(const std::string &group_name, const std::vector<std::string> &joint_names) {
    if(group_handlers_.find(group_name) != group_handlers_.end()) {
        return true;
    } else {
        JointGroupHandler* group_handler = hardware_->getJointGroupHandler(joint_names);
        if(group_handler != nullptr) {
            group_handlers_[group_name] = group_handler;
            return true;
        } else {
            return false;
        }
    }
}

bool Controller::executeTrajectory(robot_trajectory::RobotTrajectoryPtr robot_trajectory, bool computeTimeStamps,
                                   SplineTrajectory::Parameterization param) {
    SplineTrajectoryPtr spline_trajectory = std::make_shared<SplineTrajectory>(robot_trajectory, computeTimeStamps, param);
    return executeTrajectory(spline_trajectory);
}

bool Controller::executeTrajectory(SplineTrajectoryPtr spline_trajectory) {
    // Fetch JointGroupHandler
    JointGroupHandler* group_handler;
    if(group_handlers_.find(spline_trajectory->getGroupName()) == group_handlers_.end()) {
        if (!loadJointGroupHandler(spline_trajectory->getGroupName(), spline_trajectory->getJointNames())) {
            ROS_ERROR_STREAM("Can not load group handler for " << spline_trajectory->getGroupName());
            return false;
        }
    }
    group_handler = group_handlers_[spline_trajectory->getGroupName()];

    auto update_time = std::chrono::high_resolution_clock::now();
    double sample_time = spline_trajectory->startTime();
    trajectory_interface::PosVelAccState<double> sampled;
    while(sample_time <= spline_trajectory->endTime()) {
        // Sample
        std::this_thread::sleep_until(update_time);
        spline_trajectory->sample_at_time(sampled, sample_time);
        sample_time += interval_;
        //update_time += interval_chrono_;

    }
    // Compute chrono duration from SplineTrajectory
}