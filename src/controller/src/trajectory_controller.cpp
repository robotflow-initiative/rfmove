//
// Created by yongxi on 2021/7/11.
//

#include "controller/trajectory_controller.h"
#include <chrono>
#include <thread>

Controller::Controller(HardwareInterface *hardware, double interval) {
    hardware_ = hardware;
    interval_ = interval;
    interval_chrono_ = std::chrono::duration_cast
            <std::chrono::high_resolution_clock::duration>
            (std::chrono::duration<double>(interval));
    isRunning_ = false;
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

        // Send command
        /// @todo There should be timeout functionality for handler.
        if(!group_handler->setPosVels(sampled.position, sampled.velocity)){
            ROS_ERROR_STREAM("Can not send command to group " << spline_trajectory->getGroupName());
            return false;
        }

        /// @todo Add call back during the execution of trajectory.


        // Update sample time
        sample_time += interval_;
        update_time += interval_chrono_;
    }
    return true;
}

bool Controller::isRunning() {
    return isRunning_;
}

bool Controller::canExecute(const std::string &joint_name) {
    if(handlers_.find(joint_name) == handlers_.end()) {
        return loadJointHandler(joint_name);
    } else {
        return true;
    }
}

bool Controller::canExecute(const std::vector<std::string> &joint_names) {
    bool result = true;
    for(const auto& joint_name : joint_names) {
        result  = (result && canExecute(joint_name));
    }
    return result;
}

bool Controller::canExecute(SplineTrajectoryPtr spline_trajectory) {
    if(group_handlers_.find(spline_trajectory->getGroupName()) == group_handlers_.end()) {
        return loadJointGroupHandler(spline_trajectory->getGroupName(), spline_trajectory->getJointNames());
    } else {
        return true;
    }
}