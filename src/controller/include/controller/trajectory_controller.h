//
// Created by yongxi on 2021/7/11.
//

#ifndef MOVEIT_NO_ROS_TRAJECTORY_CONTROLLER_H
#define MOVEIT_NO_ROS_TRAJECTORY_CONTROLLER_H

#include "controller/hardware.h"
#include "controller/trajectory.h"
#include <chrono>

class Controller {
public:
    Controller(HardwareInterface* hardware, double interval = 0.01);
    ~Controller();
    bool isRunning();
    bool executeTrajectory(robot_trajectory::RobotTrajectoryPtr robot_trajectory,
                           bool computeTimeStamps = true,
                           SplineTrajectory::Parameterization param = SplineTrajectory::Parameterization::SPLINE);
    bool executeTrajectory(SplineTrajectoryPtr spline_trajectory);
    bool canExecute(SplineTrajectoryPtr spline_trajectory);
    bool canExecute(const std::vector<std::string>& joint_names);
    bool canExecute(const std::string& joint_name);

    /**
     *
     * @param joint_name
     * @return
     */
    bool loadJointHandler(const std::string& joint_name);
    bool loadJointGroupHandler(const std::string& group_name, const std::vector<std::string>& joint_names);
private:

    SplineTrajectoryPtr current_trajectory_;
    std::map<const std::string, JointHandler*> handlers_;
    std::map<const std::string, JointGroupHandler*> group_handlers_;
    HardwareInterface* hardware_;
    double interval_;
    std::chrono::high_resolution_clock::duration interval_chrono_;

    /// @todo Use mutex and lock to maintain synchronization.
    bool isRunning_;
};

#endif //MOVEIT_NO_ROS_TRAJECTORY_CONTROLLER_H
