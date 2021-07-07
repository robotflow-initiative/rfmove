//
// Created by yongxi on 2021/7/6.
//

#ifndef MOVEIT_NO_ROS_TRAJECTORY_H
#define MOVEIT_NO_ROS_TRAJECTORY_H

#include <trajectory_interface/quintic_spline_segment.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

/**
 * Trajectory representation using trajectory_interface::QuinticSplineSegment.
 */
class SplineTrajectory {
public:
    /// Time stamps parameterization type
    enum class Parameterization{
        SPLINE, ///< IterativeSplineParameterization
        TIME,   ///< IterativeParabolicTimeParameterization
    };
    SplineTrajectory(robot_trajectory::RobotTrajectoryPtr trajectory,
                     bool computeTimeStamps = true,
                     Parameterization param = Parameterization::SPLINE);
    static void computeTimeStamps(robot_trajectory::RobotTrajectoryPtr trajectory, Parameterization param = Parameterization::SPLINE);
private:

    std::vector<std::string> joint_names_;
    std::string group_name_;
    std::vector<trajectory_interface::QuinticSplineSegment<double>> segments_;
};

//boost::shared_ptr<trajectory_interface::QuinticSplineSegment<double>> splineFromTrajectory(robot_trajectory::RobotTrajectoryPtr trajectory) {
//
//}

#endif //MOVEIT_NO_ROS_TRAJECTORY_H
