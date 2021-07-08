//
// Created by yongxi on 2021/7/6.
//

#ifndef MOVEIT_NO_ROS_TRAJECTORY_H
#define MOVEIT_NO_ROS_TRAJECTORY_H

#include <trajectory_interface/quintic_spline_segment.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <trajectory_interface/pos_vel_acc_state.h>

/**
 * Trajectory representation using trajectory_interface::QuinticSplineSegment.
 */
class SplineTrajectory {
public:
    typedef joint_trajectory_controller::JointTrajectorySegment<trajectory_interface::QuinticSplineSegment<double>> Segment;
    typedef std::vector<Segment> TrajectoryPerJoint;
    typedef std::vector<TrajectoryPerJoint> Trajectory;

    /// Time stamps parameterization type
    enum class Parameterization{
        SPLINE, ///< IterativeSplineParameterization
        TIME,   ///< IterativeParabolicTimeParameterization
    };
    SplineTrajectory(robot_trajectory::RobotTrajectoryPtr robot_trajectory,
                     bool computeTimeStamps = true,
                     Parameterization param = Parameterization::SPLINE);
    /**
     * Sample positions.
     * @param joint_name
     * @param sample Sample result would be pushed backed into sample.
     * @return The number of position sampled.
     */
    int sample(const std::string& joint_name, trajectory_interface::PosVelAccState<double>& sample, double interval = 1e-3);
    int jointIndex(const std::string& joint_name);
    double duration();
    static void computeTimeStamps(robot_trajectory::RobotTrajectoryPtr trajectory, Parameterization param = Parameterization::SPLINE);
private:

    std::vector<std::string> joint_names_;
    std::string group_name_;
    moveit_msgs::RobotTrajectory robot_trajectory_msg_;
    Trajectory trajectory_;
};

//boost::shared_ptr<trajectory_interface::QuinticSplineSegment<double>> splineFromTrajectory(robot_trajectory::RobotTrajectoryPtr trajectory) {
//
//}

#endif //MOVEIT_NO_ROS_TRAJECTORY_H
