//
// Created by yongxi on 6/22/21.
//

#ifndef MOVEIT_NO_ROS_ROBOTTRAJECTORY_H
#define MOVEIT_NO_ROS_ROBOTTRAJECTORY_H

#include <pybind11/pybind11.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <controller/trajectory.h>

namespace py = pybind11;

void declare_robot_trajectory(py::module& m) {
    py::class_<robot_trajectory::RobotTrajectory, std::shared_ptr<robot_trajectory::RobotTrajectory>>(m, "RobotTrajectory")
        .def_property("group_name", &robot_trajectory::RobotTrajectory::getGroupName, &robot_trajectory::RobotTrajectory::setGroupName)
        .def_property("waypoints", [](robot_trajectory::RobotTrajectory& self){
            std::vector<moveit::core::RobotStatePtr> res;
            std::size_t size = self.getWayPointCount();
            for(std::size_t i= 0; i<size; ++i) {
                res.push_back(self.getWayPointPtr(i));
            }
            return res;
        }, nullptr)
        .def_property("durations", [](robot_trajectory::RobotTrajectory& self) {
            std::vector<double> res;
            size_t waypoint_count = self.getWayPointCount();
            for(size_t i = 0; i < waypoint_count; ++i) {
                res.push_back(self.getWayPointDurationFromPrevious(i));
            }
            return res;
        }, nullptr)
        .def("computeTimeStamps", [](robot_trajectory::RobotTrajectory& self) -> void{
            trajectory_processing::IterativeSplineParameterization parameterization;
            parameterization.computeTimeStamps(self);
        });
}

#endif //MOVEIT_NO_ROS_ROBOTTRAJECTORY_H
