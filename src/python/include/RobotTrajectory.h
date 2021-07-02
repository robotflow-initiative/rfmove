//
// Created by yongxi on 6/22/21.
//

#ifndef MOVEIT_NO_ROS_ROBOTTRAJECTORY_H
#define MOVEIT_NO_ROS_ROBOTTRAJECTORY_H

#include <pybind11/pybind11.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

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
        }, nullptr);
        //.def("getMessage", [](robot_trajectory::RobotTrajectory &self) -> moveit_msgs::RobotTrajectory{
        //    moveit_msgs::RobotTrajectory trajectory;
        //    ros::serialization::Serializer<moveit_msgs::RobotTrajectory> serializer;
        //    serializer.
        //});
}

#endif //MOVEIT_NO_ROS_ROBOTTRAJECTORY_H
