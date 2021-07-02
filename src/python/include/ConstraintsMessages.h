//
// Created by yongxi on 6/22/21.
//

#ifndef MOVEIT_NO_ROS_CONSTRAINTSMESSAGES_H
#define MOVEIT_NO_ROS_CONSTRAINTSMESSAGES_H

#include <moveit_msgs/Constraints.h>
#include <pybind11/pybind11.h>
#include <moveit/kinematic_constraints/utils.h>
#include <sstream>

namespace py = pybind11;

void declare_constraints_msgs(py::module& m) {
    py::class_<moveit_msgs::JointConstraint>(m, "JointConstraint")
        .def_readwrite("joint_name", &moveit_msgs::JointConstraint::joint_name)
        .def_readwrite("position", &moveit_msgs::JointConstraint::position)
        .def_readwrite("tolerance_above", &moveit_msgs::JointConstraint::tolerance_above)
        .def_readwrite("tolerance_below", &moveit_msgs::JointConstraint::tolerance_below)
        .def_readwrite("weight", &moveit_msgs::JointConstraint::weight);

    py::class_<moveit_msgs::PositionConstraint>(m, "PositionConstraint")
        .def_readwrite("link_name", &moveit_msgs::PositionConstraint::link_name)
        .def_readwrite("target_point_offset", &moveit_msgs::PositionConstraint::target_point_offset)
        //.def_readwrite("constraint_region", &moveit_msgs::PositionConstraint::constraint_region)
        .def_readwrite("weight", &moveit_msgs::PositionConstraint::weight);

    py::class_<moveit_msgs::Constraints>(m, "Constraints")
        .def("__str__", [](moveit_msgs::Constraints& self) {
            std::ostringstream sstr;
            sstr << self;
            return sstr.str();
        })
        .def("__repr__", [](moveit_msgs::Constraints& self) {
            std::ostringstream sstr;
            sstr << "<moveit_msgs::Constraints>\n";
            sstr << self;
            return sstr.str();
        })
        .def_readwrite("joint_constraint", &moveit_msgs::Constraints::joint_constraints)
        .def_readwrite("position_constraint", &moveit_msgs::Constraints::position_constraints);

    m.def("constructGoalConstraints", static_cast<moveit_msgs::Constraints(*)
        (const std::string& link_name, const geometry_msgs::PoseStamped& pose, double tolerance_pose, double tolerance_angle)>
        (&kinematic_constraints::constructGoalConstraints));
}

#endif //MOVEIT_NO_ROS_CONSTRAINTSMESSAGES_H
