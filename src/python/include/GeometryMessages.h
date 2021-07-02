//
// Created by yongxi on 6/22/21.
//

#ifndef MOVEIT_NO_ROS_GEOMETRYMESSAGES_H
#define MOVEIT_NO_ROS_GEOMETRYMESSAGES_H

#include <geometry_msgs/Vector3.h>
#include <pybind11/pybind11.h>
#include <sstream>

namespace py = pybind11;

/**
 * Generate geometry_msgs from python.
 * @param frame_id The coordinate frame id.
 * @param position A 3d array demonstrate position.
 * @param orientation A 4d array. Orientation in quaternion form.
 * @return PoseStamped which can be used in constraints.
 */
geometry_msgs::PoseStamped createPoseStamped(
        const std::string& frame_id,
        py::array_t<double> position,
        py::array_t<double> orientation) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = *position.data(0);
    pose.pose.position.y = *position.data(1);
    pose.pose.position.z = *position.data(2);

    pose.pose.orientation.x = *orientation.data(0);
    pose.pose.orientation.y = *orientation.data(1);
    pose.pose.orientation.z = *orientation.data(2);
    pose.pose.orientation.w = *orientation.data(3);
    return pose;
}

void declare_geometry_msgs(py::module& m) {
    py::class_<geometry_msgs::Vector3>(m, "Vector3")
        .def_readwrite("x", &geometry_msgs::Vector3::x)
        .def_readwrite("y", &geometry_msgs::Vector3::y)
        .def_readwrite("z", &geometry_msgs::Vector3::z);
    py::class_<geometry_msgs::PoseStamped>(m, "PoseStamped")
        .def(py::init(&createPoseStamped))
        .def("__str__", [](geometry_msgs::PoseStamped& self) {
            std::ostringstream sstr;
            sstr << self;
            return sstr.str();
        })
        .def("__repr__", [](geometry_msgs::PoseStamped& self) {
            std::ostringstream sstr;
            sstr << "<geometry_msgs::PoseStamped>\n";
            sstr << self;
            return sstr.str();
        });
}

#endif //MOVEIT_NO_ROS_GEOMETRYMESSAGES_H
