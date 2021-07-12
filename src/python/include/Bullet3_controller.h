//
// Created by yongxi on 2021/7/11.
//

#ifndef MOVEIT_NO_ROS_BULLET3_CONTROLLER_H
#define MOVEIT_NO_ROS_BULLET3_CONTROLLER_H

#include <pybind11/pybind11.h>
#include <hardware_pybullet.h>

/**
 * Class used to call pybullet within cpp.
 */


void declare_pybullet_controller(py::module& m) {
    py::class_<PybulletHardware>(m, "PybulletHardware")
        .def(py::init<py::handle, int>())
        .def("getNumBodies", &PybulletHardware::getNumBodies)
        .def("printJointInfo", &PybulletHardware::printJointInfo)
        .def("printJointState", &PybulletHardware::printJointState);
}

#endif //MOVEIT_NO_ROS_BULLET3_CONTROLLER_H
