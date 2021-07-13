//
// Created by yongxi on 2021/7/9.
//

#ifndef MOVEIT_NO_ROS_BULLET3_H
#define MOVEIT_NO_ROS_BULLET3_H

#include "controller/hardware_bullet3.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;

void declare_bullet3(py::module &m) {
    py::class_<Bullet3Hardware>(m, "Bullet3Hardware")
        .def(py::init<int>())
        .def_property("connected", &Bullet3Hardware::isConnected, nullptr)
        .def("getNumJoints", &Bullet3Hardware::getNumJoints)
        .def("debugGravity", &Bullet3Hardware::debugGravity)
        .def("getNumBodys", &Bullet3Hardware::getNumBodys);
}

#endif //MOVEIT_NO_ROS_BULLET3_H
