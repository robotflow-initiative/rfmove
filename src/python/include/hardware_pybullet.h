//
// Created by yongxi on 2021/7/11.
//

#ifndef MOVEIT_NO_ROS_HARDWARE_PYBULLET_H
#define MOVEIT_NO_ROS_HARDWARE_PYBULLET_H

#include <pybind11/pybind11.h>

namespace py=pybind11;

class PybulletHardware {
public:
    typedef py::detail::accessor<py::detail::accessor_policies::str_attr> PyAttr;
    explicit PybulletHardware(py::handle pybullet);
    int getNumBodies();
private:
    py::object pybullet_;
    PyAttr getNumBodies_;
    PyAttr getJointInfo_;
};

#endif //MOVEIT_NO_ROS_HARDWARE_PYBULLET_H
