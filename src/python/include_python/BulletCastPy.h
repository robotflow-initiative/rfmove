//
// Created by yongxi on 2021/6/10.
//

#ifndef MOVEIT_NO_ROS_BULLETCASTPY_H
#define MOVEIT_NO_ROS_BULLETCASTPY_H

#include <pybind11/pybind11.h>

namespace py = pybind11;

void printOutJointInfo(const py::tuple& l) {
    std::cout << "JointIndex:\t" <<  l[0].cast<int>() << std::endl;
    std::cout << "JointName:\t" << l[1].cast<std::string>() << std::endl;
}

#endif //MOVEIT_NO_ROS_BULLETCASTPY_H
