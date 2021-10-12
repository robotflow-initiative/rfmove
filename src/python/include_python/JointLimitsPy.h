#ifndef MOVEIT_NO_ROS_JOINTLIMITS_PY_H
#define MOVEIT_NO_ROS_JOINTLIMITS_PY_H

#include <moveit/robot_model/joint_model.h>
#include <robot_model/JointLimitsLoader.h>
#include <pybind11/pybind11.h>

namespace py=pybind11;

void declare_jointlimits_loader(py::module& m){

    py::class_<JointLimitsLoader,std::shared_ptr<JointLimitsLoader>>(m,"JointLimitsLoader")
        .def(py::init<const std::string &>())
        .def("getBound",&JointLimitsLoader::getBound)
        .def("has",&JointLimitsLoader::has)
        .def("mergeBounds",&JointLimitsLoader::mergeBounds)
        .def("__repr__",
            [](const JointLimitsLoader &jointlimitloader){
                return "<object for jointlimit loader>";
            });
}

#endif