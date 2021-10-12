//
// Created by yongxi on 2021/6/1.
//
#ifndef MOVEIT_NO_ROS_ROBOTMODEL_H
#define MOVEIT_NO_ROS_ROBOTMODEL_H

#include <pybind11/pybind11.h>
#include <moveit/robot_model/robot_model.h>

namespace py = pybind11;

void declare_robot_model(py::module &m){
    py::class_<moveit::core::RobotModel, std::shared_ptr<moveit::core::RobotModel>>(m, "RobotModel")
        .def("getName", &moveit::core::RobotModel::getName,
             "Get the name of robot.")
        .def_property("JointModelGroups", static_cast
                              <const std::vector<moveit::core::JointModelGroup*>& (moveit::core::RobotModel::*)(void)>
                      (&moveit::core::RobotModel::getJointModelGroups), nullptr,
                      py::return_value_policy::reference_internal)
        .def("getJointModelGroup", static_cast
                     <moveit::core::JointModelGroup* (moveit::core::RobotModel::*)(const std::string&)>
             (&moveit::core::RobotModel::getJointModelGroup),
             py::return_value_policy::reference_internal,
             "Get a Joint Model Group specified by its name.")
                /**
                 * @warning Make sure to set return policy of getJointModelGroups to reference_internal (reference may also works).
                 * Otherwise, take_ownership would be used by default. Which means that the destructor would be called when
                 * the instance in python is unreferenced.
                 */
        .def("getJointModelGroups", static_cast
                     <const std::vector<moveit::core::JointModelGroup*>& (moveit::core::RobotModel::*)(void)>
             (&moveit::core::RobotModel::getJointModelGroups),
             py::return_value_policy::reference_internal,
             "Get The list of JointModel Group")
        .def("getModelFrame", &moveit::core::RobotModel::getModelFrame)
        .def("getJointModels", static_cast
             <const std::vector<moveit::core::JointModel*>& (moveit::core::RobotModel::*)(void)>
            (&moveit::core::RobotModel::getJointModels), py::return_value_policy::reference_internal)
        .def("getJointModel", static_cast
            <moveit::core::JointModel* (moveit::core::RobotModel::*)(const std::string&)>
            (&moveit::core::RobotModel::getJointModel), py::return_value_policy::reference_internal);
}

#endif //MOVEIT_NO_ROS_ROBOTMODEL_H
