//
// Created by yongxi on 6/21/21.
//

#ifndef MOVEIT_NO_ROS_PLANNERMANAGER_H
#define MOVEIT_NO_ROS_PLANNERMANAGER_H

#include <pybind11/pybind11.h>
#include <planner/PlannerConfiguration.h>
#include <planner/PlannerManager.h>

namespace py = pybind11;

void declare_planner_manager(py::module& m) {
    py::class_<PlannerConfiguration, std::shared_ptr<PlannerConfiguration>>(m, "PlannerConfiguration")
        .def(py::init<const std::string&>())
        .def("__str__", &PlannerConfiguration::String)
        .def("__repr__", &PlannerConfiguration::String);

    py::class_<PlannerManager, std::shared_ptr<PlannerManager>>(m, "PlannerManager")
        .def(py::init<const moveit::core::RobotModelConstPtr&, const PlannerConfigurationConstPtr&>())
        .def("getPlanningContext", &PlannerManager::getPlanningContext);

    m.def("createPlannerConfigurationFromFile", &createPlannerConfigurationFromFile,
          py::arg("file_path"));
}

#endif //MOVEIT_NO_ROS_PLANNERMANAGER_H
