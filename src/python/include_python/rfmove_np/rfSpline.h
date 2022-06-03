#ifndef MOVEIT_NO_ROS_SPLINE_PY_H
#define MOVEIT_NO_ROS_SPLINE_PY_H

#include <planner/rfWaypoint.hpp>
#include <planner/PlannerSpline.hpp>
#include <planner/rfWaypoint.hpp>
#include <pybind11/pybind11.h>
#include "moveit/planning_scene/planning_scene.h"

namespace py=pybind11;

void declare_rfSpline(py::module& m){

    py::class_<rfWaypoint>(m,"rfWaypoint")
        .def(py::init<const std::array<double,3>&,const std::array<double,3>&>());
        
    py::class_<PlannerSpline,std::shared_ptr<PlannerSpline>>(m,"PlannerSpline")
        .def(py::init<const std::string& >())
        .def("loadRobotModel",&PlannerSpline::loadRobotModel)
        .def("loadKinematicModel",&PlannerSpline::loadKinematicModel)
        .def("loadPlannerConfig",&PlannerSpline::loadPlannerConfig)
        .def("loadJointLimit",&PlannerSpline::loadJointLimit)
        .def("InitRobotState",static_cast<void (PlannerSpline::*)(std::vector<double>&,const std::string&)>(&PlannerSpline::InitRobotState),"Set RotbotState by Waypoint")
        .def("InitRobotState",static_cast<void (PlannerSpline::*)(rfWaypoint&)>(&PlannerSpline::InitRobotState),"Set RotbotState by Waypoint")

        .def("init",&PlannerSpline::init)

        .def("CreateSplineParameterization",
                                &PlannerSpline::CreateSplineParameterization,
                                py::arg("watpoints"),
                                py::arg("group_name"),
                                py::arg("frame_id"),
                                py::arg("tip_name"),
                                py::arg("timeinterval")=0.1,
                                py::arg("max_velocity_scaling_factor")=1.0,
                                py::arg("max_acceleration_scaling_factor")=1.0)

        .def("CreateSingleWaypointPath",
                                &PlannerSpline::CreateSingleWaypointPath,
                                py::arg("waypoint"),
                                py::arg("group_name"),
                                py::arg("frame_id"),
                                py::arg("tip_name"),
                                py::arg("max_velocity_scaling_factor")=1.0,
                                py::arg("max_acceleration_scaling_factor")=1.0)

        .def("getTimeList",&PlannerSpline::getTimeList)
        .def("getJointPosValue",&PlannerSpline::getJointPosValue)
        .def("getJointVecValue",&PlannerSpline::getJointVecValue)
        .def("getJointAclValue",&PlannerSpline::getJointAclValue)
        .def("sample_by_interval",&PlannerSpline::sample_by_interval)
        .def("get_ompl_sample",&PlannerSpline::get_ompl_sample)
        .def("get_sample_by_interval_times",&PlannerSpline::get_sample_by_interval_times)
        .def("AddCollectionObject",&PlannerSpline::AddObject,"AddCollectionObject_new")
        .def("RemoveObject",&PlannerSpline::RemoveObject,"Remove Object")
        .def("clearObjects",&PlannerSpline::clearObjects,"Clear Objects")
        .def("__repr__",
            [](const PlannerSpline& ps){
                return "<object for jointlimit loader>";
            });
}

#endif