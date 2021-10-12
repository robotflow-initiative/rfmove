#ifndef MOVEIT_NO_ROS_SPLINE_PY_H
#define MOVEIT_NO_ROS_SPLINE_PY_H

#include <planner/rfWaypoint.hpp>
#include <planner/PlannerSpline.hpp>
#include <planner/rfWaypoint.hpp>
#include <pybind11/pybind11.h>

namespace py=pybind11;

void declare_rfSpline(py::module& m){

    py::class_<rfWaypoint>(m,"rfWaypoint")
        .def(py::init<const std::array<double,3>&,const std::array<double,3>&>());
        
    py::class_<PlannerSpline>(m,"PlannerSpline")
        .def(py::init<std::string&>())
        .def("loadRobotModel",&PlannerSpline::loadRobotModel)
        .def("loadKinematicModel",&PlannerSpline::loadKinematicModel)
        .def("loadPlannerConfig",&PlannerSpline::loadPlannerConfig)
        .def("loadJointLimit",&PlannerSpline::loadJointLimit)
        .def("InitRobotState",static_cast<void (PlannerSpline::*)(std::vector<double>&,std::string&)>(&PlannerSpline::InitRobotState),"Set RotbotState by Waypoint")
        .def("InitRobotState",static_cast<void (PlannerSpline::*)(rfWaypoint&)>(&PlannerSpline::InitRobotState),"Set RotbotState by Waypoint")
        .def("CreateSplineParameterization",&PlannerSpline::CreateSplineParameterization,py::arg("watpoint"),py::arg("group_name"),py::arg("timeinterval")=0.1)
        .def("getTimeList",&PlannerSpline::getTimeList)
        .def("getJointPosValue",&PlannerSpline::getJointPosValue)
        .def("getJointVecValue",&PlannerSpline::getJointVecValue)
        .def("getJointAclValue",&PlannerSpline::getJointAclValue)
        .def("computeSpline",static_cast<bool (PlannerSpline::*)(double&,std::string&)>(&PlannerSpline::computeSpline),"compute and sample pos velocity acceleration")
        //.def("computeSpline",static_cast<tk::spline (PlannerSpline::*)(std::vector<double>&,std::vector<double>&>))(&PlannerSpline::computeSpline),"get pos velocity acceleration tk::spline")
        .def("getwaypointPositionList",&PlannerSpline::getwaypointPositionList)
        .def("getwaypointVelocityList",&PlannerSpline::getwaypointVelocityList)
        .def("getwaypointAccelerationList",&PlannerSpline::getwaypointAccelerationList)
        .def("getSampletimestamp",&PlannerSpline::getSampletimestamp)
        .def("sample_by_interval",&PlannerSpline::sample_by_interval)
        .def("get_ompl_sample",&PlannerSpline::get_ompl_sample)
        .def("get_sample_by_interval_times",&PlannerSpline::get_sample_by_interval_times)
        .def("__repr__",
            [](const PlannerSpline& ps){
                return "<object for jointlimit loader>";
            });
}

#endif