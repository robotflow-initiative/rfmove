//
// Created by yongxi on 2021/7/8.
//

#ifndef MOVEIT_NO_ROS_CONTROLLER_H
#define MOVEIT_NO_ROS_CONTROLLER_H

#include <pybind11/pybind11.h>
#include "controller/trajectory.h"

namespace py = pybind11;

void declare_controller(py::module &m) {
    py::enum_<SplineTrajectory::Parameterization>(m, "TrajectoryParam")
            .value("SPLINE", SplineTrajectory::Parameterization::SPLINE)
            .value("TIME", SplineTrajectory::Parameterization::TIME);

    py::class_<SplineTrajectory>(m, "SplineTrajectory")
        .def(py::init<robot_trajectory::RobotTrajectoryPtr,
                     bool,
                     SplineTrajectory::Parameterization>(),
             py::arg("trajectory"),
             py::arg("computeTimeStamp") = true,
             py::arg("param") = SplineTrajectory::Parameterization::SPLINE)
        .def_property("duration", &SplineTrajectory::duration, nullptr)
        .def("sample", [](SplineTrajectory& self, const std::string& joint_name, double interval) -> trajectory_interface::PosVelAccState<double>*{
            auto* result = new trajectory_interface::PosVelAccState<double>;
            self.sample(joint_name, *result, interval);
            return result;
        });

    py::class_<trajectory_interface::PosVelAccState<double>,
            std::unique_ptr<trajectory_interface::PosVelAccState<double>>>(m, "SegmentState")
        .def("__repr__", [](trajectory_interface::PosVelAccState<double>& self) {
            std::ostringstream sstr;
            sstr << "<trajectory_interface::PosVecAccState<double>, SegmentState>" << std::endl;
            sstr << "Position:" << std::endl;
            sstr << '[';
            for(double pos : self.position) {
                sstr << ' ' << pos << ',' ;
            }
            sstr << ']' << std::endl;
            sstr << "Velocity:" << std::endl;
            sstr << '[';
            for(double pos : self.velocity) {
                sstr << ' ' << pos << ',' ;
            }
            sstr << ']' << std::endl;
            return sstr.str();
        })
        .def("__str__", [](trajectory_interface::PosVelAccState<double>&self){
            std::ostringstream sstr;
            sstr << "Position:" << std::endl;
            sstr << '[';
            for(double pos : self.position) {
                sstr << ' ' << pos << ',' ;
            }
            sstr << ']' << std::endl;
            sstr << "Velocity:" << std::endl;
            sstr << '[';
            for(double pos : self.velocity) {
                sstr << ' ' << pos << ',' ;
            }
            sstr << ']' << std::endl;
            return sstr.str();
        })
        .def_property("position", [](trajectory_interface::PosVelAccState<double> &self){
            return py::array(self.position.size(), self.position.data(), py::cast<>(self));
            }, nullptr);
}

#endif //MOVEIT_NO_ROS_CONTROLLER_H
