//
// Created by yongxi on 2021/7/8.
//

#ifndef MOVEIT_NO_ROS_CONTROLLER_H
#define MOVEIT_NO_ROS_CONTROLLER_H

#include <pybind11/pybind11.h>
#include "controller/trajectory.h"

namespace py = pybind11;

SplineTrajectoryPtr computeSpline(robot_trajectory::RobotTrajectoryPtr robot_trajectory,
                                  bool computeTimeStamps = true,
                                  SplineTrajectory::Parameterization param = SplineTrajectory::Parameterization::SPLINE) {
    return std::make_shared<SplineTrajectory>(robot_trajectory, computeTimeStamps, param);
}

void declare_controller(py::module &m) {
    py::enum_<SplineTrajectory::Parameterization>(m, "TrajectoryParam")
            .value("SPLINE", SplineTrajectory::Parameterization::SPLINE)
            .value("TIME", SplineTrajectory::Parameterization::TIME);

    py::class_<SplineTrajectory, std::shared_ptr<SplineTrajectory>>(m, "SplineTrajectory")
        /// Do not create SplineTrajectory directly as hope to use shared pointer.
        /// Create it through computeSpline instead.
        //.def(py::init<robot_trajectory::RobotTrajectoryPtr,
        //             bool,
        //             SplineTrajectory::Parameterization>(),
        //     py::arg("trajectory"),
        //     py::arg("computeTimeStamp") = true,
        //     py::arg("param") = SplineTrajectory::Parameterization::SPLINE)
        .def_property("duration", &SplineTrajectory::duration, nullptr)
        .def_property("start_time", &SplineTrajectory::startTime, nullptr)
        .def_property("end_time", &SplineTrajectory::endTime, nullptr)
        .def("sample", [](SplineTrajectory& self, const std::string& joint_name, double interval) -> trajectory_interface::PosVelAccState<double>*{
            //std::cout << "sample " << joint_name << " with interval " << interval << std::endl;
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
            }, nullptr)
        .def_property("velocity", [](trajectory_interface::PosVelAccState<double>& self){
            return py::array(self.velocity.size(), self.velocity.data(), py::cast<>(self));
        }, nullptr);

    m.def("computeSpline", &computeSpline,
          py::arg("robot_trajectory"),
          py::arg("computeTimeStamps") = true,
          py::arg("param") = SplineTrajectory::Parameterization::SPLINE);
}

#endif //MOVEIT_NO_ROS_CONTROLLER_H
