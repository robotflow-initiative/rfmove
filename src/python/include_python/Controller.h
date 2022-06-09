//
// Created by yongxi on 2021/7/8.
//

#ifndef MOVEIT_NO_ROS_TRAJECTORY_CONTROLLER_H
#define MOVEIT_NO_ROS_CONTROLLER_H

#include <pybind11/pybind11.h>
#include "controller/trajectory.h"
#include "controller/trajectory_controller.h"

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
        
        .def("sample_by_interval", [](SplineTrajectory& self, const std::string& joint_name, double interval)
            -> trajectory_interface::PosVelAccState<double>* {
                //std::cout << "sample " << joint_name << " with interval " << interval << std::endl;
                auto* result = new trajectory_interface::PosVelAccState<double>;
                self.sample_by_interval(joint_name, *result, interval);
                return result;
            },
            py::arg("joint_name"),
            py::arg("interval"))
        .def("sample_at_time", [](SplineTrajectory& self, double time_point)
            -> trajectory_interface::PosVelAccState<double>* {
                auto* result = new trajectory_interface::PosVelAccState<double>;
                self.sample_at_time(*result, time_point);
                return result;
            },
            py::arg("time_point"))
        .def_property("tip_transforms", [](SplineTrajectory& self) {
            auto transforms = new std::vector<Eigen::Affine3d>();
            self.getTipTransforms(*transforms);
            return transforms;
            //auto capsule = py::capsule(transforms, [](void *transforms){
            //   delete reinterpret_cast<std::vector<Eigen::Affine3d>*>(transforms);
            //});
            //return py::array(transforms->size(), transforms->data(), capsule);
        }, nullptr)
        .def_property("joint_names", &SplineTrajectory::getJointNames, nullptr);

    py::class_<trajectory_interface::PosVelAccState<double>,
            std::unique_ptr<trajectory_interface::PosVelAccState<double>>>(m, "PosVelAccState")
        .def("__repr__", [](trajectory_interface::PosVelAccState<double>& self) {
            std::ostringstream sstr;
            sstr << "<trajectory_interface::PosVelAccState<double>, SegmentState>" << std::endl;
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
        }, nullptr)
        .def_property("acceleration",[](trajectory_interface::PosVelAccState<double>& self){
            return py::array(self.acceleration.size(), self.acceleration.data(), py::cast<>(self));
        },nullptr);

    m.def("computeSpline", &computeSpline,
          py::arg("robot_trajectory"),
          py::arg("computeTimeStamps") = true,
          py::arg("param") = SplineTrajectory::Parameterization::SPLINE);

    py::class_<Controller>(m, "Controller")
        .def(py::init<HardwareInterface*, double>(),
            py::arg("hardware"),
            py::arg("interval") = 0.01)
        .def("__repr__", [](Controller& self){
            std::ostringstream sstr;
            sstr << "<Controller at " << &self << '>';
            return sstr.str();
        })
        .def("execute", static_cast<bool(Controller::*)(SplineTrajectoryPtr)>
            (&Controller::executeTrajectory),
            "Execute spline trajectory",
            py::arg("spline_trajectory"))
        .def("execute", static_cast<bool(Controller::*)
            (robot_trajectory::RobotTrajectoryPtr, bool, SplineTrajectory::Parameterization)>
            (&Controller::executeTrajectory));
}

#endif //MOVEIT_NO_ROS_TRAJECTORY_CONTROLLER_H
