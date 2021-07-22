//
// Created by yongxi on 2021/7/11.
//

#ifndef MOVEIT_NO_ROS_BULLET3_HARDWARE_H
#define MOVEIT_NO_ROS_BULLET3_HARDWARE_H

#include <pybind11/pybind11.h>
#include <hardware_pybullet.h>

#include <utility>

/**
 * Class used to call pybullet within cpp.
 */


void declare_pybullet_controller(py::module& m) {

    /**
     * @warning It is not a good idea to use JointHandler directly as it is only a accessor to pybullet.
     * This binding is mainly for debugging.
     */
    py::class_<JointHandler>(m, "JointHandler")
        .def_property("position", &JointHandler::position, nullptr)
        .def_property("velocity", &JointHandler::velocity, nullptr)
        .def("setPosVel", &JointHandler::setPosVel)
        .def_property("joint_name", &JointHandler::name, nullptr);

    py::class_<JointGroupHandler>(m, "JointGroupHandler")
        .def_property("joint_names", [](JointGroupHandler &self){
                std::vector<std::string> joint_names;
                self.jointNames(joint_names);
                return joint_names;
            }, nullptr)
        .def_property("positions", [](JointGroupHandler &self){
                std::vector<double> positions;
                self.positions(positions);
                return positions;
            }, nullptr)
        .def_property("velocities", [](JointGroupHandler &self) {
                std::vector<double> velocities;
                self.velocities(velocities);
                return velocities;
            }, nullptr)
        .def("setPosVels", &JointGroupHandler::setPosVels,
            py::arg("positions"),
            py::arg("velocities"));

    py::class_<HardwareInterface> (m, "HardwareInterface")
        .def("getJointHandler", &HardwareInterface::getJointHandler,
             py::arg("joint_name"))
        .def("getJointGroupHandler", &HardwareInterface::getJointGroupHandler,
             py::arg("joint_names"));

    /**
     * @warning It is not a good idea to use PybulletHardware methods directly as they are only accessors to pybullet.
     * Use helper classes and controller instead.
     */
    py::class_<PybulletHardware, HardwareInterface>(m, "PybulletHardware")
        .def(py::init<py::handle, int>())
        .def("getNumBodies", &PybulletHardware::getNumBodies)
        .def("printJointInfo", &PybulletHardware::printJointInfo)
        .def("printJointState", &PybulletHardware::printJointState)
        .def("stayCurrent", &PybulletHardware::stayCurrent)
        .def("free", &PybulletHardware::free,
             py::arg("force") = 200)
        .def("getJointIndex", &PybulletHardware::getJointIndex,
            py::arg("joint_name"))
        .def("drawTrajectory", &PybulletHardware::drawTrajectory,
            py::arg("trajectory"),
            py::arg("life_time") = 10,
            py::arg("line_width") = 3);
        //.def("getJointHandler", &PybulletHardware::getJointHandler,
        //     py::arg("joint_name"));

    //m.def("BulletHardware", [](py::module bullet, int bodyId) -> HardwareInterface*{
    //   return  new PybulletHardware(std::move(bullet), bodyId);
    //});
}

#endif //MOVEIT_NO_ROS_BULLET3_HARDWARE_H
