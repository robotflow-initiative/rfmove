#ifndef MOVEIT_NO_ROS_BULLET3_HELPER_H
#define MOVEIT_NO_ROS_BULLET3_HELPER_H

#include "helper_pybullet.h"

void declare_pybullet_helper(py::module& m) {
    py::class_<PlanningSceneHelper>(m, "PlanningSceneHelper")
        .def(py::init<PybulletHardware, planning_scene::PlanningScenePtr>())
        .def("sync", &PlanningSceneHelper::sync)
        .def("resync", &PlanningSceneHelper::resync)
        .def("reset", &PlanningSceneHelper::reset)
        .def("linkTransform", &PlanningSceneHelper::linkTransform,
             "Get the transformation affine matrix for specific link according to robot model frame.",
             py::arg("link_name"),
             py::arg("sync") = true)
        .def("linkRelativeTransform", &PlanningSceneHelper::linkRelativeTransform,
             "Get the transformation affine matrix for specific link according to specific frame.",
             py::arg("link_name"),
             py::arg("frame_id"),
             py::arg("sync") = true);
}

#endif //MOVEIT_NO_ROS_BULLET3_HELPER_H
