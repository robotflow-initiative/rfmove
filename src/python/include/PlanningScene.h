//
// Created by yongxi on 2021/6/2.
//

#ifndef MOVEIT_NO_ROS_PLANNINGSCENE_H
#define MOVEIT_NO_ROS_PLANNINGSCENE_H

#include "moveit/planning_scene/planning_scene.h"
//#include "moveit/collision_detection/world.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;

void declare_planning_scene(py::module &m) {
    py::class_<collision_detection::World, std::shared_ptr<collision_detection::World>>(m, "World");
    py::class_<planning_scene::PlanningScene, std::shared_ptr<planning_scene::PlanningScene>>(m, "PlanningScene")
        /// @warning Do not use const version of checkSelfCollision unless robot_state in planningScene is unchangeable.
        .def("checkSelfCollision", static_cast
            <void(planning_scene::PlanningScene::*)(const collision_detection::CollisionRequest&, collision_detection::CollisionResult&)>
            (&planning_scene::PlanningScene::checkSelfCollision),
            "Check whether the current state is in self collision")
        /**
         * @warning getCurrentStateNonConst return a reference insteade of Ptr.
         * So we have to set the return policy to reference_internal
         */
        .def("getCurrentStateNonConst", &planning_scene::PlanningScene::getCurrentStateNonConst, py::return_value_policy::reference_internal);

    py::class_<collision_detection::CollisionRequest>(m, "CollisionRequest") // We do not use smart pointer of CollisionRequest
        .def(py::init<>())
        .def_readwrite("distance", &collision_detection::CollisionRequest::distance,
                       "If true, compute proximity distance.")
        .def_readwrite("cost", &collision_detection::CollisionRequest::cost,
                       "If true, a collision cost is computed.")
        .def_readwrite("contacts", &collision_detection::CollisionRequest::contacts,
                       "If true, compute contacts.")
        .def_readwrite("max_contacts", &collision_detection::CollisionRequest::max_contacts,
                       "Overall maximum number of contacts to compute.")
        .def_readwrite("max_contacts_per_pair", &collision_detection::CollisionRequest::max_contacts_per_pair,
                       "Maximum number of contacts to compute per pair of bodies (multiple bodies may be in contact "
                       "at different configurations)")
        .def_readwrite("max_cost_sources", &collision_detection::CollisionRequest::max_cost_sources,
                       "When costs are computed, this value defines how many of the top cost sources should be returned.")
        .def_readwrite("min_cost_density", &collision_detection::CollisionRequest::min_cost_density,
                       "When costs are computed, this is the minimum cost density for a CostSource to be included in the results.")
        .def_readwrite("verbose", &collision_detection::CollisionRequest::verbose,
                       "Flag indicating whether information about detected collisions should be reported")
        .def_readwrite("group_name", &collision_detection::CollisionRequest::group_name);

    py::class_<collision_detection::CollisionResult>(m, "CollisionResult")
        .def(py::init<>())
        .def("clear", &collision_detection::CollisionResult::clear)
        .def_readonly("collision", &collision_detection::CollisionResult::collision)
        .def_readonly("contacts", &collision_detection::CollisionResult::contacts);

    py::class_<collision_detection::Contact>(m, "Contact");
}

#endif //MOVEIT_NO_ROS_PLANNINGSCENE_H
