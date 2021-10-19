//
// Created by yongxi on 6/22/21.
//

#ifndef MOVEIT_NO_ROS_PLANNINGCONTEXT_H
#define MOVEIT_NO_ROS_PLANNINGCONTEXT_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <sstream>

planning_interface::MotionPlanRequest createMotionPlanRequest(const std::string& group_name) {
    planning_interface::MotionPlanRequest req;
    req.group_name = group_name;
    return req;
}

void declare_planning_context(py::module& m) {
    //py::class_<planning_interface::RobotTra>
    py::class_<planning_interface::MotionPlanResponse>(m, "MotionPlanResponse")
        .def_readonly("trajectory", &planning_interface::MotionPlanResponse::trajectory_)
        .def_readonly("planning_time", &planning_interface::MotionPlanResponse::planning_time_);

    py::class_<planning_interface::MotionPlanRequest>(m, "MotionPlanRequest")
        .def(py::init(&createMotionPlanRequest))
        .def("__str__", [](planning_interface::MotionPlanRequest& self){
            std::ostringstream sstr;
            sstr << self;
            return sstr.str();
        })
        .def("__repr__", [](planning_interface::MotionPlanRequest& self) {
            std::ostringstream sstr;
            sstr << "<planning_interface::MotionPlanRequest>\n";
            sstr << self;
            return sstr.str();
        })
        .def_readwrite("group_name", &planning_interface::MotionPlanRequest::group_name)
        .def_readwrite("goal_constraints", &planning_interface::MotionPlanRequest::goal_constraints)
        .def("addGoal", [](planning_interface::MotionPlanRequest& self, const moveit_msgs::Constraints& goal) {
            self.goal_constraints.push_back(goal);
        })
        .def_readwrite("max_velocity_scaling_factor",&planning_interface::MotionPlanRequest::max_velocity_scaling_factor)
        .def_readwrite("max_acceleration_scaling_factor",&planning_interface::MotionPlanRequest::max_acceleration_scaling_factor);

    py::class_<planning_interface::PlanningContext, std::shared_ptr<planning_interface::PlanningContext>>(m, "PlanningContext")
        .def("solve", [](planning_interface::PlanningContext &self) -> planning_interface::MotionPlanResponse {
            planning_interface::MotionPlanResponse res;
            self.solve(res);
            return res;
        });

    //m.def("constructGoalConstraints")
}

#endif //MOVEIT_NO_ROS_PLANNINGCONTEXT_H
