//
// Created by yongxi on 2021/6/16.
//

#ifndef PLANNER_NO_ROS_PLANNERMANAGER_H
#define PLANNER_NO_ROS_PLANNERMANAGER_H

#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/robot_model/robot_model.h>
#include <planner/PlannerConfiguration.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/ompl_interface/constraints_library.h>
#include <moveit/planning_interface/planning_interface.h>

/**
 * Implementation of OMPLInterface without ros.
 * @details It can be seen as a wrapper of PlanningContextManager.
 */
class PlannerManager {
public:
    PlannerManager(const moveit::core::RobotModelConstPtr& robot_model, const PlannerConfigurationConstPtr& config);
    //loadPlannerConfigurations
    //loadConstraintApproximations
    //loadConstraintSamplers
    planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                    const planning_interface::MotionPlanRequest& req) const;
private:
    std::shared_ptr<ompl_interface::PlanningContextManager> context_manager;
    constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager;
    ompl_interface::ConstraintsLibraryPtr constraints_library;
};

#endif //PLANNER_NO_ROS_PLANNERMANAGER_H
