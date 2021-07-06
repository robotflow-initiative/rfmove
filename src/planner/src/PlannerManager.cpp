//
// Created by yongxi on 2021/6/16.
//

#include "planner/PlannerManager.h"

PlannerManager::PlannerManager(const moveit::core::RobotModelConstPtr &robot_model,
                               const PlannerConfigurationConstPtr &config)
{
    std::cout << "PlannerManager Creation" << std::endl;
    std::cout << "Create constraint_sampler_manager" << std::endl;
    constraint_sampler_manager.reset(new constraint_samplers::ConstraintSamplerManager());
    std::cout << "Create context_manager" << std::endl;
    context_manager.reset(new ompl_interface::PlanningContextManager(robot_model, constraint_sampler_manager));
    std::cout << "Create constraints_library" << std::endl;
    constraints_library.reset(new ompl_interface::ConstraintsLibrary(*context_manager));
    // Make a copy of config so that we would not change the original one.
    planning_interface::PlannerConfigurationMap pconfig = config->getConfig();
    //std::cout << config->String() << std::endl;

    std::cout << "Construct default configuration" << std::endl;
    // Construct default configurations for planning groups that don't have configs yet.
    const std::vector<const moveit::core::JointModelGroup*>& groups = robot_model->getJointModelGroups();
    for(auto group : groups) {
        if(pconfig.find(group->getName()) == pconfig.end()) {
            std::cout << "Construct default configurations for " << group->getName() << std::endl;
            planning_interface::PlannerConfigurationSettings empty;
            empty.name = empty.group = group -> getName();
            pconfig[empty.name] = empty;
        }
    }
    context_manager->setPlannerConfigurations(pconfig);
}

planning_interface::PlanningContextPtr PlannerManager::getPlanningContext(
        const planning_scene::PlanningSceneConstPtr &planning_scene,
        const planning_interface::MotionPlanRequest &req) const {
    moveit_msgs::MoveItErrorCodes err;
    std::cout << "Call getPlanningContext" << std::endl;
    ompl_interface::ModelBasedPlanningContextPtr res = context_manager->getPlanningContext(planning_scene, req, err);
    if (err.val != err.SUCCESS) {
        ROS_ERROR_STREAM("Error when get planning context with code " << err.val);
    } else {
        // configure context
        // @todo: configure whether to use constraint approximation and simplified solution.
        std::cout << "Call setConstraintsApproximations" << std::endl;
        res->setConstraintsApproximations(constraints_library);
        res->simplifySolutions(true);
    }
    return res;
}