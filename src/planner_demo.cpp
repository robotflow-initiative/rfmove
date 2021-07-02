//
// Created by yongxi on 6/22/21.
//
#include <planner/PlannerConfiguration.h>
#include <iostream>
#include <planner/PlannerManager.h>
#include <robot_model/RobotModelLoader.h>
#include <planner/PlannerConfiguration.h>

int main() {
    RobotModelLoaderPtr loader = createRobotModelLoaderFromFile("resources/panda_arm_hand.urdf",
                                                                "resources/panda_arm_hand.srdf");
    robot_model::RobotModelPtr kinematic_model = loader->getRobotModel();
    moveit::core::JointModelGroup* jmg = loader->getRobotModel()->getJointModelGroup("panda_arm");
    KinematicsLoaderPtr kinematics_loader = createKinematicsLoaderFromFile("resources/kinematics.yaml");
    loader->loadKinematicsSolvers(kinematics_loader);


    PlannerConfigurationPtr pconfig = createPlannerConfigurationFromFile("resources/ompl_planning.yaml");
    std::cout << pconfig->String() << std::endl;
    PlannerManager plannerManager(kinematic_model, pconfig);

}