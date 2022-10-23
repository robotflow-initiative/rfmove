//
// Created by yongxi on 2021/5/16.
//

// The demo running moveit_no_ros.

#include <iostream>
#include "moveit/rdf_loader/rdf_loader.h"
#include "moveit/robot_model/robot_model.h"
#include <fstream>
#include "moveit/robot_state/robot_state.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/ompl_interface/planning_context_manager.h"
#include "moveit/ompl_interface/model_based_planning_context.h"
#include "pluginlib/class_loader.h"
#include "moveit/planning_interface/planning_interface.h"
#include "moveit/kinematic_constraints/utils.h"
#include "yaml-cpp/yaml.h"
#include "moveit/kinematics_plugin_loader/kinematics_plugin_loader.h" // for kinematic solver plugin loader
#include "robot_model/RobotModelLoader.h"

std::string fileToString(std::ifstream& inStream) {
    std::ostringstream sstr;
    sstr << inStream.rdbuf();
    return sstr.str();
}

planning_interface::PlannerConfigurationMap loadYamlToCfg(const std::string& file_path) {
    YAML::Node yaml = YAML::LoadFile(file_path);
    YAML::Node planner_configs = yaml["planner_configs"];
    planning_interface::PlannerConfigurationMap plannerCfgMap;
    if (planner_configs.IsNull()) {
        ROS_FATAL_STREAM(file_path << " contains no 'planner_configs'");
    }
    for (YAML::const_iterator it = yaml.begin(); it != yaml.end(); ++it) {
        std::string group_name = it->first.as<std::string>();
        // std::cout << group_name << std::endl;
        if (group_name != "planner_configs") {
            // got a group planner_configs
            YAML::Node group_cfg_names = it->second["planner_configs"];

            assert(group_cfg_names.IsSequence());
            for (YAML::const_iterator group_cfg_name = group_cfg_names.begin(); group_cfg_name != group_cfg_names.end(); ++group_cfg_name) {
                if (group_cfg_name->IsNull()) {
                    ROS_WARN_STREAM("reach null node");
                    continue;
                }
                std::string group_cfg_name_string = group_cfg_name->as<std::string>();
                // std::cout << '\t' << group_cfg_name_string << std::endl;
                // Build setting_config
                std::map<std::string, std::string> setting_cfg;
                YAML::Node planner_config = planner_configs[group_cfg_name_string];
                for(YAML::const_iterator setting_cfg_it = planner_config.begin(); setting_cfg_it != planner_config.end(); ++setting_cfg_it) {
                    setting_cfg[setting_cfg_it->first.as<std::string>()] = setting_cfg_it->second.as<std::string>();
                }

                // Build PlannerConfigurationSettings
                std::ostringstream ss;
                ss << group_name << "[" << group_cfg_name_string << "]";
                std::string setting_name = ss.str();
                plannerCfgMap[setting_name] = planning_interface::PlannerConfigurationSettings {
                        .group =  group_name,
                        .name = setting_name,
                        .config = setting_cfg,
                };
            }

            // check existence of default setting

            //YAML::Node default_setting = it->second[group_name];
            //if (default_setting.IsNull()) {
                ROS_INFO_STREAM("Create default planner configuration for " << group_name);
                plannerCfgMap[group_name] = planning_interface::PlannerConfigurationSettings {
                    .group = group_name,
                    .name = group_name,
                    .config = std::map<std::string, std::string> {
                            {"type", "geometric::RRTConnect"},
                    },
                };
            //} else {
            //    ROS_INFO_STREAM("Load default planner configuration for " << group_name);
                // TODO: We do not know the structure of default setting.
            //}
        }
    }
    return plannerCfgMap;
}

int main(int argc, char *argv[]) {
    //ROS_LOG()

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    // Planner relay on ros node for now.
    // ros::init(argc, argv, "master");

    // Try to load rdf from file
    std::string urdfFilePath = "resources/panda_arm_hand.urdf";
    std::string srdfFilePath = "resources/panda_arm_hand.srdf";

    std::ifstream inStreamUrdf(urdfFilePath);
    //std::string urdfContains = fileToString(inStreamUrdf);
    //ROS_INFO_STREAM("URDF Contents in demo\n" << urdfContains);


    std::ifstream inStreamSrdf(srdfFilePath);
    //std::string srdfContains = fileToString(inStreamSrdf);
    //ROS_INFO_STREAM("SRDF Contents in demo\n" << srdfContains);

    RobotModelLoaderPtr robotModelLoaderNoros(new RobotModelLoader(inStreamUrdf, inStreamSrdf));

    inStreamUrdf.close();
    inStreamSrdf.close();

    //rdf_loader::RDFLoader rdfLoader(urdfContains, srdfContains);

    // Create robot model
    // moveit::core::RobotModel robotModel(rdfLoader.getURDF(), rdfLoader.getSRDF());
    //moveit::core::RobotModelPtr robotModelPtr(new moveit::core::RobotModel(rdfLoader.getURDF(), rdfLoader.getSRDF()));
    // robotModelPtr.reset(&robotModel);
    moveit::core::RobotModelConstPtr robotModel = robotModelLoaderNoros->getRobotModel();

    // List Joint Model
    const std::vector<const moveit::core::JointModel*>& jointModels = robotModel -> getJointModels();
    std::cout << "JointModels:" << std::endl;
    for (auto jointModel : jointModels) {
        std::cout << "\tName: " << jointModel->getName() << "\tType: " << jointModel->getTypeName() << std::endl;
    }

    // List Joint ModelGroup
    const auto& jointModelGroups = robotModel->getJointModelGroups();
    std::cout << "JointModelGroups:" << std::endl;
    for (const auto& jointModel : jointModelGroups) {
        std::cout << "\tName: " << jointModel->getName() << std::endl;
        for (const auto& jointModelVar : jointModel->getVariableNames()) {
            std::cout << "\t\tVariable: " << jointModelVar << std::endl;
        }
    }

    const std::string PLANNING_GROUP = "panda_arm";
    std::cout << "Use group " << PLANNING_GROUP << " for planning." << std::endl;


    // const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    // Create robot state
    moveit::core::RobotStatePtr robotStatePtr( new robot_state::RobotState(robotModel));

    // Check kinematic solver
    // Debug: load kinematic solver from model
    const robot_state::JointModelGroup* joint_model_group = robotStatePtr->getJointModelGroup(PLANNING_GROUP);
    if (joint_model_group->getGroupKinematics().first) {
        ROS_INFO_STREAM("Joint model group " << PLANNING_GROUP << " have kinematic solver");
    } else {
        ROS_WARN_STREAM("Joint model group " << PLANNING_GROUP << " have no kinematic solver");
    }
    // Create KinematicsPluginLoader
    // const kinematics_plugin_loader::KinematicsPluginLoaderPtr kinematicsPluginLoaderPtr(new kinematics_plugin_loader::KinematicsPluginLoader(rdfLoader.getRobotDescription()));

    // List state variables
    auto stateVariables = robotModel->getVariableNames();
    // const auto& stateVariables = robotStatePtr;
    std::cout << "State Variables:" << std::endl;
    for (const auto& stateVariable : stateVariables) {
        std::cout << '\t' << stateVariable << std::endl;
    }

    // Create PlanningScene
    planning_scene::PlanningScenePtr planningScenePtr(new planning_scene::PlanningScene(robotModel));
    std::cout << "PlanningScene Name: " << planningScenePtr->getName() << std::endl;
    // Configure a valid robot state.
    // Planning scene must have a valid current state.
    // "ready" is a state defined in srdf file.
    // In real scenario, that should be the current state of robot.
    planningScenePtr->getCurrentStateNonConst().setToDefaultValues(robotStatePtr->getJointModelGroup(PLANNING_GROUP), "ready");

    // Load Planner Configuration
    // ompl_interface::ModelBasedPlanningContextPtr planningContextPtr(new ompl_interface::ModelBasedPlanningContext);
    const std::string planner_cfg_path = "resources/ompl_planning.yaml";
    planning_interface::PlannerConfigurationMap plannerConfigurationMap = loadYamlToCfg(planner_cfg_path);
    std::cout << "Planner Configuration: " << std::endl;
    for(const auto & cfg_setting : plannerConfigurationMap) {
        std::cout << '\t' << cfg_setting.first << ":\t" << cfg_setting.second.group << '-' << cfg_setting.second.name << std::endl;
        for(const auto &cfg : cfg_setting.second.config) {
            std::cout << "\t\t" << cfg.first << ":\t" << cfg.second << std::endl;
        }
    }

    // Create ConstraintSampler
    constraint_samplers::ConstraintSamplerManagerPtr constraintSamplerManagerPtr(new constraint_samplers::ConstraintSamplerManager());


    // Create Planner
    ompl_interface::PlanningContextManager planningContextManager(robotModel, constraintSamplerManagerPtr);
    // Load configuration for context manager. Same as `OMPLInterface::loadPlannerConfigurations`
    planningContextManager.setPlannerConfigurations(plannerConfigurationMap);
    // Load ConstraintSampler for context manager. Same as `OMPLInterface::loadConstraintSamplers`
    //ompl_interface::ConstraintsLibrary* constraintsLibrary = new ompl_interface::ConstraintsLibrary();
    // ERROR
    // ConstraintsLibrary is defined within ompl_interface.cpp
    // We need an outer class to load constraints for us.
    // ompl_interface::ConstraintsLibraryPtr constraintsLibraryPtr(new ompl_interface::ConstraintsLibrary(planningContextManager));

    // Create MotionRequest
    planning_interface::MotionPlanRequest request;

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "panda_link0";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.4;
    pose.pose.position.z = 0.75;
    pose.pose.orientation.w = 1.0;
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);
    moveit_msgs::Constraints pose_goal =
            kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);
    request.group_name = PLANNING_GROUP;
    request.goal_constraints.push_back(pose_goal);

    // Try to add request.start_state


    // Debug: get start state from planning scene
    std::cout << "Try to get start state from planning scene." << std::endl;
    robot_state::RobotStatePtr start_state = planningScenePtr->getCurrentStateUpdated(request.start_state);

    // Create Planning Context
    std::cout << "Create Planning Context" << std::endl;
    moveit_msgs::MoveItErrorCodes errorCode;
    ompl_interface::ModelBasedPlanningContextPtr context = planningContextManager.getPlanningContext(planningScenePtr, request, errorCode);
    if (errorCode.val == errorCode.SUCCESS) {
        std::cout << "Context creation success." << std::endl;
    } else {
        ROS_FATAL_STREAM("Context creation fail with code " << errorCode.val);
    }

    // Load planner as plugin
    /*
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> plugin_loader;
    planning_interface::PlannerManagerPtr plannerManager;
    try {
        plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core",
                                                                                         "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex) {
        ROS_ERROR_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    try {
        if (plugin_loader == nullptr) {
            ROS_FATAL("Plugin Loader is null");
        } else {
            std::cout << "Planner Plugin Loader DeclaredClasses:" << std::endl;
            for (const auto& decle : plugin_loader->getDeclaredClasses()) {
                std::cout << "\t" << decle << std::endl;
            }
            plannerManager.reset(plugin_loader->createUnmanagedInstance("ompl_interface/OMPLPlanner"));
        }
    }
    catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while Create OMPL instance " << ex.what());
    }

    plannerManager->initialize(robotModelPtr, "");
    */

    std::cin.get();
    return 0;
}
