//
// Created by yongxi on 2021/6/16.
//

#ifndef PLANNER_NO_ROS_PLANNERCONFIGURATION_H
#define PLANNER_NO_ROS_PLANNERCONFIGURATION_H

#include <moveit/planning_interface/planning_interface.h>
#include <yaml-cpp/yaml.h>


MOVEIT_CLASS_FORWARD(PlannerConfiguration);

/**
 * A wrapper of planning_interface::PlannerConfigurationMap.
 * @detail Load planning_interface::PlannerConfigurationMap from yaml file.
 */
class PlannerConfiguration {
public:
    PlannerConfiguration() = default;
    explicit PlannerConfiguration(const std::string& config_contents);
    const planning_interface::PlannerConfigurationMap& getConfig() const {
        return configMap;
    };
    std::string String() const;
private:
    planning_interface::PlannerConfigurationMap configMap;
};

PlannerConfigurationPtr createPlannerConfigurationFromFile(const std::string& config_path);

#endif //PLANNER_NO_ROS_PLANNERCONFIGURATION_H
