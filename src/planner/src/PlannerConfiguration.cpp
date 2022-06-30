//
// Created by yongxi on 6/21/21.
//

#include <planner/PlannerConfiguration.h>
#include <string>
#include <path_util.h>
#include <fstream>
#include <sstream>

PlannerConfiguration::PlannerConfiguration(const std::string& config_contents) {
    YAML::Node yaml = YAML::Load(config_contents);
    YAML::Node planner_configs = yaml["planner_configs"];
    //auto* plannerCfgMap = new planning_interface::PlannerConfigurationMap();
    if (planner_configs.IsNull()) {
        ROS_ERROR("Planner configuration contains no 'planner_configs'");
        return;
    }
    for (YAML::const_iterator it = yaml.begin(); it != yaml.end(); ++it) {
        std::string group_name = it->first.as<std::string>();
        // std::cout << group_name << std::endl;
        if (group_name != "planner_configs" && group_name != "constraint_approximations_path") {
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
                configMap[setting_name] = planning_interface::PlannerConfigurationSettings {
                        .group =  group_name,
                        .name = setting_name,
                        .config = setting_cfg,
                };
            }

            // check existence of default setting

            //YAML::Node default_setting = it->second[group_name];
            //if (default_setting.IsNull()) {
            ROS_INFO_STREAM("Create default planner configuration for " << group_name);
            configMap[group_name] = planning_interface::PlannerConfigurationSettings {
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
}

std::string PlannerConfiguration::String() const {
    std::ostringstream sstr;
    for(const auto& config : configMap) {
        sstr << config.first << ":\n";
        sstr << "\tname: " << config.second.name << '\n';
        sstr << "\tgroup: " << config.second.group << '\n';
        sstr << "\tsetting:\n";
        for(const auto& setting : config.second.config) {
            sstr << "\t\t" << setting.first << ": " << setting.second << '\n';
        }
    }
    return sstr.str();
}

PlannerConfigurationPtr createPlannerConfigurationFromFile(const std::string& config_path) {
    std::ifstream input(config_path);
    PlannerConfigurationPtr res(new PlannerConfiguration(ioToString(input)));
    input.close();
    return res;
}