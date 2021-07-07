//
// Created by yongxi on 2021/5/28.
//

#include "robot_model/JointLimitsLoader.h"
#include "yaml-cpp/yaml.h"
#include "path_util.h"
#include <fstream>

JointLimitsLoader::JointLimitsLoader(const std::string &limits_contents_) {
    YAML::Node yaml = YAML::Load(limits_contents_);
    if(YAML::Node joint_limits = yaml["joint_limits"]) {
        for(auto joint_limit: joint_limits) {
            std::string joint_name = joint_limit.first.as<std::string>();
            limits_[joint_name] = moveit::core::VariableBounds{};
            if(joint_limit.second["has_position_limits"]) {
                if(joint_limit.second["has_position_limits"].as<bool>()) {
                    limits_[joint_name].position_bounded_ = true;
                    if(joint_limit.second["min_position"]) {
                        limits_[joint_name].min_position_ = joint_limit.second["min_position"].as<double>();
                    }
                    if(joint_limit.second["max_position"]) {
                        limits_[joint_name].max_position_ = joint_limit.second["max_position"].as<double>();
                    }
                }
            }
            if(joint_limit.second["has_velocity_limits"]) {
                if(joint_limit.second["has_velocity_limits"].as<bool>()) {
                    limits_[joint_name].velocity_bounded_ = true;
                    if(joint_limit.second["max_velocity"]) {
                        limits_[joint_name].max_velocity_ = joint_limit.second["max_velocity"].as<double>();
                        if(joint_limit.second["min_velocity"]) {
                            limits_[joint_name].min_velocity_ = joint_limit.second["min_velocity"].as<double>();
                        } else {
                            limits_[joint_name].min_velocity_ = -joint_limit.second["max_velocity"].as<double>();
                        }
                    }

                }
            }
            if(joint_limit.second["has_acceleration_limits"]) {
                if(joint_limit.second["has_acceleration_limits"].as<bool>()) {
                    limits_[joint_name].acceleration_bounded_ = true;
                    if(joint_limit.second["max_acceleration"]) {
                        limits_[joint_name].max_acceleration_ = joint_limit.second["max_acceleration"].as<double>();
                        if(joint_limit.second["min_acceleration"]) {
                            limits_[joint_name].min_acceleration_ = joint_limit.second["min_acceleration"].as<double>();
                        } else {
                            limits_[joint_name].min_acceleration_ = -joint_limit.second["max_acceleration"].as<double>();
                        }
                    }
                }
            }
        }
    }
}

const moveit::core::VariableBounds& JointLimitsLoader::getBound(const std::string &variable_name) const {
    //return limits_[variable_name];
    return limits_.at(variable_name);
}

std::map<std::string, moveit::core::VariableBounds>::const_iterator JointLimitsLoader::begin() const {
    return limits_.begin();
}

std::map<std::string, moveit::core::VariableBounds>::const_iterator JointLimitsLoader::end() const {
    return limits_.end();
}

bool JointLimitsLoader::has(const std::string& variable_name) const {
    return limits_.find(variable_name) != limits_.end();
}

JointLimitsLoaderPtr createJointLimitsLoaderFromFile(const std::string& file_path){
    std::ifstream input(file_path);
    std::string yaml_content = ioToString(input);
    input.close();
    // return std::make_shared<KinematicsLoader>(yaml_content);
    return JointLimitsLoaderPtr(new JointLimitsLoader(yaml_content));
}

moveit::core::VariableBounds JointLimitsLoader::mergeBounds(const moveit::core::VariableBounds &old_bound,
                                                            const moveit::core::VariableBounds &new_bound) {
    moveit::core::VariableBounds res = old_bound;
    if (new_bound.position_bounded_) {
        res.position_bounded_ = true;
        res.max_position_ = new_bound.max_position_;
        res.min_position_ = new_bound.min_position_;
    }
    if (new_bound.velocity_bounded_) {
        res.velocity_bounded_ = true;
        res.max_velocity_ = new_bound.max_velocity_;
        res.min_velocity_ = new_bound.min_velocity_;
    }
    if (new_bound.acceleration_bounded_) {
        res.acceleration_bounded_ = true;
        res.min_acceleration_ = new_bound.min_acceleration_;
        res.max_acceleration_ = new_bound.max_acceleration_;
    }
    return res;
}

std::ostream& operator<<(std::ostream& out, const JointLimitsLoader& loader) {
    for (const auto& limit : loader.limits_) {
        out << limit.first << ": " << limit.second << std::endl;
    }
}