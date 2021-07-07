//
// Created by yongxi on 2021/5/28.
//

#ifndef MOVEIT_NO_ROS_JOINTLIMITS_H
#define MOVEIT_NO_ROS_JOINTLIMITS_H

#include <moveit/robot_model/joint_model.h>
#include <moveit/macros/class_forward.h>
#include <iostream>

// TODO: Load joint limits for joint with multi variable

MOVEIT_CLASS_FORWARD(JointLimitsLoader)

/**
 * JointLimitsLoader load joint limit configuration from yaml contents.
 * It provides limit info corresponding to joint model name.
 */
class JointLimitsLoader {
public:
    explicit JointLimitsLoader(const std::string& limits_contents_);
    // moveit::core::VariableBounds getBound(const std::string& jointModel, const std::string& variable);
    const moveit::core::VariableBounds& getBound(const std::string& variable_name) const;
    std::map<std::string, moveit::core::VariableBounds>::const_iterator begin() const;
    std::map<std::string, moveit::core::VariableBounds>::const_iterator end() const;
    bool has(const std::string& variable_name) const;
    friend std::ostream& operator<< (std:: ostream& out, const JointLimitsLoader &loader);
    static moveit::core::VariableBounds mergeBounds(const moveit::core::VariableBounds& old_bound,
                                                    const moveit::core::VariableBounds& new_bound);
private:
    // limits_
    // variable_name: VariableBounds
    std::map<std::string, moveit::core::VariableBounds> limits_;
};

JointLimitsLoaderPtr createJointLimitsLoaderFromFile(const std::string& file_path);

#endif //MOVEIT_NO_ROS_JOINTLIMITS_H
