//
// Created by yongxi on 2021/5/28.
//

#ifndef MOVEIT_NO_ROS_JOINTLIMITS_H
#define MOVEIT_NO_ROS_JOINTLIMITS_H

#include <moveit/robot_model/joint_model.h>

// TODO: Load joint limits for joint with multi variable

/*
 * JointLimitsLoader load joint limit configuration from yaml contents.
 * It provides limit info corresponding to joint model name.
 */
class JointLimitsLoader {
public:
    JointLimitsLoader(const std::string& limits_contents_);
    moveit::core::VariableBounds getBound(const std::string& jointModel, const std::string& variable);
private:
    // limits_
    // JointModel:
    //      Variable:
    //          VariableBound
    std::map<std::string, moveit::core::VariableBoundsMap> limits_;
};

#endif //MOVEIT_NO_ROS_JOINTLIMITS_H
