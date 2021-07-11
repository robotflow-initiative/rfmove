/**
 * @file hardware.h
 * @brief Definition of hardware interface for controller.
 * @author Yongxi Huang
 * @date 2021-07-09
 */

#ifndef MOVEIT_NO_ROS_HARDWARE_H
#define MOVEIT_NO_ROS_HARDWARE_H

#include <string>

class HardwareInterface {
public:
    double getJointPosition(const std::string& joint_name);
    double getJointVelocity(const std::string& joint_name);
    void setJointPosition(const std::string& joint_name);
    void setJointVelocity(const std::string& joint_name);
};

class JointInterface {
public:
    virtual double position();
    virtual double velocity();
    virtual void setPosVel(const double& position, const double& velocity);
    virtual const std::string& name();
};

class JointGroupInterface {

};

#endif //MOVEIT_NO_ROS_HARDWARE_H
