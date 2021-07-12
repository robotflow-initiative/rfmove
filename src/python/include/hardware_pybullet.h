//
// Created by yongxi on 2021/7/11.
//

#ifndef MOVEIT_NO_ROS_HARDWARE_PYBULLET_H
#define MOVEIT_NO_ROS_HARDWARE_PYBULLET_H

#include <pybind11/pybind11.h>
#include "controller/hardware.h"

namespace py=pybind11;

class PybulletJointHandler : public JointHandler {
public:
    typedef py::detail::accessor<py::detail::accessor_policies::str_attr> PyAttr;
    PybulletJointHandler(int body_id_, int joint_index, const std::string& joint_name, PyAttr getJointState);

    // implementation of JointHandler's virtual methods.
    ~PybulletJointHandler() override;
    double position() override;
    double velocity() override;
    bool setPosVel(const double& position, const double& velocity) override;
    bool setPosition(const double& position) override;
    bool setVelocity(const double& velocity) override;
    const std::string& name() override;
private:
    PyAttr getJointState_;
    std::string name_;
    int joint_index_;
    int body_id_;
};

class PybulletHardware : public HardwareInterface {
public:
    typedef py::detail::accessor<py::detail::accessor_policies::str_attr> PyAttr;
    explicit PybulletHardware(py::handle pybullet, int bodyUniqueId);
    int getNumBodies();
    /**
     * printout all joint info
     * @details Used for debugging.
     */
    void printJointInfo();

    /**
     * printout all joint state
     * @details Used for debuffing.
     */
     void printJointState();

    // Implementation of HardwareInterface virtual interface.
    JointHandler * getJointHandler(const std::string &joint_name) override;
private:
    py::object pybullet_;
    PyAttr getNumBodies_;
    PyAttr getJointInfo_;
    PyAttr getNumJoints_;
    PyAttr getJointState_;
    std::map<std::string, int> joint_index_map_; /// A map between joint name and joint index in bullet.
    int body_id_;
    int joint_num_;
};

#endif //MOVEIT_NO_ROS_HARDWARE_PYBULLET_H
