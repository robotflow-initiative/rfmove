//
// Created by yongxi on 2021/7/11.
//

#ifndef MOVEIT_NO_ROS_HARDWARE_PYBULLET_H
#define MOVEIT_NO_ROS_HARDWARE_PYBULLET_H

#include <pybind11/pybind11.h>
#include "controller/hardware.h"
#include "controller/trajectory.h"

namespace py=pybind11;


/// @todo Make HardwareInterface and JointHandler extendable in python.
class PybulletJointHandler : public JointHandler {
public:
    typedef py::detail::accessor<py::detail::accessor_policies::str_attr> PyAttr;
    PybulletJointHandler(int body_id_,
                         int joint_index,
                         const std::string& joint_name,
                         PyAttr getJointState,
                         PyAttr setJointMotorControl2);

    // implementation of JointHandler's virtual methods.
    ~PybulletJointHandler() override;
    double position() override;
    double velocity() override;
    bool setPosVel(const double& position, const double& velocity) override;
    bool setPosition(const double& position) override;
    bool setVelocity(const double& velocity) override;
    const std::string& name() override;

    static int POSITION_CONTROL_;   // declaration
    static int VELOCITY_CONTROL_;   // declaration
    static int TORQUE_CONTROL_;
                                    // Note that any cpp source file still need to define the static member variable
                                    // as here are only declarations.
private:
    PyAttr getJointState_;
    PyAttr setJointMotorControl2_;
    std::string name_;
    int joint_index_;
    int body_id_;
};

class PybulletHardware : public HardwareInterface {
public:
    static int GEOM_BOX_;

    typedef py::detail::accessor<py::detail::accessor_policies::str_attr> PyAttr;
    explicit PybulletHardware(py::handle pybullet, int bodyUniqueId);
    int getNumBodies();

    /**
     * Get the joint index in pybullet according to the joint_name.
     * @param joint_name
     * @return Index of joint if founc. -1 otherwise.
     */
    int getJointIndex(const std::string& joint_name);

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

    /**
     * get all joint states
     * @details This is not a HardwareInterface method. This is mainly used by help methods for pybullet.
     */
    void getJointStateAll(std::vector<double>& position, std::vector<double>& velocity);

    const std::vector<std::string>& getJointNames() {return joint_names_;};

    /**
     * Make the pybullet hardware stay in current position.
     */
    void stayCurrent();

    /**
     *
     */
    void free(int force = 200);

    void drawTrajectory(SplineTrajectoryPtr trajectory, double life_time = 10, double line_width = 3);

    // Implementation of HardwareInterface virtual interface.
    JointHandler * getJointHandler(const std::string &joint_name) override;
private:
    py::object pybullet_;
    PyAttr getNumBodies_;
    PyAttr getJointInfo_;
    PyAttr getNumJoints_;
    PyAttr getJointState_;
    PyAttr getJointStates_;
    PyAttr setJointMotorControl2_;
    PyAttr addUserDebugLine_;
    std::map<std::string, int> joint_index_map_; /// A map between joint name and joint index in bullet.
    int body_id_;
    int joint_num_;
    std::vector<std::string> joint_names_;
    std::vector<int> all_index_;
};

#endif //MOVEIT_NO_ROS_HARDWARE_PYBULLET_H
