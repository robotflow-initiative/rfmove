/**
 * @file hardware.h
 * @brief Definition of hardware interface for controller.
 * @author Yongxi Huang
 * @date 2021-07-09
 */

#ifndef MOVEIT_NO_ROS_HARDWARE_H
#define MOVEIT_NO_ROS_HARDWARE_H

#include <string>
#include <vector>
#include <map>

/**
 * Hardware interface for a single joint.
 * @details It can fetch joint state and send joint command.
 * @node Do not use pure virtual function as we still need to create JointHandler pointer.
 */
class JointHandler {
public:
    /**
     * @details Always mark destructor virtual.
     */
    virtual ~JointHandler() = default;
    virtual double position() {return 0;};
    virtual double velocity() {return 0;};
    virtual bool setPosVel(const double& position, const double& velocity) {return false;};
    virtual bool setPosition(const double& position) {return false;};
    virtual bool setVelocity(const double& velocity) {return false;};
    virtual const std::string& name() {return dummy_name_;} // Warning would occur without dummy_name_
private:
    std::string dummy_name_ = "dummy";
};

/**
 * Hardware interface for a group of joints.
 * @details JointGroupHandler can be used as handler for a moveit JointModelGroup.
 * The easiest way to implement JointGroupHandler is a list of JointHandler.
 * Such JointGroupHandler is implemented by DefaultJointGroupHandler.
 * However, specific hardware may implement its own JointGroupHandler with more efficient methods.
 */
class JointGroupHandler {
public:
    /**
     * Always mark the base classes' destructors virtual.
     */
    virtual ~JointGroupHandler() = default;
    virtual void positions(std::vector<double> positions) {positions.resize(0);}
    virtual void velocities(std::vector<double> velocities) {velocities.resize(0);}
    virtual int setPosVels(const std::vector<double>& positions, const std::vector<double>& velocities) = 0;
    virtual int setPositions(const std::vector<double>& positions) = 0;
    virtual int setVelocities(const std::vector<double>& velocities) = 0;
    virtual void jointNames(std::vector<std::string>& joint_names) = 0;
};

/**
 * Implementation of joint group handler by JointHandler vector.
 */
class DefaultJointGroupHandler : public JointGroupHandler{
public:
    explicit DefaultJointGroupHandler(std::vector<JointHandler*> handlers);
    ~DefaultJointGroupHandler() override;
    void positions(std::vector<double> _positions) override;
    void velocities(std::vector<double> _velocities) override;
    int setPosVels(const std::vector<double>& positions, const std::vector<double>& velocities) override;
    int setPositions(const std::vector<double>& positions) override;
    int setVelocities(const std::vector<double>& velocities) override;
    void jointNames(std::vector<std::string>& joint_names) override;
private:
    std::vector<JointHandler*> handlers_;
};

class HardwareInterface {
public:
    /**
     * Always mark the destructor of base class as virtual.
     */
    virtual ~HardwareInterface() = default;
    virtual JointHandler* getJointHandler(const std::string& joint_name) = 0;

    /**
     * @details Mark this function as 'virtual' so that the inherited class will use their own implementation if exists.
     * We also provide a default implementation of getJointGroupHandler use DefaultJointGroupHandler, which means that
     * it is ok that inherited class only implement getJointHandler.
     * @param joint_names
     * @return JointGroupHandler
     */
    virtual JointGroupHandler* getJointGroupHandler(const std::vector<std::string>& joint_names);
};

#endif //MOVEIT_NO_ROS_HARDWARE_H
