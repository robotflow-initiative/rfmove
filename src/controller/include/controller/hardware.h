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
 */
class JointHandler {
public:
    /**
     * @details Always mark destructor virtual.
     */
    virtual ~JointHandler() = 0;
    virtual double position() = 0;
    virtual double velocity() = 0;
    virtual bool setPosVel(const double& position, const double& velocity) = 0;
    virtual bool setPosition(const double& position) = 0;
    virtual bool setVelocity(const double& velocity) = 0;
    virtual const std::string& name() = 0;
};

/**
 * Hardware interface for a group of joints.
 * @details The easiest way to implement JointGroupHandler is a list of JointHandler.
 * Such JointGroupHandler is implemented by DefaultJointGroupHandler.
 * However, specific hardware may implement its own JointGroupHandler with more efficient methods.
 */
class JointGroupHandler {
public:
    /**
     * Always mark the base classes' destructors virtual.
     */
    virtual ~JointGroupHandler() = 0;
    virtual void positions(std::vector<double> positions) = 0;
    virtual void velocities(std::vector<double> velocities) = 0;
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
private:
    std::vector<JointHandler*> handlers_;
};

class HardwareInterface {
public:
    virtual JointHandler* getJointHandler(const std::string& joint_name) = 0;
    //virtual int getJointHandler(const std::vector<const std::string>& joint_names, std::vector<JointHandler*>& result) = 0;
    /**
     * @details Mark this function as 'virtual' so that the inherited class will use their own implementation if exists.
     * @param joint_names
     * @return
     */
    virtual JointGroupHandler* getJointGroupHandler(const std::vector<std::string>& joint_names);
};

#endif //MOVEIT_NO_ROS_HARDWARE_H
