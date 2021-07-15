/**
 * @details Some help methods for pybullet. However, many of these methods can be implemented as member function
 * of PybulletHardware. But I do not hope that the hardware itself depends on moveit.
 * So these methods are implemented as helper functions with PybulletHardware as parameter.
 */

#ifndef MOVEIT_NO_ROS_BULLET3_HELPER_H
#define MOVEIT_NO_ROS_BULLET3_HELPER_H

#include <hardware_pybullet.h>

/**
 * Synchronize robot state between moveit and pybullet.
 * @details Walk through all joints from pybullet hardware and set the joint position in robot state the same
 * as pybullet.
 * @param pybullet
 * @param robot_state
 */
void syncBulletState(PybulletHardware& pybullet, planning_scene::PlanningScenePtr planningScene) {
    moveit::core::RobotState& robot_state = planningScene->getCurrentStateNonConst();
    std::vector<double> position;
    std::vector<double> velocity;
    pybullet.getJointStateAll(position, velocity);
    size_t index = 0;
    for(const auto& name: pybullet.getJointNames()) {
        /// @todo Support multi dof joint
        robot_state.setJointPositions(name, &position[index]);
        // Velocity is not synchronized.
        ++index;
    }
}

void declare_pybullet_helper(py::module& m) {
    m.def("syncBulletState", &syncBulletState);
}

#endif //MOVEIT_NO_ROS_BULLET3_HELPER_H
