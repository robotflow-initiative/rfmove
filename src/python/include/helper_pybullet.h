/**
 * @details Some help methods for pybullet. However, many of these methods can be implemented as member function
 * of PybulletHardware. But I do not hope that the hardware itself depends on moveit.
 * So these methods are implemented as helper functions with PybulletHardware as parameter.
 */

#ifndef MOVEIT_NO_ROS_HELPER_PYBULLET_H
#define MOVEIT_NO_ROS_HELPER_PYBULLET_H

#include "hardware_pybullet.h"
#include "moveit/planning_scene/planning_scene.h"


class PlanningSceneHelper{
public:
    PlanningSceneHelper(PybulletHardware pybullet, planning_scene::PlanningScenePtr);

    /**
     * Synchronize robot state in moveit the same as pybullet.
     * @details Walk through all joints from pybullet hardware and set the joint position in robot state the same
     * as pybullet.
     */
    void sync();

    /**
     * Reverse version of sync. Synchronize robot state in pybullet the same as moveit.
     * @details Walk through all joints from pybullet hardware and change their position in pybullet according to
     * the joint position read from robot state.
     */
    void resync();

    /**
     * Get specific link transformation according to robot model frame.
     * @param link_name Name of link.
     * @param _sync Whether to sync robot state between pybullet and moveit. Default is true.
     * @return Transformation affine matrix.
     */
     Eigen::Affine3d linkTransform(const std::string& link_name, bool _sync = true);

     /**
      *
      * @param link_name
      * @param frame
      * @param _sync
      * @return
      */
     Eigen::Affine3d linkRelativeTransform(const std::string& link_name, const std::string& frame, bool _sync = true);

private:
    PybulletHardware pybullet_; // hardware only contains accessors.
                                // It is ok to keep an instance instead of an reference.
    planning_scene::PlanningScenePtr planning_scene_;
};

#endif //MOVEIT_NO_ROS_HELPER_PYBULLET_H
