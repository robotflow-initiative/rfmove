/**
 * @file RobotModelLoader.h
 * @brief RobotModelLoader class
 * @author Yongxi Huang
 * @date 2021-05-29
 */

#ifndef MOVEIT_NO_ROS_ROBOTMODELLOADER_H
#define MOVEIT_NO_ROS_ROBOTMODELLOADER_H

#include "moveit/rdf_loader/rdf_loader.h" // rdf_loader::RDFLoader
#include "moveit/robot_model/robot_model.h" // robot_model::RobotModel
#include "robot_model/KinematicsLoader.h"
#include "robot_model/JointLimitsLoader.h"
#include "moveit/robot_model/joint_model.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/planning_scene/planning_scene.h"
#include "BulletCast.h"

MOVEIT_CLASS_FORWARD(RobotModelLoader); // define the smart pointer of loader

/**
 * @brief Implementation of robot_model_loader::RobotModelLoader without ros environment.
 * @details  RobotModelLoader is used to load robot model, together with its kinematics solvers and joint limits
 * configuration without ros environment.
 *
 * It play the same role as robot_model_loader::RobotModelLoader.
 */
class RobotModelLoader {
public:
    /** @todo Remove default constructor */
    // RobotModelLoader()= default;

    /// Constructor from urdf and srdf file contents.
    RobotModelLoader(const std::string &urdf_contents, const std::string &srdf_contents);

    /// Constructor from urdf and srdf input stream.
    RobotModelLoader(std::istream &urdf_input, std::istream &stdf_input);

    /**
     * @brief Load Kinematics Solvers
     * @details RobotModelLoader can not configure kinematic solvers.
     * All configuration should be done through KinematicsLoader.
     * @param loader Smart pointer of KinematicsLoader.
     * All configured solver in loader would be created and bound with
     * corresponding joint model group.
     */
    void loadKinematicsSolvers(const KinematicsLoaderPtr& loader);

    /**
     * Load joint limits.
     * @param loader JointLimitsLoader which can be created from joint limits yaml configuration file.
     */
    void loadJointLimits(const JointLimitsLoaderConstPtr& loader);

    // Create a kinematics solver for specific joint model group.
    // Note that the solver is not bound with joint model group yet.
    // kinematics::KinematicsBasePtr allocateSolver(const moveit::core::JointModelGroup* jmg);

    // getAllocatorFn return the function pointer of allocateSolver
    // moveit::core::SolverAllocatorFn getAllocatorFn();

    // Note: Do not return const ptr of model.
    // const ptr of model would only be able get const ptr of joint model group.
    robot_model::RobotModelPtr getRobotModel();
    rdf_loader::RDFLoaderPtr getRDFLoader();
    robot_state::RobotStatePtr newRobotState();
    planning_scene::PlanningScenePtr newPlanningScene();

    long countModelUse();

    void outputModelInfo(std::ostream& out);
    void outputLimitsInfo(std::ostream& out);
    std::string modelInfoString();

    /**
     * JointInfo returned by PyBullet.getJointInfo.
     * @details It is mainly used as map info between pybullet and moveit.
     */
    struct BtJointInfo {
        int jointIndex;
        std::string jointName;
        int jointType;
        /// Corresponding moveit joint model.
        const moveit::core::JointModel* moveitJointModel;
    };

    /**
     * Get the pybullet joint index corresponding to a specific jointName.
     * @param jointName The name of joint.
     * @return pybullet joint index.
     */
    int getBtJointIndex(const std::string& jointName);

    std::map<std::string, BtJointInfo> joint_info_map_;
    void printJointMap();
private:
    rdf_loader::RDFLoaderPtr rdf_loader_;
    robot_model::RobotModelPtr robot_model_;
};

RobotModelLoaderPtr createRobotModelLoaderFromFile(const std::string& urdf_path, const std::string& srdf_path);

#endif //MOVEIT_NO_ROS_ROBOTMODELLOADER_H
