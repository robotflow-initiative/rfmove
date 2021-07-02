//
// Created by yongxi on 2021/5/26.
//
/*
 * KinematicsLoader is used to load kinematics configuration
 * and provide kinematics solver allocator.
 */

#ifndef MOVEIT_NO_ROS_KINEMATICSLOADER_H
#define MOVEIT_NO_ROS_KINEMATICSLOADER_H

#include <string>
#include <vector>
#include <map>
#include <ostream>
#include <istream>
#include <yaml-cpp/yaml.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h> // Used when initialize a kinematic solver

class KinematicLoaderException : public std::exception {
public:
    explicit KinematicLoaderException(const char* str);
    const char* what() const noexcept override {
        return reason;
    }
private:
    const char * reason;
};

#define EmptyGroup KinematicLoaderException("Joint Model Group is Null.")
#define UnknownSolver KinematicLoaderException("Unknown Solver type.")
#define EmptyLinkModel KinematicLoaderException("No links within joint model group.")
#define NoGroupConfiguration KinematicLoaderException("No configuration for joint model group.")

//static const KinematicLoaderException EmptyJmg("Joint Model Group is null.");
MOVEIT_CLASS_FORWARD(KinematicsLoader);
class KinematicsLoader {
public:
    /*
     * SolverConfig defines the configuration of a kinematic solver.
     * - group: name of JointModelGroup this solver supports.
     * - solver_type: type of solver.
     */
    struct SolverConfig {
        std::string group;
        std::string solver_types;
        double search_resolution;
        double timeout;
        unsigned int attempts;
        std::string tip_link;
    };

    KinematicsLoader() = default;
    explicit KinematicsLoader(const std::string& kinematics_config_contents);
    kinematics::KinematicsBasePtr allocKinematicsSolver(const moveit::core::JointModelGroup* jmg, rdf_loader::RDFLoaderPtr rdf_loader_);
    moveit::core::SolverAllocatorFn getAllocatorFn(rdf_loader::RDFLoaderPtr rdf_loader_);

    /**
     * @brief Get kinematics configuration.
     * @details Returned configuration is read-only. Do not use this function to set configuration.
     * @return A map between joint model group name and kinematic solver configuration.
     */
    const std::map<std::string, SolverConfig>& getConfiguration() const;
    //explicit KinematicsLoader(const std::istream& input);
    std::string configToString();
private:
    std::map<std::string, SolverConfig> configuration;  // map<group_name : solver_configuration>
    YAML::Node yaml;
    static kinematics::KinematicsBasePtr createSolver(const std::string& solver_type);
    static kinematics::KinematicsBasePtr createInitSolver(
            const std::string& solver_type,
            rdf_loader::RDFLoaderPtr rdf_loader_,
            const std::string& group_name,
            const std::string& base_frame,
            const std::string& tip_frame,
            double search_discretization);
    static inline std::string chooseTipFrames(const moveit::core::JointModelGroup* jmg);
    static const std::string default_solver_type;
    static const double default_timeout;
    static const unsigned int default_attempts;
    static const double default_search_resolution;
};

KinematicsLoaderPtr createKinematicsLoaderFromFile(const std::string& yaml_path);

#endif //MOVEIT_NO_ROS_KINEMATICSLOADER_H
