//
// Created by yongxi on 2021/5/26.
//

#include "robot_model/KinematicsLoader.h"
#include "path_util.h"
#include <fstream>
#include <memory>
#include <utility>
#include "kdl_kinematics_plugin/kdl_kinematics_plugin.h"
#include <class_loader/class_loader.h>
#include "franka_panda_arm_ikfast_moveit_plugin.h"
#include "tobor_left_arm_group_ikfast_moveit_plugin.h"

const std::string KinematicsLoader::default_solver_type = "kdl_kinematics_plugin/KDLKinematicsPlugin"; // NOLINT
const double KinematicsLoader::default_timeout = 0.5;
const unsigned int KinematicsLoader::default_attempts = 2;
const double KinematicsLoader::default_search_resolution = 0.005;

KinematicsLoader::KinematicsLoader(const std::string& kinematics_config_contents) {
    yaml = YAML::Load(kinematics_config_contents);

    for (YAML::const_iterator groupCfg = yaml.begin(); groupCfg != yaml.end(); ++groupCfg) {
        std::string group_name = groupCfg->first.as<std::string>();
        configuration[group_name] = SolverConfig{
            .group =  group_name,
        };

        // load solver types
        if(groupCfg->second["kinematics_solver"]) {
            configuration[group_name].solver_types = groupCfg->second["kinematics_solver"].as<std::string>();
        } else {
            configuration[group_name].solver_types = default_solver_type;
        }

        // load solver timeout
        if(groupCfg->second["kinematics_solver_timeout"]) {
            configuration[group_name].timeout = groupCfg->second["kinematics_solver_timeout"].as<double>();
        } else {
            configuration[group_name].timeout = default_timeout;
        }

        // load solver attempts
        if(groupCfg->second["kinematics_solver_attempts"]) {
            configuration[group_name].timeout = groupCfg->second["kinematics_solver_attempts"].as<unsigned int>();
        } else {
            configuration[group_name].attempts = default_attempts;
        }

        // load solver search resolution
        if(groupCfg->second["kinematics_solver_search_resolution"]) {
            configuration[group_name].search_resolution = groupCfg->second["kinematics_solver_search_resolution"].as<double>();
        } else {
            configuration[group_name].search_resolution = default_search_resolution;
        }

        // load tip links
        if(groupCfg->second["kinematics_solver_ik_link"]) {
            const YAML::Node tmpNode = groupCfg->second["kinematics_solver_ik_link"];
            //for(YAML::const_iterator ite = tmpNode.begin(); ite != tmpNode.end(); ++ite) {
            //    configuration[group_name].tip_links.push_back(ite->as<std::string>());
            //}
            configuration[group_name].tip_link = groupCfg->second["kinematics_solver_ik_link"].as<std::string>();
        }
    }
}

std::string KinematicsLoader::configToString() {
    YAML::Node node;
    for (const auto& groupCfg : configuration) {
        YAML::Node groupNode;
        groupNode["kinematics_solver"] = groupCfg.second.solver_types;
        groupNode["kinematics_solver_search_resolution"] = groupCfg.second.search_resolution;
        groupNode["kinematics_solver_timeout"] = groupCfg.second.timeout;
        groupNode["kinematics_solver_attempts"] = groupCfg.second.attempts;
        //for(const auto& tip : groupCfg.second.tip_links) {
        //    groupNode["kinematics_solver_ik_link"].push_back(tip);
        //}
        groupNode["kinematics_solver_ik_link"] = groupCfg.second.tip_link;
        node[groupCfg.first] = groupNode;
    }
    return Dump(node);
}

KinematicsLoaderPtr createKinematicsLoaderFromFile(const std::string& yaml_path) {
    std::ifstream input(yaml_path);
    std::string yaml_content = ioToString(input);
    input.close();
    // return std::make_shared<KinematicsLoader>(yaml_content);
    return KinematicsLoaderPtr(new KinematicsLoader(yaml_content));
}

kinematics::KinematicsBasePtr KinematicsLoader::allocKinematicsSolver(const moveit::core::JointModelGroup* jmg, rdf_loader::RDFLoaderPtr rdf_loader_) {
    if(!jmg) {
        ROS_ERROR("Null joint model group when allocate kinematics solver.");
        throw EmptyGroup;
    }
    kinematics::KinematicsBasePtr result;
    // Look up solver type for jmg
    std::map<std::string, KinematicsLoader::SolverConfig>::const_iterator config_ite = configuration.find(jmg->getName());
    if (config_ite != configuration.end()) {
        // Create solver of specific type.
        const KinematicsLoader::SolverConfig* solver_cfg = &(config_ite->second);
        ROS_INFO_STREAM("Create kinematic solver for " << solver_cfg->solver_types);
        // result = createSolver(solver_cfg->solver_types);

        // Initialize solver
        // choose base frame
        // TODO: Print base and tip info in console for debug.
        // TODO: Base and tip frame info is already defined in srdf file.
        // TODO: Raise error when creation failed.
        const std::vector<const robot_model::LinkModel*>& links = jmg->getLinkModels();
        const std::string& base = links.front()->getParentJointModel()->getParentLinkModel() ?
                                  links.front()->getParentJointModel()->getParentLinkModel() -> getName() :
                                  jmg->getParentModel().getModelFrame();
        ROS_INFO_STREAM("Use base frame " << base);

        // choose tip link
        result = createInitSolver(solver_cfg->solver_types, rdf_loader_, jmg->getName(),
                                  (base.empty() || base[0]!='/') ? base : base.substr(1),
                                  solver_cfg->tip_link.empty() ? chooseTipFrames(jmg) : solver_cfg->tip_link,
                                  solver_cfg->search_resolution);
        /*
        result->initialize(rdf_loader_, jmg->getName(),
                           (base.empty() || base[0]!='/') ? base : base.substr(1),
                           solver_cfg->tip_links.empty() ? chooseTipFrames(jmg) : solver_cfg->tip_links,
                           solver_cfg->search_resolution);
                           */
        ROS_INFO_STREAM("Solver initialized.");
        // Other setting
        result->setDefaultTimeout(solver_cfg->timeout);
        return result;
    } else {
        ROS_ERROR_STREAM("No kinematic configuration for joint model group " << jmg->getName());
        throw NoGroupConfiguration;
    }
}

moveit::core::SolverAllocatorFn KinematicsLoader::getAllocatorFn(rdf_loader::RDFLoaderPtr rdf_loader_) {
    return boost::bind(&KinematicsLoader::allocKinematicsSolver, this, _1, rdf_loader_);
}

const std::map<std::string, KinematicsLoader::SolverConfig>& KinematicsLoader::getConfiguration() const {
    return configuration;
}

/*
 * Note:
 *  This is slightly different from KinematicsPluginLoader::KinematicsLoaderImpl::chooseTipFrames.
 *  This function does not check the tip configured. But choose them from model group directly.
 */
std::string KinematicsLoader::chooseTipFrames(const moveit::core::JointModelGroup* jmg)
{
    // Get the last link in the chain
    return jmg->getLinkModels().back()->getName();
}
/*
template<class T> std::shared_ptr<T> to_std_ptr(const boost::shared_ptr<T> &p) {
    typedef Holder<std::shared_ptr<T>> H;
    if(H *h = boost::get_deleter<H>(p)) {
        return h->p;
    } else {
        return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T>>(p));
    }
}*/

kinematics::KinematicsBasePtr KinematicsLoader::createSolver(const std::string& solver_type){
    
   
    if (solver_type=="kdl_kinematics_plugin/KDLKinematicsPlugin") {
        return kinematics::KinematicsBasePtr(new kdl_kinematics_plugin::KDLKinematicsPlugin());
    }
    if (solver_type=="franka_ikfast_panda_arm_plugin/IKFastKinematicsPlugin") {
        return kinematics::KinematicsBasePtr(new franka_panda_arm_ikfast::IKFastKinematicsPlugin());
    }
    
    if (solver_type=="tobor_ikfast_left_arm_group_plugin/IKFastKinematicsPlugin"){
        return kinematics::KinematicsBasePtr(new tobor_left_arm_group::IKFastKinematicsPlugin());
    }
    ROS_ERROR_STREAM("Unknown solver type " << solver_type);
    throw UnknownSolver;
}

kinematics::KinematicsBasePtr KinematicsLoader::createInitSolver(
        const std::string& solver_type,
        rdf_loader::RDFLoaderPtr rdf_loader_,
        const std::string& group_name,
        const std::string& base_frame,
        const std::string& tip_frame,
        double search_discretization)
{
    if (solver_type=="kdl_kinematics_plugin/KDLKinematicsPlugin") {
        auto* kdl_plugin = new kdl_kinematics_plugin::KDLKinematicsPlugin();
        ROS_INFO_STREAM("Initialize kdl for " << group_name << ". From " << base_frame << " To " << tip_frame);
        kdl_plugin ->initialize(rdf_loader_, group_name, base_frame, tip_frame, search_discretization);
        return kinematics::KinematicsBasePtr(kdl_plugin);
    }
    else if(solver_type=="franka_ikfast_panda_arm_plugin/IKFastKinematicsPlugin") 
    {
        franka_panda_arm_ikfast::IKFastKinematicsPlugin* franka_ikfast_panda_arm_plugin =new franka_panda_arm_ikfast::IKFastKinematicsPlugin();
        ROS_INFO_STREAM("Initialize ikfast for " << group_name << ". From " << base_frame << " To " << tip_frame);
        franka_ikfast_panda_arm_plugin->initialize(rdf_loader_, group_name, base_frame, tip_frame, search_discretization);
        return kinematics::KinematicsBasePtr(franka_ikfast_panda_arm_plugin);
    }
    else if (solver_type=="tobor_ikfast_left_arm_group_plugin/IKFastKinematicsPlugin"){
        tobor_left_arm_group::IKFastKinematicsPlugin* tobor_ikfast_left_arm_group_plugin =new tobor_left_arm_group::IKFastKinematicsPlugin();
        ROS_INFO_STREAM("Initialize ikfast for " << group_name << ". From " << base_frame << " To " << tip_frame);
        tobor_ikfast_left_arm_group_plugin->initialize(rdf_loader_, group_name, base_frame, tip_frame, search_discretization);
        return kinematics::KinematicsBasePtr(tobor_ikfast_left_arm_group_plugin);
    }
    else {

        ROS_ERROR_STREAM("Solver " << solver_type << "not supported yet.");
        return kinematics::KinematicsBasePtr(nullptr);
    }
}

KinematicLoaderException::KinematicLoaderException(const char *str) : reason(str){

}
