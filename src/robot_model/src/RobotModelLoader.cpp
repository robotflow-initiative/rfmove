//
// Created by yongxi on 2021/5/22.
//

#include "robot_model/RobotModelLoader.h"
#include "path_util.h"
#include <sstream>
#include <fstream>

RobotModelLoaderPtr createRobotModelLoaderFromFile(const std::string& urdf_path, const std::string& srdf_path){
    std::ifstream urdf_input(urdf_path);
    std::ifstream srdf_input(srdf_path);
    //std::make_shared<RobotModelLoader>(urdf_input, srdf_input);
    // BUG: The use of pointer cannot be tracked if we create return value through make_shared
    return RobotModelLoaderPtr(new RobotModelLoader(urdf_input, srdf_input));
}

RobotModelLoader::RobotModelLoader(const std::string& urdf_contents, const std::string& srdf_contents):
    rdf_loader_(new rdf_loader::RDFLoader(urdf_contents, srdf_contents)),
    robot_model_(new robot_model::RobotModel(rdf_loader_->getURDF(), rdf_loader_->getSRDF()))
{
    //std::cout << "model ref use during creation: " << countModelUse() << std::endl;
}

RobotModelLoader::RobotModelLoader(std::istream& urdf_input, std::istream& srdf_input){
    std::string urdf_contents = ioToString(urdf_input);
    std::string srdf_contents = ioToString(srdf_input);

    rdf_loader_.reset(new rdf_loader::RDFLoader(urdf_contents, srdf_contents));
    robot_model_.reset(new robot_model::RobotModel(rdf_loader_->getURDF(), rdf_loader_->getSRDF()));
    //std::cout << "model ref use during creation: " << countModelUse() << std::endl;
}

/* [DEPRECATED] Create KinematicsLoader instance directly instead of initializing it from RobotModelLoader.
void RobotModelLoader::initKinematic(const std::string& kinematic_config_contents) {
    kinematic_loader_.reset(new KinematicsLoader(kinematic_config_contents));
}

void RobotModelLoader::initKinematicFromFile(const std::string& kinematic_config_path) {
    std::ifstream kinematic_input(kinematic_config_path);
    std::string kinematic_config_contents = ioToString(kinematic_input);
    kinematic_input.close();
    initKinematic(kinematic_config_contents);
}
 */

/*
kinematics::KinematicsBasePtr RobotModelLoader::allocateSolver(const moveit::core::JointModelGroup* jmg) {
    return kinematic_loader_->allocKinematicsSolver(jmg, rdf_loader_);
}
 */

/*
moveit::core::SolverAllocatorFn RobotModelLoader::getAllocatorFn() {
    return boost::bind(&RobotModelLoader::allocateSolver, this, _1);
}
*/

void RobotModelLoader::loadKinematicsSolvers(const KinematicsLoaderPtr& kinematics_loader) {
    // Create AllocatorFn
    moveit::core::SolverAllocatorFn kinematic_allocator = kinematics_loader->getAllocatorFn(rdf_loader_);

    // Build AllocatorFn Map
    std::map<std::string, robot_model::SolverAllocatorFn> groupToFn;
    const std::map<std::string, KinematicsLoader::SolverConfig>& kinematics_conf = kinematics_loader->getConfiguration();
    for(const auto& conf : kinematics_conf) {
        // @todo Check the ability of allocator before bind solver with joint model group.
        if(!robot_model_->hasJointModelGroup(conf.first)) {
            ROS_WARN_STREAM("Robot Model " << robot_model_->getName() << " does not have joint model group "
                << conf.first << " which is configured in KinematicsLoader. Nothing would happen.");
            continue;
        }
        groupToFn[conf.first] = kinematic_allocator;
    }

    // Bound AllocatorFn to joint model group.
    robot_model_->setKinematicsAllocators(groupToFn);

    // Set other solver configuration
    for(const auto &conf : kinematics_conf) {
        if(!robot_model_->hasJointModelGroup(conf.first)) continue;
        robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(conf.first);
        jmg->setDefaultIKTimeout(conf.second.timeout);
        jmg->setDefaultIKAttempts(conf.second.attempts);
    }
}

void RobotModelLoader::loadJointLimits(const JointLimitsLoaderConstPtr& loader) {
    /*
    for(const auto& limit : *loader) {
        moveit::core::JointModel* jointModel = robot_model_->getJointModel(limit.first);
        std::cout << limit.first << ": " << limit.second << std::endl;
        jointModel->setVariableBounds();
    }
     */

    /**
     * @note If a joint has multi variable, the variable name will be "joint_name/variable1, joint_name/variable2, ...".
     */
    std::cout << "Current bounds:" << std::endl;
    for(auto joint: robot_model_->getJointModels()) {
        for(const auto& variable : joint->getVariableNames()) {
            /**
             * @note Coming joint limits should be appended on the original limits. So we merge the original and comming
             * limits together.
             */
            if (loader->has(variable)) {
                joint->setVariableBounds(variable,
                                         loader->mergeBounds(joint->getVariableBounds(variable), loader->getBound(variable)));
            }
        }
    }
}

robot_model::RobotModelPtr RobotModelLoader::getRobotModel() {
    return robot_model_;
}

rdf_loader::RDFLoaderPtr RobotModelLoader::getRDFLoader() {
    return rdf_loader_;
}

robot_state::RobotStatePtr RobotModelLoader::newRobotState() {
    return robot_state::RobotStatePtr(new robot_state::RobotState(robot_model_));
}

planning_scene::PlanningScenePtr RobotModelLoader::newPlanningScene() {
    return planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(robot_model_));
}

void RobotModelLoader::outputModelInfo(std::ostream& out) {
    out << robot_model_->getName() << ':' << std::endl;
    for (const auto& joint_model_group : robot_model_->getJointModelGroups()) {
        out << "  " << joint_model_group->getName() << ':' << std::endl;
        for(const auto& joint_model : joint_model_group->getJointModels()) {
            out << "    " << joint_model->getName() << ':' << std::endl;
            //out << "      " << "Type:\t" << joint_model->getTypeName() << std::endl;
            //out << "      " << "Variables:" << std::endl;
            //for(const auto& variable_name : joint_model->getVariableNames()) {
            //    out << "        - " << variable_name << std::endl;
            //}
        }
    }
}

std::string RobotModelLoader::modelInfoString() {
    std::ostringstream sstr;
    outputModelInfo(sstr);
    return sstr.str();
}

void RobotModelLoader::outputLimitsInfo(std::ostream &out) {
    for (const auto& joint : robot_model_->getJointModels()) {
        out << joint->getName() << std::endl;
        const std::vector<moveit::core::VariableBounds>& limits = joint->getVariableBounds();
        const std::vector<std::string>& variable_names = joint->getVariableNames();
        for(int i=0; i<variable_names.size(); ++i) {
            out << "  " << variable_names[i] << std::endl;
            out << "    position:\t";
            if (limits[i].position_bounded_) {
                out << '[' << limits[i].min_position_ << ", " << limits[i].max_position_ << ']'<< std::endl;
            } else {
                out << "none" << std::endl;
            }

            out << "    velocity:\t";
            if (limits[i].velocity_bounded_) {
                out << '[' << limits[i].min_velocity_ << ", " << limits[i].max_velocity_ << ']'<< std::endl;
            } else {
                out << "none" << std::endl;
            }

            out << "    acceleration:\t";
            if (limits[i].acceleration_bounded_) {
                out << '[' << limits[i].min_acceleration_ << ", " << limits[i].max_acceleration_ << ']'<< std::endl;
            } else {
                out << "none" << std::endl;
            }
        }
    }
}

long RobotModelLoader::countModelUse() {
    return robot_model_.use_count();
}

void RobotModelLoader::printJointMap() {
    for(auto joint_info : joint_info_map_) {
        std::cout << joint_info.first << "\t" << joint_info.second.moveitJointModel->getJointIndex()
            << '-' << joint_info.second.jointIndex << std::endl;
    }
}

int RobotModelLoader::getBtJointIndex(const std::string &jointName) {
    auto jointInfo = joint_info_map_.find(jointName);
    if (jointInfo == joint_info_map_.end()) {
        ROS_ERROR_STREAM("Joint " << jointName << " does not exist or has not been registered.");
        return -1;
    }
    return jointInfo->second.jointIndex;
}
