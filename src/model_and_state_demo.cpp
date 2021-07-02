//
// Created by yongxi on 2021/5/26.
//

#include "robot_model/KinematicsLoader.h"
#include "robot_model/RobotModelLoader.h"
#include "moveit/robot_state/robot_state.h"
#include <iostream>
#include "kdl_parser/kdl_parser.hpp"
#include "moveit/robot_model/robot_model.h"

int main() {

    // Create robot model
    RobotModelLoaderPtr loader = createRobotModelLoaderFromFile("resources/panda_arm_hand.urdf",
                                                                "resources/panda_arm_hand.srdf");
    std::cout << "Model Ref use after creation: " << loader->countModelUse() << std::endl;
    // Print out info of robot model
    std::cout << "===== RobotModel Info =====" << std::endl;
    //loader->outputModelInfo(std::cout);

    // Print out current joint limits info
    std::cout << "\n===== Joint Limits Info =====" << std::endl;
    //loader->outputLimitsInfo(std::cout);

    // Allocate Kinematics Solver
    /*
    std::cout << "\n===== Allocate Kinematics Solver =====" << std::endl;
    loader.initKinematicFromFile("resources/kinematics.yaml");
    moveit::core::JointModelGroup* jmg = loader.getRobotModel()->getJointModelGroup("panda_arm");
    kinematics::KinematicsBasePtr solver = loader.allocateSolver(jmg);
    std::cout << "Kinematics solver created for group " << solver->getGroupName() << std::endl;
    std::cout << "From " << solver->getBaseFrame() << " To " << solver->getTipFrame() << std::endl;
    */

    // Implement MoveIt robot model and robot state tutorial.
    std::cout << "\n===== Implement MoveIt robot model and robot state tutorial =====" << std::endl;
    robot_model::RobotModelPtr kinematic_model = loader->getRobotModel();
    moveit::core::JointModelGroup* jmg = loader->getRobotModel()->getJointModelGroup("panda_arm");
    // rdf_loader::RDFLoaderPtr rdf_loader_ = loader->getRDFLoader();
    KinematicsLoaderPtr kinematics_loader = createKinematicsLoaderFromFile("resources/kinematics.yaml");
    //kinematic_model->setKinematicsAllocators(std::map<std::string, moveit::core::SolverAllocatorFn>{
    //        {"panda_arm", kinematics_loader->getAllocatorFn(rdf_loader_)}
    //});
    loader->loadKinematicsSolvers(kinematics_loader);

    //std::cout << "Model frame: " << kinematic_model->getModelFrame() << std::endl;
    std::printf("Model frame: %s\n", kinematic_model->getModelFrame().c_str());
    //robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    std::cout << "model ptr use before state: " << loader->countModelUse() << std::endl;
    robot_state::RobotStatePtr kinematic_state = loader->newRobotState();
    std::cout << "model ptr use after state: " << loader->countModelUse() << std::endl;
    kinematic_state->setToDefaultValues();

    const std::vector<std::string>& joint_names = jmg->getVariableNames();
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(jmg, joint_values);
    std::printf("\n== Get Joint Value ==\n");
    std::printf("Current position:\n");
    for (std::size_t i = 0; i< joint_names.size(); ++i) {
        std::printf("\tJoint %s: %f\n", joint_names[i].c_str(), joint_values[i]);
    }

    std::printf("\n== Joint Limit ==\n");
    std::printf("Set position out of bound\n");
    joint_values[0] = 5.57;
    kinematic_state ->setJointGroupPositions(jmg, joint_values);
    std::cout << "Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid") << std::endl;

    std::printf("Enforce bounds\n");
    kinematic_state->enforceBounds();
    std::cout << "Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid") << std::endl;

    std::printf("\n== Forward Kinematics ==\n");
    kinematic_state->setToRandomPositions(jmg);
    const Eigen::Affine3d& end_effector_state = kinematic_state ->getGlobalLinkTransform("panda_link8");
    std::cout << "Translation: \n" << end_effector_state.translation() << std::endl;
    std::cout << "Rotation: \n" << end_effector_state.rotation() << std::endl;

    std::printf("\n== Inverse Kinematics ==\n");
    bool found_ik = kinematic_state->setFromIK(jmg, end_effector_state, 10, 0.1);
    if(found_ik) {
        kinematic_state ->copyJointGroupPositions(jmg, joint_values);
        for (std::size_t i = 0; i< joint_names.size(); ++i) {
            std::printf("\tJoint %s: %f\n", joint_names[i].c_str(), joint_values[i]);
        }
    } else {
        std::cout << "Did not find IK solution" << std::endl;
    }

    std::printf("\n== Get the Jacobian ==\n");
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state ->getJacobian(jmg,
                                  kinematic_state->getLinkModel(jmg->getLinkModelNames().back()),
                                  reference_point_position, jacobian);

    std::cout << "Jacobian:\n" << jacobian << std::endl;


    std::cout << "sizeof(double) = " << sizeof(double) << std::endl;
    return 0;
}