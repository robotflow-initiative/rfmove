//
// Created by yongxi on 2021/5/23.
//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // Makes returning of vector possible
#include <robot_model/RobotModelLoader.h>
#include <robot_model/ExampleClass.h>
#include <robot_model/KinematicsLoader.h>
#include <moveit/robot_state/robot_state.h>
#include <EigenPy.h>
#include <RobotModel.h>
#include <RobotState.h>
#include <JointModelGroup.h>
#include <PlanningScene.h>
#include <BulletCastPy.h>
#include <Eigen/Core>
#include <PlannerManager.h>
#include <PlanningContext.h>
#include <ConstraintsMessages.h>
#include <GeometryMessages.h>
#include <RobotTrajectory.h>
#include <Controller.h>
#include <Bullet3_helper.h>
#include <Shapes.h>
#include <JointLimitsPy.h>
#include <rfSpline.h>

#ifdef WITH_BULLET3
#include "Bullet3.h"
#endif

#include "Bullet3_hardware.h"

namespace py = pybind11;

PYBIND11_MODULE(moveit_noros, m) {
    //declare_transform<double, 3, 2, 0>(m, "Affine3d");
    declare_eigen(m);
    declare_robot_model(m);
    declare_robot_state(m);
    declare_joint_model_group(m);
    declare_planning_scene(m);
    declare_planner_manager(m);
    declare_planning_context(m);
    declare_geometry_msgs(m);
    declare_constraints_msgs(m);
    declare_robot_trajectory(m);
    declare_controller(m);
    declare_pybullet_controller(m);
    declare_pybullet_helper(m);
    declare_shapes(m);
    declare_jointlimits_loader(m);
    declare_rfSpline(m);
    
#ifdef WITH_BULLET3
    declare_bullet3(m);
#endif

    m.def("printOutJointInfo", &printOutJointInfo);

    /** @warning Make sure to declare shared_ptr as the container of RobotModelLoader
     *  Otherwise, after creation using createRobotModelLoaderFromFile, pybind11 would use
     *  unique_ptr instead of the returned shared_ptr as return value, which will cause
     *  the ref count to shared_ptr members of loader to 0.
     */
    py::class_<RobotModelLoader, std::shared_ptr<RobotModelLoader>>(m, "RobotModelLoader")
        .def(py::init<const std::string&, const std::string&>())
        .def("getModel", &RobotModelLoader::getRobotModel, // py::return_value_policy::reference_internal,
             "Get the RobotModel object loaded by loader")
        .def("loadKinematicsSolvers", &RobotModelLoader::loadKinematicsSolvers,
             "Load Kinematics Solvers configured within a KinematicsLoader and bind solvers with joint model groups.",
             py::arg("kinematics_loader"))
        .def("loadJointLimits", &RobotModelLoader::loadJointLimits,
             "Load Joint Limits configuration from JointLimitsLoader.")
        .def_property("modelInfo", &RobotModelLoader::modelInfoString, nullptr,
                      "Return a string about robot model info.")
        .def("newRobotState", &RobotModelLoader::newRobotState)
        .def("newPlanningScene", &RobotModelLoader::newPlanningScene)
        .def("countModelUse", &RobotModelLoader::countModelUse)
        /**
         * @brief Register the joint info in pybullet to RobotModelLoader.
         * @details The relationship between pybullet and moveit joint model is kept in joint_info_map_.
         * That relationship is helpful as the joint index in pybullet and moveit of the same RobotModel is different.
         * This function is implemented as a lambda function as we do not hope to include pybind11 within moveit_noros
         * cpp lib.
         * @param t A python list returned by pybullet.getJointInfo()
         * @todo Try to move joint_info_map_ into private.
         */
        .def("registerBtJointInfo", [](RobotModelLoader& self, const py::tuple& t) -> void{
             std::string jointName = t[1].cast<std::string>();
             //std::cout << "Register " << jointName << std::endl;
             const moveit::core::JointModel* joint_model = self.getRobotModel()->getJointModel(jointName);
             if (joint_model== nullptr) {
                 ROS_ERROR_STREAM("Robot have no joint named " << jointName);
                 return;
             }
             self.joint_info_map_[jointName] = RobotModelLoader::BtJointInfo{
                     .jointIndex = t[0].cast<int>(),
                     .jointName = jointName,
                     .jointType = t[2].cast<int>(),
                     .moveitJointModel = joint_model,
             };
        },
        "Register the joint info in pybullet to RobotModelLoader")
        .def("printJointMap", &RobotModelLoader::printJointMap)
        .def("getBtJointIndex", &RobotModelLoader::getBtJointIndex);

    m.def("createRobotModelLoaderFromFile", &createRobotModelLoaderFromFile,
          "Create a robot model loader based the file path of urdf and srdf file",
          py::arg("urdf_path"),
          py::arg("srdf_path"));

    py::class_<ExampleClass, std::shared_ptr<ExampleClass>>(m, "ExampleClass")
        .def(py::init<>())
        .def("AddString", &ExampleClass::AddString)
        .def("AddChild", &ExampleClass::AddChild)
        .def("AddChildPtr", &ExampleClass::AddChildPtr)
        .def("GetStringVec", &ExampleClass::GetStringVec)
        .def("GetChildPtrVec", &ExampleClass::GetChildPtrVec)
        .def("GetChildVec", &ExampleClass::GetChildVec)
        .def("copyStringVec", [](ExampleClass &self) {
            std::vector<std::string> str = {"a", "b", "c"};
            std::cout << "copy begin" << std::endl;
            self.copyStringVec(str);
            for(const auto& s : str) {
                std::cout << s << std::endl;
            }
            return str;
        });

    py::class_<ExampleChild>(m, "ExampleChild")
        .def(py::init<const std::string&>())
        .def("GetName", &ExampleChild::getName);

    m.def("changeChildName", &changeChildName);
    m.def("changeVectorContains", &changeVectorContains);

    py::class_<KinematicsLoader, std::shared_ptr<KinematicsLoader>>(m, "KinematicsLoader")
        //.def(py::init<const std::string&>())
        //.def(py::init<>())
        .def("configToString", &KinematicsLoader::configToString);
        //.def("allocKinematicsSolver", &KinematicsLoader::allocKinematicsSolver);

    //py::class_<JointLimitsLoader, std::shared_ptr<JointLimitsLoader>> (m, "JointLimitsLoader")
    //    .def("__str__", [](const JointLimitsLoader& self) -> std::string {
    //        std::ostringstream sstr;
    //        sstr << self;
    //        return sstr.str();
    //    })
    //    .def("__repr__", [](const JointLimitsLoader& self) -> std::string{
    //        std::ostringstream sstr;
    //        sstr << "<JointLimitsLoader>" << std::endl;
    //        sstr << self;
    //        return sstr.str();
    //    });

    m.def("createKinematicsLoaderFromFile", &createKinematicsLoaderFromFile,
          "Create a KinematicsLoader from an yaml configuration file.",
          py::arg("file_path"));

    m.def("createJointLimitsLoaderFromFile", &createJointLimitsLoaderFromFile,
          "Create a JointLimitsLoader from an yaml configuration file.",
          py::arg("file_path"));

    py::class_<kinematics::KinematicsBase, std::shared_ptr<kinematics::KinematicsBase>>(m, "KinematicsBase")
        .def("getGroupName", &kinematics::KinematicsBase::getGroupName);


}