//
// Created by yongxi on 2021/6/1.
//

#ifndef MOVEIT_NO_ROS_JOINTMODELGROUP_H
#define MOVEIT_NO_ROS_JOINTMODELGROUP_H

#include <pybind11/pybind11.h>
#include <moveit/robot_model/joint_model_group.h>
namespace py = pybind11;

/**
 * Bind JointModelGroup and JointModel to python
 * @param m pybind11 module reference
 */
void declare_joint_model_group(py::module &m){

    /** @warning We do not need to set the container of JointModelGroup because the moveit interfaces always return
     *  JointModelGroup as raw pointer instead of shared_ptr. In that case, we can just set the return policy of these
     *  functions returning JointModelGroup to reference. The default container unique_ptr works fine here.
     */
    py::class_<moveit::core::JointModelGroup>(m, "JointModelGroup")
        .def_property("name", &moveit::core::JointModelGroup::getName, nullptr)
        .def("getName", &moveit::core::JointModelGroup::getName, "Get the name of grup")
        .def("getJointModelNames", &moveit::core::JointModelGroup::getJointModelNames,
             py::return_value_policy::reference_internal,
             "Return a list contains all the JointModel Names.")
        .def("getJointModel", &moveit::core::JointModelGroup::getJointModel,
             py::return_value_policy::reference_internal,
             "Get the JointModel with corresponding name.")
        .def("getJointModels", &moveit::core::JointModelGroup::getJointModels,
             py::return_value_policy::reference_internal,
             "Return a list contains all the JointModels")
        .def("getVariableNames", &moveit::core::JointModelGroup::getVariableNames,
             py::return_value_policy::reference_internal,
             "Return a list contains all the variable names")
        .def("getLinkModelNames", &moveit::core::JointModelGroup::getLinkModelNames,
            py::return_value_policy::reference_internal,
            "Return a list contains all the LinkModel Names.")
        .def("getEndEffectorTips", [](moveit::core::JointModelGroup& self) -> std::vector<std::string> {
            std::vector<std::string> res;
            self.getEndEffectorTips(res);
            return res;
        });

    py::class_<moveit::core::JointModel>(m, "JointModel")
        .def("getName", &moveit::core::JointModel::getName,
             "Get the name of Joint")
        .def("getJointIndex", &moveit::core::JointModel::getJointIndex,
             "Get the index of this joint within the robot model.")
        .def_property("variables", &moveit::core::JointModel::getVariableNames, nullptr)
        .def("getVariableBounds", static_cast
            <const moveit::core::VariableBounds& (moveit::core::JointModel::*)(const std::string& variable) const>
            (&moveit::core::JointModel::getVariableBounds), py::return_value_policy::reference_internal,
            "Get variable bound according to the variable name.")
        .def("getVariableBounds", static_cast
            <const std::vector<moveit::core::VariableBounds>& (moveit::core::JointModel::*)(void) const>
            (&moveit::core::JointModel::getVariableBounds), py::return_value_policy::reference_internal,
            "Get all variable bounds.");

    py::class_<moveit::core::LinkModel>(m, "LinkModel")
        .def("__str__", [](moveit::core::LinkModel& self) {
            return self.getName();
        })
        .def("__repr__", [](moveit::core::LinkModel& self) {
            std::ostringstream sstr;
            sstr << "<moveit::core::LinkModel>" << std::endl;
            sstr << self.getName() << std::endl;
            return sstr.str();
        })
        .def("getName", &moveit::core::LinkModel::getName,
             "Get the name of Link");

    py::class_<moveit::core::VariableBounds>(m, "VariableBounds")
        .def("__str__", [](moveit::core::VariableBounds& self){
            std::ostringstream sstr;
            sstr << self;
            return sstr.str();
        })
        .def("__repr__", [](moveit::core::VariableBounds& self){
            std::ostringstream sstr;
            sstr << "<moveit::core::VariableBounds>" << std::endl;
            sstr << self;
            return sstr.str();
        });
}

#endif //MOVEIT_NO_ROS_JOINTMODELGROUP_H
