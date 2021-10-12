//
// Created by yongxi on 2021/6/12.
//

#ifndef MOVEIT_NO_ROS_ROBOTSTATE_H
#define MOVEIT_NO_ROS_ROBOTSTATE_H

#include <pybind11/pybind11.h>
#include <moveit/robot_model/robot_model.h>

namespace py = pybind11;

void declare_robot_state(py::module &m) {
    py::class_<moveit::core::RobotState, std::shared_ptr<moveit::core::RobotState>>(m, "RobotState")
        /** @warning Do not create RobotState instance directly. Use RobotStatePtr created by RobotModelLoader instead.
         *  It would raise error when free RobotState instance because it contains shared_ptr in it....maybe... */
        //.def(py::init<const moveit::core::RobotModelConstPtr&>())
        //.def("copyJointGroupPositions", static_cast
        //    <void(moveit::core::RobotState::*)(const moveit::core::JointModelGroup* group, std::vector<double>&) const>
        //    (&moveit::core::RobotState::copyJointGroupPositions),
        //    "Get a copy of joint model group positions")
        .def("__str__", [](moveit::core::RobotState& self){
            return self.getRobotModel()->getName();
        })
        .def("__repr__", [](moveit::core::RobotState& self) {
            std::ostringstream sstr;
            sstr << "<moveit::core::RobotState at " << &self <<'>' << std::endl;
            sstr << self.getRobotModel()->getName();
            return sstr.str();
        })
        .def("copyJointGroupPositions", [](moveit::core::RobotState& self, const moveit::core::JointModelGroup* group) {
                 std::vector<double> result;
                 self.copyJointGroupPositions(group, result);
                 return result;
             },"Get a copy of positions corresponding to a joint model group.",
             py::arg("joint_model_group"))
        .def("setToDefaultValues", static_cast
            <void(moveit::core::RobotState::*)(void)>
            (&moveit::core::RobotState::setToDefaultValues))
        .def("setJointGroupPositions", static_cast
             <void(moveit::core::RobotState::*)(const moveit::core::JointModelGroup*, const std::vector<double>&)>
             (&moveit::core::RobotState::setJointGroupPositions),
             py::arg("joint_model_group"),
             py::arg("positions"))
                /** Note that default parameter value should be set in argument description instead of function decleration. */
        .def("satisfiesBounds", static_cast
             <bool(moveit::core::RobotState::*)(double)const>
             (&moveit::core::RobotState::satisfiesBounds),
             py::arg("margin")=0.0)
        .def("satisfiesBounds", static_cast
             <bool(moveit::core::RobotState::*)(const moveit::core::JointModelGroup*, double)const>
             (&moveit::core::RobotState::satisfiesBounds),
             py::arg("joint_model_group"),
             py::arg("margin")=0.0)
        .def("enforceBounds", static_cast
            <void(moveit::core::RobotState::*)(void)>
            (&moveit::core::RobotState::enforceBounds))
        .def("enforceBounds", static_cast
            <void(moveit::core::RobotState::*)(const moveit::core::JointModelGroup*)>
            (&moveit::core::RobotState::enforceBounds))
        .def("setToRandomPositions", static_cast
                <void(moveit::core::RobotState::*)(void)>
        (&moveit::core::RobotState::setToRandomPositions))
        .def("setToRandomPositions", static_cast
            <void(moveit::core::RobotState::*)(const moveit::core::JointModelGroup*)>
            (&moveit::core::RobotState::setToRandomPositions),
            py::arg("joint_model_group"))
        /** getGlobalLinkTransform would update link transforms first.
         *  There is another const version of getGlobalLinkTransform which does not but raise error if
         *  checkLinkTransforms fails.
         */
        .def("getGlobalLinkTransform", static_cast
            <const Eigen::Affine3d&(moveit::core::RobotState::*)(const std::string&)>
            (&moveit::core::RobotState::getGlobalLinkTransform))
        /**
         * @todo: We can not bind setFromIK directly for now as we have not bind all parameter classes to python.
         */
        .def("setFromIK", [](moveit::core::RobotState &self, const moveit::core::JointModelGroup* group, const Eigen::Affine3d& pose, int attempts = 3, double timeout = 1.0)->bool{
                return self.setFromIK(group, pose, attempts, timeout);
            },
            "" ,
            py::arg("group"),
            py::arg("pose"),
            py::arg("attempts") = 3,
            py::arg("timeout") = 1.0)
        .def("getJacobian", [](moveit::core::RobotState &self, const moveit::core::JointModelGroup* jmg, const std::string& linkModel, const Eigen::Vector3d& pos) -> Eigen::MatrixXd{
            /// @todo return pointer and let python take ownership of it
            Eigen::MatrixXd jacobian;
            self.getJacobian(jmg, self.getLinkModel(linkModel), pos, jacobian);
            return jacobian;
        })
        /// We use lambda function for setJointPosition instead of func cast. Note that the function name has been changed.
        .def("setJointPosition", [](moveit::core::RobotState &self, const std::string& joint_name, double position)->void {
            self.setJointPositions(joint_name, &position);
        })
        .def("getJointPosition", [](moveit::core::RobotState &self, const std::string& joint_name) -> double {
            return self.getJointPositions(joint_name)[0];
        })
        .def("getJointVelocity", [](moveit::core::RobotState &self, const std::string& joint_name) -> double {
            return self.getJointVelocities(joint_name)[0];
        })
        .def("getVariableAcceleration",[](moveit::core::RobotState &self,const std::string& joint_name)->double {
            return self.getVariableAcceleration(joint_name);
        })
        .def("toMsg", [](moveit::core::RobotState &self) {

        })
        .def("dirty", &moveit::core::RobotState::dirty)
        .def("update", &moveit::core::RobotState::update,
         py::arg("force") = false)
        .def("linkRelativeTransform", [](moveit::core::RobotState& self,
                                                const std::string& link_name,
                                                const std::string& frame_id) -> Eigen::Affine3d {
            Eigen::Affine3d linkTran = self.getGlobalLinkTransform(link_name);
            Eigen::Affine3d frameTran = self.getFrameTransform(frame_id);
            return (frameTran.inverse(Eigen::TransformTraits::Affine) * linkTran);
        });

}


#endif //MOVEIT_NO_ROS_ROBOTSTATE_H
