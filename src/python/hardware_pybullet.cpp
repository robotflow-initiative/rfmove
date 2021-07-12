//
// Created by yongxi on 2021/7/11.
//

#include "hardware_pybullet.h"
#include <iostream>
#include <utility>


PybulletJointHandler::PybulletJointHandler(int body_id, int joint_index, const std::string& joint_name,  PyAttr getJointState)
    : getJointState_(getJointState)
{
    joint_index_ = joint_index;
    body_id_ = body_id;
    name_ = joint_name;
}

PybulletJointHandler::~PybulletJointHandler() {
    std::cout << "Destructor of PybulletJointHandler" << std::endl;
}

const std::string & PybulletJointHandler::name() {
    return name_;
}

double PybulletJointHandler::position() {
    return py::cast<py::tuple>(getJointState_(body_id_, joint_index_))[0].cast<double>();
}

double PybulletJointHandler::velocity() {
    return py::cast<py::tuple>(getJointState_(body_id_, joint_index_))[1].cast<double>();
}

bool PybulletJointHandler::setPosVel(const double &position, const double &velocity) {
    return false;
}

bool PybulletJointHandler::setPosition(const double &position) {
    return false;
}

bool PybulletJointHandler::setVelocity(const double &velocity) {
    return false;
}

PybulletHardware::PybulletHardware(py::handle pybullet, int bodyUniqueId)
        :pybullet_(py::reinterpret_borrow<py::object>(pybullet))
        , getNumBodies_(pybullet_.attr("getNumBodies"))
        , getJointInfo_(pybullet_.attr("getJointInfo"))
        , getNumJoints_(pybullet_.attr("getNumJoints"))
        , getJointState_(pybullet_.attr("getJointState")){
    body_id_ = bodyUniqueId;
    joint_num_ = py::cast<int>(getNumJoints_(body_id_));
    std::cout << "Create PybulletHardware for body " << body_id_ << " with " << joint_num_ << " joints." << std::endl;
    for(int i=0; i<joint_num_; ++i) {
        auto info = py::cast<py::tuple>(getJointInfo_(body_id_, i));
        // std::cout << "[ " << info[0].cast<int>() << ", " << info[1].cast<std::string>() << ']' << std::endl;
        joint_index_map_[info[1].cast<std::string>()] = i;
    }
}

int PybulletHardware::getNumBodies() {
    return py::cast<int>(getNumBodies_());
}

JointHandler * PybulletHardware::getJointHandler(const std::string &joint_name) {
    return new PybulletJointHandler(body_id_, joint_index_map_[joint_name], joint_name, getJointState_);
}

void PybulletHardware::printJointInfo() {
    for(int i=0; i<joint_num_; ++i) {
        auto info = py::cast<py::tuple>(getJointInfo_(body_id_, i));
        std::cout << "[ " << info[0].cast<int>() << ", " << info[1].cast<std::string>() << ']' << std::endl;
    }
}

void PybulletHardware::printJointState() {
    for(int i=0; i<joint_num_; ++i) {
        auto state = py::cast<py::tuple>(getJointState_(body_id_, i));
        auto info = py::cast<py::tuple>(getJointInfo_(body_id_, i));
        std::cout << info[1].cast<std::string>() << ": [ " << state[0].cast<double>() << ", " << state[1].cast<double>() << ']' << std::endl;
    }
}
