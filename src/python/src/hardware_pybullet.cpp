//
// Created by yongxi on 2021/7/11.
//

#include "hardware_pybullet.h"
#include <iostream>
#include <utility>
#include <pybind11/stl.h>

int PybulletJointHandler::POSITION_CONTROL_; // definition
int PybulletJointHandler::VELOCITY_CONTROL_; // definition

PybulletJointHandler::PybulletJointHandler(int body_id,
                                           int joint_index,
                                           const std::string& joint_name,
                                           PyAttr getJointState,
                                           PyAttr setJointMotorControl2)
    : getJointState_(getJointState)
    , setJointMotorControl2_(setJointMotorControl2)
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
    std::cout << "Set body " << body_id_ << " joint " << joint_index_ << " to " << "P: " << position << " V: " << velocity << std::endl;
    setJointMotorControl2_(body_id_, joint_index_, POSITION_CONTROL_, position, velocity);
    return true;
}

bool PybulletJointHandler::setPosition(const double &position) {
    std::cout << "Set body " << body_id_ << " joint " << joint_index_ << " to " << "P: " << position << std::endl;
    setJointMotorControl2_(body_id_, joint_index_, POSITION_CONTROL_, position);
    return true;
}

bool PybulletJointHandler::setVelocity(const double &velocity) {
    std::cout << "Set body " << body_id_ << " joint " << joint_index_ << " to " << "V: " << velocity << std::endl;
    setJointMotorControl2_(body_id_, joint_index_, VELOCITY_CONTROL_, velocity);
    return true;
}

PybulletHardware::PybulletHardware(py::handle pybullet, int bodyUniqueId)
        : pybullet_(py::reinterpret_borrow<py::object>(pybullet))
        , getNumBodies_(pybullet_.attr("getNumBodies"))
        , getJointInfo_(pybullet_.attr("getJointInfo"))
        , getNumJoints_(pybullet_.attr("getNumJoints"))
        , getJointState_(pybullet_.attr("getJointState"))
        , getJointStates_(pybullet_.attr("getJointStates"))
        , setJointMotorControl2_(pybullet_.attr("setJointMotorControl2")){
    body_id_ = bodyUniqueId;
    joint_num_ = py::cast<int>(getNumJoints_(body_id_));

    PybulletJointHandler::POSITION_CONTROL_ = pybullet_.attr("POSITION_CONTROL").cast<int>();
    PybulletJointHandler::VELOCITY_CONTROL_ = pybullet_.attr("VELOCITY_CONTROL").cast<int>();
    std::cout << "Create PybulletHardware for body " << body_id_ << " with " << joint_num_ << " joints." << std::endl;
    all_index_.resize(joint_num_);
    joint_names_.resize(joint_num_);
    for(int i=0; i<joint_num_; ++i) {
        auto info = py::cast<py::tuple>(getJointInfo_(body_id_, i));
        // std::cout << "[ " << info[0].cast<int>() << ", " << info[1].cast<std::string>() << ']' << std::endl;
        all_index_[i] = i;
        joint_index_map_[info[1].cast<std::string>()] = i;
        joint_names_[i] = info[1].cast<std::string>();
    }
}

int PybulletHardware::getNumBodies() {
    return py::cast<int>(getNumBodies_());
}

JointHandler * PybulletHardware::getJointHandler(const std::string &joint_name) {
    return new PybulletJointHandler(body_id_,
                                    joint_index_map_[joint_name],
                                    joint_name,
                                    getJointState_,
                                    setJointMotorControl2_);
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

void PybulletHardware::getJointStateAll(std::vector<double>& position, std::vector<double>& velocity) {
    position.resize(joint_num_);
    velocity.resize(joint_num_);
    // py::list tmp = py::cast<>(all_index_);
    auto all_states = py::cast<py::tuple>(getJointStates_(body_id_, all_index_));
    for(size_t i=0; i<joint_num_; ++i) {
        position[i] = all_states[i].cast<py::tuple>()[0].cast<double>();
        velocity[i] = all_states[i].cast<py::tuple>()[0].cast<double>();
    }
}
