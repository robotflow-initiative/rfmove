//
// Created by yongxi on 2021/7/11.
//

#include "controller/hardware.h"

#include <utility>

DefaultJointGroupHandler::DefaultJointGroupHandler(std::vector<JointHandler *> handlers) {
    handlers_ = std::move(handlers);
}

DefaultJointGroupHandler::~DefaultJointGroupHandler() {
    for(JointHandler* handler : handlers_) {
        delete handler;
    }
}

void DefaultJointGroupHandler::positions(std::vector<double> _positions) {
    _positions.resize(handlers_.size());
    for(size_t i = 0; i < handlers_.size(); ++i) {
        _positions[i] = handlers_[i]->position();
    }
}

void DefaultJointGroupHandler::velocities(std::vector<double> _velocities) {
    _velocities.resize(handlers_.size());
    for(size_t i = 0; i < handlers_.size(); ++i) {
        _velocities[i] = handlers_[i]->velocity();
    }
}

JointGroupHandler * HardwareInterface::getJointGroupHandler(const std::vector<std::string> &joint_names) {
    std::vector<JointHandler*> results;
    results.reserve(joint_names.size());
    for(const std::string& joint_name : joint_names) {
        results.push_back(getJointHandler(joint_name));
    }
    return new DefaultJointGroupHandler(results);
}