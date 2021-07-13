//
// Created by yongxi on 2021/7/13.
//

#include "controller/hardware.h"

class DummyJointHandler : JointHandler {
public:
    explicit DummyJointHandler(const std::string& _name) {
        name_ = _name;
    }

    ~DummyJointHandler() override = default;

    double position() override {return position_;}
    double velocity() override {return velocity_;}
    bool setPosVel(const double& position, const double& velocity) override {
        position_ = position;
        velocity_ = velocity;
        return true;
    }
    bool setPosition(const double& position) override {
        position_ = position;
        return true;
    };
    bool setVelocity(const double& velocity) override {
        velocity_ = velocity;
        return true;
    };
    const std::string& name() override { return name_;};
private:
    double position_ = 0;
    double velocity_ = 0;
    std::string name_;
};

int main() {
    DummyJointHandler handler("joint_0");
    return 0;
}