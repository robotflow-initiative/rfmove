//
// Created by yongxi on 6/28/21.
//

#include "robot_model/RobotModelLoader.h"
#include "rokae_client/rokae_client.h"

int main() {
    RobotModelLoaderPtr loader = createRobotModelLoaderFromFile("resources/panda_arm_hand.urdf",
                                                                "resources/panda_arm_hand.srdf");
    robot_trajectory::RobotTrajectory trajectory(loader->getRobotModel(), "panda_arm");
    RokaeClient client;
    client.hello();
    client.postTrajectory(trajectory);
    return 0;
}