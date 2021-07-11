//
// Created by yongxi on 2021/7/9.
//

#ifndef MOVEIT_NO_ROS_HARDWARE_BULLET3_H
#define MOVEIT_NO_ROS_HARDWARE_BULLET3_H

#include "SharedMemory/SharedMemoryPublic.h" // Definition of connection method
//#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h"
//#include "SharedMemory/PhysicsClientSharedMemory.h"
#include "controller/hardware.h"
#include <iostream>

/**
 * @deprecated It seems that we cannot reach the robot info from bullet shared_memory client.
 * The role of this class has been replaced by 'python/include/hardware_pybullet.h/PybulletHardware'
 */
class Bullet3Hardware {
public:
    Bullet3Hardware(int key);
    ~Bullet3Hardware() {
        delete client;
    }
    bool isConnected() {
        return client->isConnected();
    }

    int getNumBodys() {
        return client->getNumBodies();
    }
    int getNumJoints(int bodyId) {
        std::cerr << "WARN: Do not call getNumJoints from cpp. This func is used for debugging only." << std::endl;
        std::cout << "getNumJoints(" << client->getBodyUniqueId(bodyId) << ')' << std::endl;
        return client->getNumJoints(client->getBodyUniqueId(bodyId));
    }

    void debugGravity() {
        client->setGravity(btVector3(0,10,10));
        return;
    }
private:
    b3RobotSimulatorClientAPI_NoGUI* client;
    //PhysicsClientSharedMemory* client;
};

#endif //MOVEIT_NO_ROS_HARDWARE_BULLET3_H
