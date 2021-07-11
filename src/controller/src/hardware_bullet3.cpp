//
// Created by yongxi on 2021/7/9.
//

#include "controller/hardware_bullet3.h"
#include <iostream>

Bullet3Hardware::Bullet3Hardware(int key) {
    //client = new PhysicsClientSharedMemory();
    client = new b3RobotSimulatorClientAPI_NoGUI();
    if(!client->connect(eCONNECT_SHARED_MEMORY, "localhost", key)) {
    //if(!client->connect()){
        std::cerr << "ERROR: Connect to bullet3 server failed." << std::endl;
    }
    //client->getBodyUniqueId()
}