//
// Created by yongxi on 2021/7/11.
//

#include "hardware_pybullet.h"

PybulletHardware::PybulletHardware(py::handle pybullet)
        :pybullet_(py::reinterpret_borrow<py::object>(pybullet))
        , getNumBodies_(pybullet_.attr("getNumBodies"))
        , getJointInfo_(pybullet_.attr("getJointInfo")){
}

int PybulletHardware::getNumBodies() {
    return py::cast<int>(getNumBodies_());
}
