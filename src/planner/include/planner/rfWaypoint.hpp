#ifndef RFWAYPOINT_TIME_H
#define RFWAYPOINT_TIME_H

#include<vector>
#include<array>
#include<Eigen/Dense>
#include<Eigen/Geometry>

class rfWaypoint{

public:
    std::array<double,3> trans;
    std::array<double,3> eular;
    std::array<double,4> quad;

public:
    explicit rfWaypoint(const std::array<double,3> &trans,const std::array<double,3> &eular);
    std::array<double,4>  ToQuaternion(double yaw, double pitch, double roll);
    
};

rfWaypoint::rfWaypoint(const std::array<double,3> &trans,const std::array<double,3> &eular){ 
            this->trans=trans;
            this->eular=eular;
            this->quad=ToQuaternion(eular[2],eular[1],eular[0]);  // yaw (Z)eular2, pitch (Y)eular1, roll (X)eular0
}

std::array<double,4> rfWaypoint::ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    Eigen::Quaterniond qua(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
                           Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*
                           Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX()).matrix());

    qua.normalize();
    std::array<double,4> quad;
    quad[3] = qua.w();
    quad[0] = qua.x();
    quad[1] = qua.y();
    quad[2] = qua.z();
    return quad;
}

#endif