#ifndef RFWAYPOINT_TIME_H
#define RFWAYPOINT_TIME_H

#include<vector>
#include<array>

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
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
 
    std::array<double,4> quad;
    quad[3] = cy * cp * cr + sy * sp * sr;
    quad[0] = cy * cp * sr - sy * sp * cr;
    quad[1] = sy * cp * sr + cy * sp * cr;
    quad[2] = sy * cp * cr - cy * sp * sr;
    //quad.w = cy * cp * cr + sy * sp * sr;
    //quad.x = cy * cp * sr - sy * sp * cr;
    //quad.y = sy * cp * sr + cy * sp * cr;
    //quad.z = sy * cp * cr - cy * sp * sr;
    //waypoint 0 x;1 y;2 z;3 w
    return quad;
}

#endif