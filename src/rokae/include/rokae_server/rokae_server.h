//
// Created by yongxi on 2021/6/29.
//

#ifndef MOVEIT_NO_ROS_ROKAE_SERVER_H
#define MOVEIT_NO_ROS_ROKAE_SERVER_H

#include <httplib.h>
#include <json.hpp>
#include <vector>
#include <rokae_server/rokae_trajectory.h>

/**
 * RokaeServer is used as server running on rokae robot.
 * @details RokaeServer only use head only libraries.
 */
class RokaeServer {
public:
    /**
     * Callback function which would be called when receive a trajectory info.
     * @details Trajectory info contains the name of joint model group, the name and order of all joints,
     * and the number of waypoints;
     */
    typedef void(*TrajectoryInfoCb)(RobotTrajectoryInfoPtr);

    /**
     * Callback function which would be called when receive a trajectory waypoint.
     * @details Trajectory waypoint contains velocity, position, and effort info for all joints of joint model group.
     * The order is the same as 'joint_names' in RobotTrajectoryInfo.
     */
    typedef void(*TrajectoryWaypointCb)(RobotTrajectoryWaypointPtr);

    /**
     * Callback function which would be called when receive the end signal of one robot trajectory.
     */
    typedef void(*TrajectoryEndCb)(const std::string&);

    /**
     *
     * @param host Listening host.
     * @param port Listening port.
     */
    explicit RokaeServer(const std::string &host = "0.0.0.0", int port = 8080);

    /**
     * Start listening.
     * @note This function would block the current thread.
     */
    void start();

    /**
     * Set callback function for trajectory info.
     */
     void setInfoCallback(TrajectoryInfoCb);

     /**
      * Set callback function for trajectory waypoint.
      */
     void setWaypointCallback(TrajectoryWaypointCb);

     /**
      * Set callback function for trajectory end.
      */
     void setEndCallback(TrajectoryEndCb);

private:
    std::string host_;
    int port_;
    httplib::Server server_;
    static void hello(const httplib::Request&, httplib::Response &res);
    httplib::Server::Handler waypointHandler();
    httplib::Server::Handler infoHandler();
    httplib::Server::Handler endHandler();
    TrajectoryInfoCb info_cb_;
    TrajectoryWaypointCb waypoint_cb_;
    TrajectoryEndCb end_cb_;
};

#endif //MOVEIT_NO_ROS_ROKAE_SERVER_H
