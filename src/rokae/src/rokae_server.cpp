//
// Created by yongxi on 2021/6/29.
//

#include <rokae_server/rokae_server.h>
#include <sstream>

using json = nlohmann::json;

static inline void noCallBack(httplib::Response& response, const std::string& path) {
    std::cerr << "Callback function for " << path << " has not been initialized." << std::endl;
    response.status = 503;
    std::ostringstream sstr;
    sstr << "Callback function for " << path << " has not been initialized.";
    response.set_content(sstr.str(), "text/plain");
}

RokaeServer::RokaeServer(const std::string &host, int port) {
    host_ = host;
    port_ = port;
    server_.Get("/hi", hello);

    // Initialize router
    server_.Post("/trajectory/info", infoHandler());
    server_.Post("/trajectory/waypoint", waypointHandler());
    server_.Post("/trajectory/end", endHandler());

    // Set callback to null
    info_cb_ = nullptr;
    waypoint_cb_ = nullptr;
    end_cb_ = nullptr;
}

httplib::Server::Handler RokaeServer::infoHandler() {
    return [this](const httplib::Request& req, httplib::Response& res){
        if (info_cb_ == nullptr){
            noCallBack(res, "/trajectory/info");
        } else {
            json info = json::parse(req.body);
            RobotTrajectoryInfoPtr trajectoryInfo(new RobotTrajectoryInfo {
                    .group_name =  info["group_name"],
                    .waypoint_count = info["waypoint_count"],
                    .joint_names = info["joint_names"]
            });
            info_cb_(std::move(trajectoryInfo));
            res.set_content("Trajectory info received", "text/plain");
        }
    };
}

httplib::Server::Handler RokaeServer::waypointHandler() {
    return [this](const httplib::Request& req, httplib::Response& res) -> void{
        if (waypoint_cb_ == nullptr) {
            noCallBack(res, "/trajectory/waypoint");
        } else {
            json waypoint = json::parse(req.body);
            RobotTrajectoryWaypointPtr trajectoryWaypoint(new RobotTrajectoryWaypoint {
                .position = waypoint["positions"],
                .velocity = waypoint["velocities"],
                .effort = waypoint["effort"]
            });
            waypoint_cb_(std::move(trajectoryWaypoint));
            res.set_content("Trajectory waypoint received", "text/plain");
        }
    };
}

httplib::Server::Handler RokaeServer::endHandler() {
    return [this](const httplib::Request& req, httplib::Response& res){
        if (end_cb_ == nullptr) {
            noCallBack(res, "/trajectory/end");
        } else {
            end_cb_(req.body);
            res.set_content("Trajectory ends.", "text/plain");
        }
    };
}

void RokaeServer::start() {
    server_.listen(host_.c_str(), port_);
}

void RokaeServer::hello(const httplib::Request &, httplib::Response &res) {
    res.set_content("Hello!", "text/plain");
}

void RokaeServer::setInfoCallback(TrajectoryInfoCb info_cb) {
    info_cb_ = info_cb;
}

void RokaeServer::setWaypointCallback(TrajectoryWaypointCb waypoint_cb) {
    waypoint_cb_ = waypoint_cb;
}

void RokaeServer::setEndCallback(TrajectoryEndCb end_cb) {
    end_cb_ = end_cb;
}