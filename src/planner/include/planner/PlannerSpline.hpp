#ifndef PLANNER_TIME_H
#define PLANNER_TIME_H

#include <planner/PlannerConfiguration.h>
#include <iostream>
#include <planner/PlannerManager.h>
#include <robot_model/RobotModelLoader.h>
#include <planner/PlannerConfiguration.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "controller/trajectory.h"
#include <moveit/kinematic_constraints/utils.h>
#include "controller/trajectory_controller.h"
#include <planner/spline.h>
#include <planner/rfWaypoint.hpp>
#include <robot_model/JointLimitsLoader.h>
#include <controller/trajectory.h>

class PlannerCException
{
public:
    std::string msg;
    PlannerCException(std::string error) : msg(error) {}
};

class PlannerSpline
{
public:
    typedef std::shared_ptr<PlannerSpline> Ptr;
    // robot loader
    RobotModelLoaderPtr loader;

    // robot mode
    robot_model::RobotModelPtr kinematic_model;

    // kinematic loader
    KinematicsLoaderPtr kinematics_loader;

    // planning_scene
    planning_scene::PlanningScenePtr planning_scene;

    // planning_limit
    JointLimitsLoaderPtr joint_limit;

    // ompl config
    PlannerConfigurationPtr pconfig;

    // group name
    std::string groupname;

public:
    explicit PlannerSpline(const std::string &groupname) : groupname(groupname) {}
    
    // create robot_loader
    void loadRobotModel(const std::string &urdf, const std::string &srdf);
    void loadKinematicModel(const std::string &kinematic_path);
    void loadPlannerConfig(const std::string &Planner_path);
    void loadJointLimit(const std::string &jointLimit_path);

    // SYNC your Robot and Moveit Before you use moveit to create you spline traj
    void InitRobotState(rfWaypoint &waypont);
    void InitRobotState(std::vector<double> &JGJointValues, const std::string &group_name);

    void init(const std::string &urdf,
              const std::string &srdf,
              const std::string &kinematic_path,
              const std::string &Planner_path,
              const std::string &jointLimit_path);

    const collision_detection::WorldPtr& AddObject(const std::string& obj_id,
                                                             const shapes::ShapeConstPtr& shape,
                                                             const Eigen::Affine3d& pose3d);
    bool RemoveObject(const std::string& id);

    void clearObjects();
    // CreateSpline based on spline.h
    // 这个API主要功能在于合并多个waypoint点，但还是有问题？
    void CreateSplineSegment(planning_interface::MotionPlanRequest &req,
                             geometry_msgs::PoseStamped &pose,
                             const std::string &tip_name,
                             bool firstflag,
                             robot_state::RobotStatePtr& state);

    void CreateSplineParameterization(std::vector<rfWaypoint> &waypoint,
                                      const std::string &group_name,
                                      const std::string &frame_id,
                                      const std::string &tip_name,
                                      double timeinterval = 0.1,
                                      double max_velocity_scaling_factor = 1.0,
                                      double max_acceleration_scaling_factor = 1.0);

    //这个API功能在于计算一个waypoint点
    void CreateSingleWaypointPath(rfWaypoint &waypoint,
                                  const std::string &group_name,
                                  const std::string &frame_id,
                                  const std::string &tip_name,
                                  double max_velocity_scaling_factor = 1.0,
                                  double max_acceleration_scaling_factor = 1.0);

    void CreateSingleWaypointSegment(planning_interface::MotionPlanRequest &req,
                                     const std::string &tip_name,
                                     geometry_msgs::PoseStamped &pose);

    // ompl 的计时器，但不是spline的计时器
    double time = 0.0;

    // omp的时间容器，但不是spline的时间容器
    std::vector<double> timeslist;
    //获取timesList
    std::vector<double> getTimeList();
    //通过ompl规划出来的joint信息，robot 包含pos acl以及vec信息 但不包含ompl采样
    std::vector<moveit::core::RobotStatePtr> JointsPositionsList;
    // std::vector<std::vector<double>> getJointsPosList();
    //通过节点名获取对应的pos joint acceleration 信息
    std::vector<double> getJointPosValue(const std::string &jointname);
    std::vector<double> getJointVecValue(const std::string &jointname);
    std::vector<double> getJointAclValue(const std::string &Jointname);

    // 还没由开始采样的轨迹
    std::vector<robot_trajectory::RobotTrajectoryPtr> trajectories;
    // 通过sample_by_interval 来获取采样轨迹
    int sample_by_interval(double timeinterval);
    //一共n个节点，包含n个PosVecAccState
    std::map<std::string, trajectory_interface::PosVelAccState<double>> sampleVector;
    trajectory_interface::PosVelAccState<double> get_ompl_sample(const std::string &jointname);

    //通过sample_by_interval 来获取的时间撮
    std::vector<double> sample_by_interval_times;
    std::vector<double> get_sample_by_interval_times();

    // name to idx
    // 名字到索引的映射
    std::map<std::string, int> name_idx;
    int jointIndex(const std::string &joint_name);
    void reset_name_idx(const std::string &groupname);
};

trajectory_interface::PosVelAccState<double> PlannerSpline::get_ompl_sample(const std::string &jointname)
{
    std::map<std::string, trajectory_interface::PosVelAccState<double>>::iterator iter;
    iter = sampleVector.find(jointname);
    return iter->second;
}

void PlannerSpline::init(const std::string &urdf,
                         const std::string &srdf,
                         const std::string &kinematic_path,
                         const std::string &Planner_path,
                         const std::string &jointLimit_path)
{
    this->loadRobotModel(urdf, srdf);
    this->loadKinematicModel(kinematic_path);
    this->loadPlannerConfig(Planner_path);
    this->loadJointLimit(jointLimit_path);
}

int PlannerSpline::jointIndex(const std::string &jointname)
{
    std::map<std::string, int>::iterator iter;
    iter = name_idx.find(jointname);
    if (iter == name_idx.end())
    {
        std::cout << "Error jointname ,please input the right joint name " << std::endl;
        return -1;
    }
    return iter->second;
}

void PlannerSpline::CreateSingleWaypointSegment(planning_interface::MotionPlanRequest &req,
                                                const std::string &tip_name,
                                                geometry_msgs::PoseStamped &pose)
{
    PlannerManager planner(kinematic_model, pconfig);

    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(tip_name, pose, 0.01, 0.01);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);

    planning_interface::PlanningContextPtr context = planner.getPlanningContext(planning_scene, req);
    planning_interface::MotionPlanResponse response;
    context->solve(response);

    trajectory_processing::IterativeParabolicTimeParameterization parameterization;
    parameterization.computeTimeStamps(*response.trajectory_.get(), req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor);

    for (int it = 0; it < (response.trajectory_->getWayPointCount()); it++)
    {
        timeslist.push_back(time = time + response.trajectory_->getWayPointDurationFromPrevious(it));
        JointsPositionsList.push_back(response.trajectory_->getWayPointPtr(it));
    }
}

void PlannerSpline::CreateSingleWaypointPath(rfWaypoint &waypoint,
                                             const std::string &group_name,
                                             const std::string &frame_id,
                                             const std::string &tip_name,
                                             double max_velocity_scaling_factor,
                                             double max_acceleration_scaling_factor)
{
    robot_state::RobotState &robot_state = planning_scene->getCurrentStateNonConst();
    planning_interface::MotionPlanRequest req;
    req.group_name = group_name;
    req.max_velocity_scaling_factor = max_velocity_scaling_factor;
    req.max_acceleration_scaling_factor = max_acceleration_scaling_factor;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;

    JointsPositionsList.clear();
    timeslist.clear();
    time = 0.0;

    pose.pose.position.x = waypoint.trans[0];
    pose.pose.position.y = waypoint.trans[1];
    pose.pose.position.z = waypoint.trans[2];
    pose.pose.orientation.x = waypoint.quad[0];
    pose.pose.orientation.y = waypoint.quad[1];
    pose.pose.orientation.z = waypoint.quad[2];
    pose.pose.orientation.w = waypoint.quad[3];

    this->CreateSingleWaypointSegment(req, tip_name, pose);
}

std::vector<double> PlannerSpline::get_sample_by_interval_times()
{
    return sample_by_interval_times;
}

int PlannerSpline::sample_by_interval(double timeinterval)
{
    
    sample_by_interval_times.clear();
    sampleVector.clear();
    int sample_count=0;
    //在运行的时候先清空时间列表
    for (auto &jointname : name_idx)
    {
        sample_count=0;
        double target_time=0;
        const std::string joint_name=jointname.first;
        std::cout<<joint_name<<std::endl;
        if(joint_name.compare("panda_joint8")!=0)
            sample_by_interval_times.clear();
        trajectory_interface::PosVelAccState<double> Sample= trajectory_interface::PosVelAccState<double>();
     
        for(size_t index=0;index<trajectories.size();index++)
        { 
            //Spline for each trajectory 
            //There are some thing improve
            trajectory_interface::PosVelAccState<double> sample= trajectory_interface::PosVelAccState<double>();
            SplineTrajectory Spline(trajectories[index],false,SplineTrajectory::Parameterization::SPLINE);
            Spline.sample_by_interval(joint_name,sample, timeinterval);
            for(size_t i=0;i<sample.position.size();i++)
            {
                if(joint_name.compare("panda_joint8")!=0)
                    sample_by_interval_times.push_back(target_time);
                Sample.position.push_back(sample.position[i]);
                Sample.velocity.push_back(sample.velocity[i]);
                Sample.acceleration.push_back(sample.acceleration[i]);
                sample_count++;
                target_time+=timeinterval;
            }
        }
        sampleVector.insert(std::make_pair(joint_name,Sample));
    }

    return sample_count;
}

const collision_detection::WorldPtr& PlannerSpline::AddObject(const std::string& obj_id,const shapes::ShapeConstPtr& shape,const Eigen::Affine3d& pose3d)
{
    const collision_detection::WorldPtr& world=this->planning_scene->getWorldNonConst();
    world->addToObject(obj_id,shape,pose3d);
    return world;
}

bool PlannerSpline::RemoveObject(const std::string& id)
{   
    const collision_detection::WorldPtr& world=this->planning_scene->getWorldNonConst();
    world->removeObject(id);
    return true;
}

void PlannerSpline::clearObjects()
{
    const collision_detection::WorldPtr& world=this->planning_scene->getWorldNonConst();
    world->clearObjects();
}


// 一系列登陆模型的对象
void PlannerSpline::loadRobotModel(const std::string &urdf, const std::string &srdf)
{
    loader = createRobotModelLoaderFromFile(urdf, srdf);
    kinematic_model = loader->getRobotModel();
}

void PlannerSpline::loadJointLimit(const std::string &jointLimit_path)
{
    joint_limit = createJointLimitsLoaderFromFile(jointLimit_path);
    loader->loadJointLimits(joint_limit);
}

void PlannerSpline::loadKinematicModel(const std::string &kinematic_path)
{
    kinematics_loader = createKinematicsLoaderFromFile(kinematic_path);
    loader->loadKinematicsSolvers(kinematics_loader);
}

void PlannerSpline::loadPlannerConfig(const std::string &Planner_path)
{
    pconfig = createPlannerConfigurationFromFile(Planner_path);
    // PlannerManager planner(kinematic_model, pconfig);
    planning_scene = loader->newPlanningScene();
    robot_state::RobotState &robot_state = planning_scene->getCurrentStateNonConst();
    std::vector<std::string> namelist = robot_state.getJointModelGroup(groupname)->getJointModelNames();
    int jointidx = 0;
    for (std::vector<std::string>::iterator it = namelist.begin(); it != namelist.end(); it++)
    {
        name_idx.insert(std::make_pair(*it, jointidx));
        jointidx++;
    }
}

void PlannerSpline::reset_name_idx(const std::string &groupname)
{
    this->name_idx.clear();
    robot_state::RobotState &robot_state = planning_scene->getCurrentStateNonConst();
    std::vector<std::string> namelist = robot_state.getJointModelGroup(groupname)->getJointModelNames();
    int jointidx = 0;
    for (std::vector<std::string>::iterator it = namelist.begin(); it != namelist.end(); it++)
    {
        name_idx.insert(std::make_pair(*it, jointidx));
        jointidx++;
    }
}
// 还没有实现
void PlannerSpline::InitRobotState(rfWaypoint &waypont)
{
}

//初始化到一个初始化的robot state状态
void PlannerSpline::InitRobotState(std::vector<double> &JGJointValues, const std::string &group_name)
{
    moveit::core::JointModelGroup *joint_model_group = loader->getRobotModel()->getJointModelGroup(group_name);
    robot_state::RobotState &robot_state = planning_scene->getCurrentStateNonConst();
    robot_state.setJointGroupPositions(joint_model_group, JGJointValues);
}

void PlannerSpline::CreateSplineSegment(planning_interface::MotionPlanRequest &req,
                                        geometry_msgs::PoseStamped &pose,
                                        const std::string &tip_name,
                                        bool firstflag,
                                        robot_state::RobotStatePtr& state)
{
    PlannerManager planner(kinematic_model, pconfig);
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(
        tip_name,
        pose,
        0.01,
        0.01);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);

    planning_interface::PlanningContextPtr context = planner.getPlanningContext(planning_scene, req);
    planning_interface::MotionPlanResponse response;
    context->solve(response);

    //获取规划后的对象
    trajectory_processing::IterativeSplineParameterization parameterization;
    if(firstflag){
        parameterization.computeTimeStamps(*response.trajectory_.get(), req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor);
    }else{

        //衔接上一次好，还改变结尾好，明天在试试
        response.trajectory_->changPrefixWayPoint(state,response.trajectory_->getWayPointDurationFromPrevious(0)+100);
        parameterization.computeTimeStamps(*response.trajectory_.get(), req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor);
    }

    for (int it = 0; it < (response.trajectory_->getWayPointCount()); it++)
    {
        timeslist.push_back(time = time + response.trajectory_->getWayPointDurationFromPrevious(it));
        JointsPositionsList.push_back(response.trajectory_->getWayPointPtr(it));
    }
    trajectories.push_back(response.trajectory_);
}

void PlannerSpline::CreateSplineParameterization(std::vector<rfWaypoint> &waypoint,
                                                 const std::string &group_name,
                                                 const std::string &frame_id,
                                                 const std::string &tip_name,
                                                 double timeintervel,
                                                 double max_velocity_scaling_factor,
                                                 double max_acceleration_scaling_factor)
{
    robot_state::RobotState &robot_state = planning_scene->getCurrentStateNonConst();
    planning_interface::MotionPlanRequest req;

    req.group_name = group_name;
    this->reset_name_idx(group_name);
    req.max_velocity_scaling_factor = max_velocity_scaling_factor;
    req.max_acceleration_scaling_factor = max_acceleration_scaling_factor;

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;

    trajectories.clear();
    JointsPositionsList.clear();
    timeslist.clear();
    time = 0.0;

    for (std::vector<rfWaypoint>::const_iterator it = waypoint.begin(); it != waypoint.end(); it++)
    {
        pose.pose.position.x = it->trans[0];
        pose.pose.position.y = it->trans[1];
        pose.pose.position.z = it->trans[2];
        pose.pose.orientation.x = it->quad[0];
        pose.pose.orientation.y = it->quad[1];
        pose.pose.orientation.z = it->quad[2];
        pose.pose.orientation.w = it->quad[3];
        if (it == waypoint.begin())
            CreateSplineSegment(req, pose, tip_name, true, JointsPositionsList[JointsPositionsList.size() - 1]);
        else
        {
            CreateSplineSegment(req, pose, tip_name, false, JointsPositionsList[JointsPositionsList.size() - 1]);
        }
        robot_state = *JointsPositionsList[JointsPositionsList.size() - 1].get();
        planning_scene->setCurrentState(robot_state);
    }
}

std::vector<double> PlannerSpline::getTimeList()
{
    return this->timeslist;
}

std::vector<double> PlannerSpline::getJointPosValue(const std::string &jointname)
{
    std::vector<double> jointPos;
    std::vector<moveit::core::RobotStatePtr>::iterator iter = JointsPositionsList.begin();
    for (; iter != JointsPositionsList.end(); ++iter)
    {
        jointPos.push_back((*iter)->getVariablePosition(jointname));
    }
    return jointPos;
}

std::vector<double> PlannerSpline::getJointVecValue(const std::string &jointname)
{
    std::vector<double> vecValue;
    std::vector<moveit::core::RobotStatePtr>::iterator iter = JointsPositionsList.begin();
    for (; iter != JointsPositionsList.end(); ++iter)
    {
        vecValue.push_back((*iter)->getVariableVelocity(jointname));
    }
    return vecValue;
}

std::vector<double> PlannerSpline::getJointAclValue(const std::string &jointname)
{
    std::vector<double> AcePos;
    std::vector<moveit::core::RobotStatePtr>::iterator iter = JointsPositionsList.begin();
    for (; iter != JointsPositionsList.end(); ++iter)
    {
        AcePos.push_back((*iter)->getVariableAcceleration(jointname));
    }
    return AcePos;
}

#endif