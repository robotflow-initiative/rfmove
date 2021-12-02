//
// Created by yongxi on 6/22/21.
//
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


class PlannerCException{
public:
    std::string msg;
    PlannerCException(std::string error):msg(error){}
};


class PlannerSpline{
public:
    
    //robot loader 
    RobotModelLoaderPtr loader;

    //robot mode
    robot_model::RobotModelPtr kinematic_model;

    //kinematic loader
    KinematicsLoaderPtr kinematics_loader;

    //planning_scene
    planning_scene::PlanningScenePtr planning_scene;

    //planning_limit
    JointLimitsLoaderPtr joint_limit;
    
    //ompl config
    PlannerConfigurationPtr pconfig;

     //group name
    std::string groupname;
public:
    explicit PlannerSpline(const std::string& groupname):groupname(groupname){}

    //create robot_loader
    void loadRobotModel(const std::string& urdf,const std::string& srdf);
    void loadKinematicModel(const std::string& kinematic_path);
    void loadPlannerConfig(const std::string& Planner_path);
    void loadJointLimit(const std::string& jointLimit_path);

    //SYNC your Robot and Moveit Before you use moveit to create you spline traj
    void InitRobotState(rfWaypoint& waypont);
    void InitRobotState(std::vector<double> &JGJointValues,const std::string &group_name);

    void init(const std::string& urdf,
              const std::string& srdf,
              const std::string& kinematic_path,
              const std::string& Planner_path,
              const std::string& jointLimit_path);
    
 
    
    // CreateSpline based on spline.h
    // 这个API主要功能在于合并多个waypoint点，但还是有问题？
    void CreateSplineSegment(planning_interface::MotionPlanRequest &req,
                             geometry_msgs::PoseStamped &pose,
                             const std::string &tip_name,
                             bool firstflag,
                             double timeinterval);

    void CreateSplineParameterization(std::vector<rfWaypoint>& waypoint,
                                      const std::string &group_name,
                                      const std::string &frame_id,
                                      const std::string &tip_name,
                                      double timeinterval=0.1,
                                      double max_velocity_scaling_factor=1.0,
                                      double max_acceleration_scaling_factor=1.0);


    //这个API功能在于计算一个waypoint点
    void CreateSingleWaypointPath(rfWaypoint& waypoint,
                                  const std::string &group_name,
                                  const std::string &frame_id,
                                  const std::string &tip_name,
                                  double max_velocity_scaling_factor=1.0,
                                  double max_acceleration_scaling_factor=1.0);

    void CreateSingleWaypointSegment(planning_interface::MotionPlanRequest &req,
                                     const std::string &tip_name,
                                     geometry_msgs::PoseStamped &pose);

    //ompl 的计时器，但不是spline的计时器
    double time=0.0;

    //omp的时间容器，但不是spline的时间容器
    std::vector<double> timeslist;
    //获取timesList
    std::vector<double> getTimeList();
    //通过ompl规划出来的joint信息，robot 包含pos acl以及vec信息 但不包含ompl采样
    std::vector<moveit::core::RobotStatePtr> JointsPositionsList;
    //std::vector<std::vector<double>> getJointsPosList();

    //通过节点名获取对应的pos joint acceleration 信息
    //ompl轨迹
    // 1 fail
    // 0 success
    std::vector<double> getJointPosValue(const std::string& jointname);
    std::vector<double> getJointVecValue(const std::string& jointname);
    std::vector<double> getJointAclValue(const std::string& Jointname);

    // 通过sample_by_interval 来获取采样轨迹
    int sample_by_interval(double timeinterval);
    std::vector<double> sample_by_time(double time,const std::string& jointname);

    //一共n个节点，包含n个PosVecAccState
    std::map<std::string,trajectory_interface::PosVelAccState<double>> sampleVector;

    trajectory_interface::PosVelAccState<double> sample;
    trajectory_interface::PosVelAccState<double> get_ompl_sample(const std::string& jointname);

    //通过sample_by_interval 来获取的时间撮
    std::vector<double> sample_by_interval_times;
    std::vector<double> get_sample_by_interval_times();


    // name to idx
    // 名字到索引的映射
    std::map<std::string,int>  name_idx;
    int jointIndex(const std::string& joint_name);
};


trajectory_interface::PosVelAccState<double> PlannerSpline::get_ompl_sample(const std::string& jointname)
{
    std::map<std::string,trajectory_interface::PosVelAccState<double>>::iterator iter;    
    iter=sampleVector.find(jointname);
    return iter->second;
}

void PlannerSpline::init(const std::string& urdf,
                         const std::string& srdf,
                         const std::string& kinematic_path,
                         const std::string& Planner_path,
                         const std::string& jointLimit_path)
{
    this->loadRobotModel(urdf,srdf);
    this->loadKinematicModel(kinematic_path);
    this->loadPlannerConfig(Planner_path);
    this->loadJointLimit(jointLimit_path);
}

int PlannerSpline::jointIndex(const std::string& jointname)
{
    std::map<std::string, int>::iterator iter;
    iter=name_idx.find(jointname);
    if(iter==name_idx.end())
    {
        std::cout<<"Error jointname ,please input the right joint name "<<std::endl;
        return -1;
    }
    return iter->second;
}



void PlannerSpline::CreateSingleWaypointSegment(planning_interface::MotionPlanRequest &req,
                                                const std::string &tip_name,
                                                geometry_msgs::PoseStamped &pose)
{
    PlannerManager planner(kinematic_model,pconfig);

    moveit_msgs::Constraints pose_goal=kinematic_constraints::constructGoalConstraints(tip_name, pose,0.01, 0.01);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);

    planning_interface::PlanningContextPtr context=planner.getPlanningContext(planning_scene,req);
    planning_interface::MotionPlanResponse response;
    context->solve(response);

    trajectory_processing::IterativeParabolicTimeParameterization parameterization;
    parameterization.computeTimeStamps(*response.trajectory_.get(),req.max_velocity_scaling_factor,req.max_acceleration_scaling_factor);

    JointsPositionsList.clear();
    timeslist.clear();

    for(int it=0;it<(response.trajectory_->getWayPointCount());it++)
    {
        timeslist.push_back(time=time+response.trajectory_->getWayPointDurationFromPrevious(it));
        JointsPositionsList.push_back(response.trajectory_->getWayPointPtr(it));
        
    }
}

 void PlannerSpline::CreateSingleWaypointPath(rfWaypoint& waypoint,
                                              const std::string &group_name,
                                              const std::string &frame_id,
                                              const std::string &tip_name,
                                              double max_velocity_scaling_factor,
                                              double max_acceleration_scaling_factor)
{
    robot_state::RobotState& robot_state=planning_scene->getCurrentStateNonConst();
    planning_interface::MotionPlanRequest req;
    req.group_name=group_name;
    req.max_velocity_scaling_factor=max_velocity_scaling_factor;
    req.max_acceleration_scaling_factor=max_acceleration_scaling_factor;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id=frame_id;
    time=0.0;

    pose.pose.position.x=waypoint.trans[0];
    pose.pose.position.y=waypoint.trans[1];
    pose.pose.position.z=waypoint.trans[2];
    pose.pose.orientation.x=waypoint.quad[0];
    pose.pose.orientation.y=waypoint.quad[1];
    pose.pose.orientation.z=waypoint.quad[2];
    pose.pose.orientation.w=waypoint.quad[3];

    this->CreateSingleWaypointSegment(req,tip_name,pose);
}

std::vector<double> PlannerSpline::get_sample_by_interval_times()
{
    return sample_by_interval_times;
}
int PlannerSpline::sample_by_interval(double timeinterval)
{
    sampleVector.clear();
    std::vector<double> PosVecAcl;
    int sample_count=0;
    //在运行的时候先清空时间列表
    sample_by_interval_times.clear();
    for(auto& jointname:name_idx)
    {
        sample = trajectory_interface::PosVelAccState<double>();
        PosVecAcl.clear();
       
        sample_count=0;
        if(JointsPositionsList.empty())
        {
            std::cout<<"JointsPosition is not inital!"<<std::endl;
            return -1;
        }
        //首个时间
        double start_time=timeslist[0];
        //末尾时间点
        double end_time=timeslist[timeslist.size()-1];
        try{
        for(double target_time=start_time;target_time<=end_time;target_time+=timeinterval)
        {
                
            PosVecAcl=sample_by_time(target_time,jointname.first);
            if(PosVecAcl.empty())
            {
                std::cout<<"Sample is fail at time: "<<target_time<<std::endl;
                return -1;
            }

            //仅仅在刚开始的时候清空列表，然后初始化一次sample_by_interval_times
            if(!jointname.second)
            {
                sample_by_interval_times.push_back(target_time);
            }
            
            sample.position.push_back(PosVecAcl[0]);
            sample.velocity.push_back(PosVecAcl[1]);
            sample.acceleration.push_back(PosVecAcl[2]);
            ++sample_count;
        }
        sampleVector.insert(std::make_pair(jointname.first,sample));
        }catch(...)
        {
            std::cout<<"Joint name is error"<<std::endl;
            return -1;
        }
    }
    return sample_count;
}

// 设返回值，0是pos 1是vec 2 is acl
std::vector<double> PlannerSpline::sample_by_time(double time,const std::string& jointname)
{
    std::vector<double> PosVecAcl;
    double deltatime;
    double deltapos;
    double slope;
    double compute_time;
    //我们需要看时间段，因此减去最后一个元素
    for(int index_time=0;index_time<(timeslist.size()-1);index_time++)
    {
        if(time>timeslist[index_time+1])
        {
            continue;
        }
        if(time<=timeslist[index_time+1]&&time>=timeslist[index_time])
        {
            //compute delta time
            deltatime=timeslist[index_time+1]-timeslist[index_time];
            deltatime+=std::numeric_limits<double>::epsilon();  // prevent divide-by-zero

            //compute pos sample
            deltapos=JointsPositionsList[index_time+1]->getVariablePosition(jointname)-JointsPositionsList[index_time]->getVariablePosition(jointname);
            slope=deltapos/deltatime;
            compute_time=time-timeslist[index_time];
            PosVecAcl.push_back(slope*compute_time+JointsPositionsList[index_time]->getVariablePosition(jointname));

            //compute vec sample
            deltapos=JointsPositionsList[index_time+1]->getVariableVelocity(jointname)-JointsPositionsList[index_time]->getVariableVelocity(jointname);
            slope=deltapos/deltatime;
            compute_time=time-timeslist[index_time];
            PosVecAcl.push_back(slope*compute_time+JointsPositionsList[index_time]->getVariableVelocity(jointname));

            //compute acl sample
            deltapos=JointsPositionsList[index_time+1]->getVariableAcceleration(jointname)-JointsPositionsList[index_time]->getVariableAcceleration(jointname);
            slope=deltapos/deltatime;
            compute_time=time-timeslist[index_time];
            PosVecAcl.push_back(slope*compute_time+JointsPositionsList[index_time]->getVariableAcceleration(jointname));
            return PosVecAcl;
        } 
    }
    return PosVecAcl;
}

// 一系列登陆模型的对象
void PlannerSpline::loadRobotModel(const std::string& urdf,const std::string& srdf)
{
    loader = createRobotModelLoaderFromFile(urdf,srdf);
    kinematic_model = loader->getRobotModel();
}

void PlannerSpline::loadJointLimit(const std::string& jointLimit_path)
{
    joint_limit=createJointLimitsLoaderFromFile(jointLimit_path);
    loader->loadJointLimits(joint_limit);
}

void PlannerSpline::loadKinematicModel(const std::string& kinematic_path)
{
    kinematics_loader = createKinematicsLoaderFromFile(kinematic_path);
    loader->loadKinematicsSolvers(kinematics_loader);
}

void PlannerSpline::loadPlannerConfig(const std::string& Planner_path)
{
    pconfig = createPlannerConfigurationFromFile(Planner_path);
    //PlannerManager planner(kinematic_model, pconfig);
    planning_scene=loader->newPlanningScene();
    robot_state::RobotState& robot_state=planning_scene->getCurrentStateNonConst();
    
    std::vector<std::string> namelist=robot_state.getJointModelGroup(groupname)->getJointModelNames();
    int jointidx=0;
    for(std::vector<std::string>::iterator it=namelist.begin();it!=namelist.end();it++)
    {
        //std::cout<<"name is "<<*it<<std::endl;
        std::vector<double> joint_value_tmp;
        name_idx.insert(std::make_pair(*it,jointidx));
        jointidx++;
    }
}
// 还没有实现
void PlannerSpline::InitRobotState(rfWaypoint& waypont)
{

}

//初始化到一个初始化的robot state状态
void PlannerSpline::InitRobotState(std::vector<double> &JGJointValues,const std::string &group_name)
{
    moveit::core::JointModelGroup* joint_model_group = loader->getRobotModel()->getJointModelGroup(group_name);    
    robot_state::RobotState& robot_state=planning_scene->getCurrentStateNonConst();
    robot_state.setJointGroupPositions(joint_model_group,JGJointValues);
}


void PlannerSpline::CreateSplineSegment(planning_interface::MotionPlanRequest &req,
                                        geometry_msgs::PoseStamped &pose,
                                        const std::string &tip_name,
                                        bool firstflag,
                                        double timeintervel)
{
    PlannerManager planner(kinematic_model, pconfig);
    moveit_msgs::Constraints pose_goal=kinematic_constraints::constructGoalConstraints(
        tip_name,
        pose,
        0.01,
        0.01
    );
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);
    //std::cout<<"req.max_velocity_scaling_factor::"<<req.max_velocity_scaling_factor<<std::endl;
    //std::cout<<"max_acceleration_scaling_factor::"<<req.max_acceleration_scaling_factor<<std::endl;
    planning_interface::PlanningContextPtr context=planner.getPlanningContext(planning_scene,req);
    planning_interface::MotionPlanResponse response;
    context->solve(response);

    //获取规划后的对象
    trajectory_processing::IterativeParabolicTimeParameterization parameterization;
    parameterization.computeTimeStamps(*response.trajectory_.get(),req.max_velocity_scaling_factor,req.max_acceleration_scaling_factor);

    
    for(int it=0;it<(response.trajectory_->getWayPointCount());it++)
    {
        if(!it)
        {
            if(firstflag)  //firstflat 判断为true时说明是整个waypoints path第一个点
            {
                timeslist.push_back(time);
                JointsPositionsList.push_back(response.trajectory_->getWayPointPtr(it));
                time=time+response.trajectory_->getWayPointDurationFromPrevious(it);
            }else
            {
                time=time+response.trajectory_->getWayPointDurationFromPrevious(it);
                continue;
            }
        }else if(time-timeslist[timeslist.size()-1]<=timeintervel)  //setup the timeinterval
        {
            //如果是最后一个点的话，转而删除前面一个点
            if(it==response.trajectory_->getWayPointCount()-1)
            {
                timeslist.pop_back();
                JointsPositionsList.pop_back();
                timeslist.push_back(time);
                JointsPositionsList.push_back(response.trajectory_->getWayPointPtr(it));
                time=time+response.trajectory_->getWayPointDurationFromPrevious(it);
                continue;
            }else{
                time=time+response.trajectory_->getWayPointDurationFromPrevious(it);
                continue;
            }
        }else
        {
            timeslist.push_back(time);
            JointsPositionsList.push_back(response.trajectory_->getWayPointPtr(it));         
            time=time+response.trajectory_->getWayPointDurationFromPrevious(it);
        }
    }
}


void PlannerSpline::CreateSplineParameterization(std::vector<rfWaypoint>& waypoint,
                                                const std::string &group_name,
                                                const std::string &frame_id,
                                                const std::string &tip_name,
                                                double timeintervel,
                                                double max_velocity_scaling_factor,
                                                double max_acceleration_scaling_factor)
{
    robot_state::RobotState& robot_state=planning_scene->getCurrentStateNonConst();
    planning_interface::MotionPlanRequest req;

    req.group_name=group_name;
    
    req.max_velocity_scaling_factor=max_velocity_scaling_factor;
    req.max_acceleration_scaling_factor=max_acceleration_scaling_factor;

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id=frame_id;
    time=0.0;

    for(std::vector<rfWaypoint>::const_iterator it=waypoint.begin();it!=waypoint.end();it++)
    {
        if(it==waypoint.begin())
        {
            pose.pose.position.x=it->trans[0];
            pose.pose.position.y=it->trans[1];
            pose.pose.position.z=it->trans[2];
            pose.pose.orientation.x=it->quad[0];
            pose.pose.orientation.y=it->quad[1];
            pose.pose.orientation.z=it->quad[2];
            pose.pose.orientation.w=it->quad[3];
            //hanlde with the first waypoint
            CreateSplineSegment(req,pose,tip_name,true,timeintervel);
        }else{
            pose.pose.position.x=it->trans[0];
            pose.pose.position.y=it->trans[1];
            pose.pose.position.z=it->trans[2];
            pose.pose.orientation.x=it->quad[0];
            pose.pose.orientation.y=it->quad[1];
            pose.pose.orientation.z=it->quad[2];
            pose.pose.orientation.w=it->quad[3];
            CreateSplineSegment(req,pose,tip_name,false,timeintervel);
        }

        moveit::core::JointModelGroup* joint_model_group = loader->getRobotModel()->getJointModelGroup(group_name);    
        robot_state=*JointsPositionsList[JointsPositionsList.size()-1].get();
        planning_scene->setCurrentState(robot_state);
    }
}

std::vector<double> PlannerSpline::getTimeList()
{
    return this->timeslist;
}

std::vector<double> PlannerSpline::getJointPosValue(const std::string& jointname)
{
    std::vector<double> jointPos;
    std::vector<moveit::core::RobotStatePtr>::iterator iter = JointsPositionsList.begin();
    for(; iter!= JointsPositionsList.end(); ++iter){
        jointPos.push_back((*iter)->getVariablePosition(jointname));
    }
    return jointPos;
}

std::vector<double> PlannerSpline::getJointVecValue(const std::string& jointname)
{
    std::vector<double> vecValue;
    std::vector<moveit::core::RobotStatePtr>::iterator iter = JointsPositionsList.begin();
    for(; iter!= JointsPositionsList.end(); ++iter){
        vecValue.push_back((*iter)->getVariableVelocity(jointname));
    }
    return vecValue;
}

std::vector<double> PlannerSpline::getJointAclValue(const std::string& jointname)
{
    std::vector<double> AcePos;
    std::vector<moveit::core::RobotStatePtr>::iterator iter = JointsPositionsList.begin();
    for(; iter!= JointsPositionsList.end(); ++iter){
        AcePos.push_back((*iter)->getVariableAcceleration(jointname));
    }
    return AcePos;
}


#endif