#include "tobor_left_arm_group_ikfast_moveit_plugin.h"
#include "ikfast70_tobor_arm_group.cpp"

namespace tobor_left_arm_group
{


bool IKFastKinematicsPlugin::initialize(rdf_loader::RDFLoaderPtr rdf_loader, const std::string& group_name,
                                     const std::string& base_frame, const std::string& tip_frame,
                                     double search_discretization)
{
    // setValues set the values in base class.
    // It does not publish values to parameter server (Thanks god).
    setValues("DummyDescription", group_name, base_frame, tip_frame, search_discretization);

    fillFreeParams(GetNumFreeParameters(), GetFreeParameters());

    if (free_params_.size() > 1)
    {
      ROS_FATAL("Only one free joint parameter supported!");
      return false;
    }
    else if (free_params_.size() == 1)
    {
      redundant_joint_indices_.clear();
      redundant_joint_indices_.push_back(free_params_[0]);
      KinematicsBase::setSearchDiscretization(DEFAULT_SEARCH_DISCRETIZATION);
    }

    const srdf::ModelSharedPtr& srdf = rdf_loader -> getSRDF();
    const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader -> getURDF();

    if (!urdf_model || !srdf)
    {
        ROS_ERROR_NAMED("kdl", "URDF and SRDF must be loaded for KDL kinematics solver to work.");
        return false;
    }

    robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));


    urdf::LinkConstSharedPtr link = urdf_model->getLink(getTipFrame());
    while (link->name != base_frame_ && joint_names_.size() <= num_joints_)
    {
   
        link_names_.push_back(link->name);
        urdf::JointSharedPtr joint = link->parent_joint;
        if (joint)
        {
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
            {
              ROS_DEBUG_STREAM_NAMED(name_, "Adding joint " << joint->name);

              joint_names_.push_back(joint->name);
              float lower, upper;
              int hasLimits;
              if (joint->type != urdf::Joint::CONTINUOUS)
              {
                if (joint->safety)
                {
                  lower = joint->safety->soft_lower_limit;
                  upper = joint->safety->soft_upper_limit;
                }
                else
                {
                  lower = joint->limits->lower;
                  upper = joint->limits->upper;
                }
                hasLimits = 1;
              }
              else
              {
                lower = -M_PI;
                upper = M_PI;
                hasLimits = 0;
              }
              if (hasLimits)
              {
                joint_has_limits_vector_.push_back(true);
                joint_min_vector_.push_back(lower);
                joint_max_vector_.push_back(upper);
              }
              else
              {
                joint_has_limits_vector_.push_back(false);
                joint_min_vector_.push_back(-M_PI);
                joint_max_vector_.push_back(M_PI);
              }
            }
          }
      else
      {
        ROS_WARN_NAMED(name_, "no joint corresponding to %s", link->name.c_str());
      }
      link = link->getParent();
    }

    if (joint_names_.size() != num_joints_)
    {
      ROS_FATAL_STREAM_NAMED(name_, "Joint numbers mismatch: URDF has " << joint_names_.size() << " and IKFast has "
                                                                        << num_joints_);
      return false;
    }

  std::reverse(link_names_.begin(), link_names_.end());
  std::reverse(joint_names_.begin(), joint_names_.end());
  std::reverse(joint_min_vector_.begin(), joint_min_vector_.end());
  std::reverse(joint_max_vector_.begin(), joint_max_vector_.end());
  std::reverse(joint_has_limits_vector_.begin(), joint_has_limits_vector_.end());

  for (size_t i = 0; i < num_joints_; ++i)
  {
    ROS_INFO_STREAM_NAMED(name_, joint_names_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i] << " "
                                                  << joint_has_limits_vector_[i]);
    ROS_DEBUG_STREAM_NAMED(name_, joint_names_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i] << " "
                                                  << joint_has_limits_vector_[i]);
  }

  active_ = true;
  return true;
}


bool IKFastKinematicsPlugin::initialize(const std::string& robot_description, const std::string& group_name,
                                        const std::string& base_name, const std::string& tip_name,
                                        double search_discretization)
{
  setValues(robot_description, group_name, base_name, tip_name, search_discretization);

  ros::NodeHandle node_handle("~/" + group_name);

  std::string robot;
  lookupParam("robot", robot, std::string());

  // IKFast56/61
  fillFreeParams(GetNumFreeParameters(), GetFreeParameters());

  if (free_params_.size() > 1)
  {
    ROS_FATAL("Only one free joint parameter supported!");
    return false;
  }
  else if (free_params_.size() == 1)
  {
    redundant_joint_indices_.clear();
    redundant_joint_indices_.push_back(free_params_[0]);
    KinematicsBase::setSearchDiscretization(DEFAULT_SEARCH_DISCRETIZATION);
  }

  urdf::Model robot_model;
  std::string xml_string;

  std::string urdf_xml, full_urdf_xml;
  lookupParam("urdf_xml", urdf_xml, robot_description);
  node_handle.searchParam(urdf_xml, full_urdf_xml);

  ROS_DEBUG_NAMED(name_, "Reading xml file from parameter server");
  if (!node_handle.getParam(full_urdf_xml, xml_string))
  {
    ROS_FATAL_NAMED(name_, "Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }

  robot_model.initString(xml_string);

  ROS_DEBUG_STREAM_NAMED(name_, "Reading joints and links from URDF");

  urdf::LinkConstSharedPtr link = robot_model.getLink(getTipFrame());
  while (link->name != base_frame_ && joint_names_.size() <= num_joints_)
  {
    ROS_DEBUG_NAMED(name_, "Link %s", link->name.c_str());
    link_names_.push_back(link->name);
    urdf::JointSharedPtr joint = link->parent_joint;
    if (joint)
    {
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      {
        ROS_DEBUG_STREAM_NAMED(name_, "Adding joint " << joint->name);

        joint_names_.push_back(joint->name);
        float lower, upper;
        int hasLimits;
        if (joint->type != urdf::Joint::CONTINUOUS)
        {
          if (joint->safety)
          {
            lower = joint->safety->soft_lower_limit;
            upper = joint->safety->soft_upper_limit;
          }
          else
          {
            lower = joint->limits->lower;
            upper = joint->limits->upper;
          }
          hasLimits = 1;
        }
        else
        {
          lower = -M_PI;
          upper = M_PI;
          hasLimits = 0;
        }
        if (hasLimits)
        {
          joint_has_limits_vector_.push_back(true);
          joint_min_vector_.push_back(lower);
          joint_max_vector_.push_back(upper);
        }
        else
        {
          joint_has_limits_vector_.push_back(false);
          joint_min_vector_.push_back(-M_PI);
          joint_max_vector_.push_back(M_PI);
        }
      }
    }
    else
    {
      ROS_WARN_NAMED(name_, "no joint corresponding to %s", link->name.c_str());
    }
    link = link->getParent();
  }

  if (joint_names_.size() != num_joints_)
  {
    ROS_FATAL_STREAM_NAMED(name_, "Joint numbers mismatch: URDF has " << joint_names_.size() << " and IKFast has "
                                                                      << num_joints_);
    return false;
  }

  std::reverse(link_names_.begin(), link_names_.end());
  std::reverse(joint_names_.begin(), joint_names_.end());
  std::reverse(joint_min_vector_.begin(), joint_min_vector_.end());
  std::reverse(joint_max_vector_.begin(), joint_max_vector_.end());
  std::reverse(joint_has_limits_vector_.begin(), joint_has_limits_vector_.end());

  for (size_t i = 0; i < num_joints_; ++i)
    ROS_DEBUG_STREAM_NAMED(name_, joint_names_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i] << " "
                                                  << joint_has_limits_vector_[i]);

  active_ = true;
  return true;
}

void IKFastKinematicsPlugin::setSearchDiscretization(const std::map<int, double>& discretization)
{
  if (discretization.empty())
  {
    ROS_ERROR("The 'discretization' map is empty");
    return;
  }

  if (redundant_joint_indices_.empty())
  {
    ROS_ERROR_STREAM("This group's solver doesn't support redundant joints");
    return;
  }

  if (discretization.begin()->first != redundant_joint_indices_[0])
  {
    std::string redundant_joint = joint_names_[free_params_[0]];
    ROS_ERROR_STREAM("Attempted to discretize a non-redundant joint "
                     << discretization.begin()->first << ", only joint '" << redundant_joint << "' with index "
                     << redundant_joint_indices_[0] << " is redundant.");
    return;
  }

  if (discretization.begin()->second <= 0.0)
  {
    ROS_ERROR_STREAM("Discretization can not takes values that are <= 0");
    return;
  }

  redundant_joint_discretization_.clear();
  redundant_joint_discretization_[redundant_joint_indices_[0]] = discretization.begin()->second;
}

bool IKFastKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices)
{
  ROS_ERROR_STREAM("Changing the redundant joints isn't permitted by this group's solver ");
  return false;
}

int IKFastKinematicsPlugin::solve(KDL::Frame& pose_frame, const std::vector<double>& vfree,
                                  IkSolutionList<IkReal>& solutions) const
{
  // IKFast56/61
  solutions.Clear();

  double trans[3];
  trans[0] = pose_frame.p[0];  //-.18;
  trans[1] = pose_frame.p[1];
  trans[2] = pose_frame.p[2];

  KDL::Rotation mult;
  KDL::Vector direction;

  switch (GetIkType())
  {
    case IKP_Transform6D:
    case IKP_Translation3D:
      // For **Transform6D**, eerot is 9 values for the 3x3 rotation matrix. For **Translation3D**, these are ignored.

      mult = pose_frame.M;

      double vals[9];
      vals[0] = mult(0, 0);
      vals[1] = mult(0, 1);
      vals[2] = mult(0, 2);
      vals[3] = mult(1, 0);
      vals[4] = mult(1, 1);
      vals[5] = mult(1, 2);
      vals[6] = mult(2, 0);
      vals[7] = mult(2, 1);
      vals[8] = mult(2, 2);

      // IKFast56/61
      ComputeIk(trans, vals, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
      return solutions.GetNumSolutions();

    case IKP_Direction3D:
    case IKP_Ray4D:
    case IKP_TranslationDirection5D:
      // For **Direction3D**, **Ray4D**, and **TranslationDirection5D**, the first 3 values represent the target
      // direction.

      direction = pose_frame.M * KDL::Vector(0, 0, 1);
      ComputeIk(trans, direction.data, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
      std::cout<<"found :::"<<vfree[0]<<":::"<<vfree[1]<<":::"<<std::endl;
      return solutions.GetNumSolutions();

    case IKP_TranslationXAxisAngle4D:
    case IKP_TranslationYAxisAngle4D:
    case IKP_TranslationZAxisAngle4D:
      // For *TranslationXAxisAngle4D*, *TranslationYAxisAngle4D*, *TranslationZAxisAngle4D* - end effector origin
      // reaches desired 3D translation, manipulator direction makes a specific angle with x/y/z-axis (defined in the
      // manipulator base link’s coordinate system)
      ROS_ERROR_NAMED(name_, "IK for this IkParameterizationType not implemented yet.");
      return 0;

    case IKP_TranslationLocalGlobal6D:
      // For **TranslationLocalGlobal6D**, the diagonal elements ([0],[4],[8]) are the local translation inside the end
      // effector coordinate system.
      ROS_ERROR_NAMED(name_, "IK for this IkParameterizationType not implemented yet.");
      return 0;

    case IKP_Rotation3D:
    case IKP_Lookat3D:
    case IKP_TranslationXY2D:
    case IKP_TranslationXYOrientation3D:
      ROS_ERROR_NAMED(name_, "IK for this IkParameterizationType not implemented yet.");
      return 0;

    case IKP_TranslationXAxisAngleZNorm4D:
      double roll, pitch, yaw;
      // For **TranslationXAxisAngleZNorm4D** - end effector origin reaches desired 3D translation, manipulator
      // direction needs to be orthogonal to z axis and be rotated at a certain angle starting from the x axis (defined
      // in the manipulator base link’s coordinate system)
      pose_frame.M.GetRPY(roll, pitch, yaw);
      ComputeIk(trans, &yaw, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
      return solutions.GetNumSolutions();

    case IKP_TranslationYAxisAngleXNorm4D:
      // For **TranslationYAxisAngleXNorm4D** - end effector origin reaches desired 3D translation, manipulator
      // direction needs to be orthogonal to x axis and be rotated at a certain angle starting from the y axis (defined
      // in the manipulator base link’s coordinate system)
      pose_frame.M.GetRPY(roll, pitch, yaw);
      ComputeIk(trans, &roll, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
      return solutions.GetNumSolutions();

    case IKP_TranslationZAxisAngleYNorm4D:
      // For **TranslationZAxisAngleYNorm4D** - end effector origin reaches desired 3D translation, manipulator
      // direction needs to be orthogonal to y axis and be rotated at a certain angle starting from the z axis (defined
      // in the manipulator base link’s coordinate system)
      pose_frame.M.GetRPY(roll, pitch, yaw);
      ComputeIk(trans, &pitch, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
      return solutions.GetNumSolutions();

    default:
      ROS_ERROR_NAMED(name_, "Unknown IkParameterizationType! "
                             "Was the solver generated with an incompatible version of Openrave?");
      return 0;
  }
}

void IKFastKinematicsPlugin::getSolution(const IkSolutionList<IkReal>& solutions, int i,
                                         std::vector<double>& solution) const
{
  solution.clear();
  solution.resize(num_joints_);

  // IKFast56/61
  const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
  std::vector<IkReal> vsolfree(sol.GetFree().size());
  sol.GetSolution(&solution[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

  // std::cout << "solution " << i << ":" ;
  // for(int j=0;j<num_joints_; ++j)
  //   std::cout << " " << solution[j];
  // std::cout << std::endl;

  // ROS_ERROR("%f %d",solution[2],vsolfree.size());
}

void IKFastKinematicsPlugin::getSolution(const IkSolutionList<IkReal>& solutions,
                                         const std::vector<double>& ik_seed_state, int i,
                                         std::vector<double>& solution) const
{
  solution.clear();
  solution.resize(num_joints_);
 
  // IKFast56/61
  const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
  std::vector<IkReal> vsolfree(sol.GetFree().size());
  sol.GetSolution(&solution[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

  // rotate joints by +/-360° where it is possible and useful
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    if (joint_has_limits_vector_[i])
    {
      double signed_distance = solution[i] - ik_seed_state[i];
      while (signed_distance > M_PI && solution[i] - 2 * M_PI > (joint_min_vector_[i] - LIMIT_TOLERANCE))
      {
        signed_distance -= 2 * M_PI;
        solution[i] -= 2 * M_PI;
      }
      while (signed_distance < -M_PI && solution[i] + 2 * M_PI < (joint_max_vector_[i] + LIMIT_TOLERANCE))
      {
        signed_distance += 2 * M_PI;
        solution[i] += 2 * M_PI;
      }
    }
  }
}

double IKFastKinematicsPlugin::harmonize(const std::vector<double>& ik_seed_state, std::vector<double>& solution) const
{
  double dist_sqr = 0;
  std::vector<double> ss = ik_seed_state;
  for (size_t i = 0; i < ik_seed_state.size(); ++i)
  {
    while (ss[i] > 2 * M_PI)
    {
      ss[i] -= 2 * M_PI;
    }
    while (ss[i] < 2 * M_PI)
    {
      ss[i] += 2 * M_PI;
    }
    while (solution[i] > 2 * M_PI)
    {
      solution[i] -= 2 * M_PI;
    }
    while (solution[i] < 2 * M_PI)
    {
      solution[i] += 2 * M_PI;
    }
    dist_sqr += fabs(ik_seed_state[i] - solution[i]);
  }
  return dist_sqr;
}

// void IKFastKinematicsPlugin::getOrderedSolutions(const std::vector<double> &ik_seed_state,
//                                  std::vector<std::vector<double> >& solslist)
// {
//   std::vector<double>
//   double mindist = 0;
//   int minindex = -1;
//   std::vector<double> sol;
//   for(size_t i=0;i<solslist.size();++i){
//     getSolution(i,sol);
//     double dist = harmonize(ik_seed_state, sol);
//     //std::cout << "dist[" << i << "]= " << dist << std::endl;
//     if(minindex == -1 || dist<mindist){
//       minindex = i;
//       mindist = dist;
//     }
//   }
//   if(minindex >= 0){
//     getSolution(minindex,solution);
//     harmonize(ik_seed_state, solution);
//     index = minindex;
//   }
// }

void IKFastKinematicsPlugin::getClosestSolution(const IkSolutionList<IkReal>& solutions,
                                                const std::vector<double>& ik_seed_state,
                                                std::vector<double>& solution) const
{
  
  double mindist = DBL_MAX;
  int minindex = -1;
  std::vector<double> sol;
  std::cout<<"found:::::::::::::::::::"<<solutions.GetNumSolutions()<<std::endl;
  // IKFast56/61
  for (size_t i = 0; i < solutions.GetNumSolutions(); ++i)
  {
   
    getSolution(solutions, i, sol);
    double dist = harmonize(ik_seed_state, sol);
    ROS_INFO_STREAM_NAMED(name_, "Dist " << i << " dist " << dist);
    // std::cout << "dist[" << i << "]= " << dist << std::endl;
    if (minindex == -1 || dist < mindist)
    {
      minindex = i;
      mindist = dist;
    }
  }
  if (minindex >= 0)
  {
    getSolution(solutions, minindex, solution);
    harmonize(ik_seed_state, solution);
  }
}

void IKFastKinematicsPlugin::fillFreeParams(int count, int* array)
{
  free_params_.clear();
  for (int i = 0; i < count; ++i)
    free_params_.push_back(array[i]);
}

bool IKFastKinematicsPlugin::getCount(int& count, const int& max_count, const int& min_count) const
{
  if (count > 0)
  {
    if (-count >= min_count)
    {
      count = -count;
      return true;
    }
    else if (count + 1 <= max_count)
    {
      count = count + 1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if (1 - count <= max_count)
    {
      count = 1 - count;
      return true;
    }
    else if (count - 1 >= min_count)
    {
      count = count - 1;
      return true;
    }
    else
      return false;
  }
}

bool IKFastKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                           const std::vector<double>& joint_angles,
                                           std::vector<geometry_msgs::Pose>& poses) const
{
  if (GetIkType() != IKP_Transform6D)
  {
    // ComputeFk() is the inverse function of ComputeIk(), so the format of
    // eerot differs depending on IK type. The Transform6D IK type is the only
    // one for which a 3x3 rotation matrix is returned, which means we can only
    // compute FK for that IK type.
    ROS_ERROR_NAMED(name_, "Can only compute FK for Transform6D IK type!");
    return false;
  }

  KDL::Frame p_out;
  if (link_names.size() == 0)
  {
    ROS_WARN_STREAM_NAMED(name_, "Link names with nothing");
    return false;
  }

  if (link_names.size() != 1 || link_names[0] != getTipFrame())
  {
    ROS_ERROR_NAMED(name_, "Can compute FK for %s only", getTipFrame().c_str());
    return false;
  }

  bool valid = true;

  IkReal eerot[9], eetrans[3];

  if (joint_angles.size() != num_joints_)
  {
    ROS_ERROR_NAMED(name_, "Unexpected number of joint angles");
    return false;
  }

  IkReal angles[num_joints_];
  for (unsigned char i = 0; i < num_joints_; i++)
    angles[i] = joint_angles[i];

  // IKFast56/61
  ComputeFk(angles, eetrans, eerot);
  std::cout<<"found :::::"<<angles[0]<<std::endl;
  for (int i = 0; i < 3; ++i)
    p_out.p.data[i] = eetrans[i];

  for (int i = 0; i < 9; ++i)
    p_out.M.data[i] = eerot[i];

  poses.resize(1);
  tf::poseKDLToMsg(p_out, poses[0]);

  return valid;
}

bool IKFastKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, double timeout,
                                              std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool IKFastKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, double timeout,
                                              const std::vector<double>& consistency_limits,
                                              std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool IKFastKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, double timeout,
                                              std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                              moveit_msgs::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool IKFastKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, double timeout,
                                              const std::vector<double>& consistency_limits,
                                              std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                              moveit_msgs::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  ROS_DEBUG_STREAM_NAMED(name_, "searchPositionIK");
  std::cout<<":::::::::::::::"<<"searchPositionIK"<<std::endl;

  /// search_mode is currently fixed during code generation
  SEARCH_MODE search_mode = OPTIMIZE_MAX_JOINT;

  // Check if there are no redundant joints
  if (free_params_.size() == 0)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "No need to search since no free params/redundant joints");

    std::vector<geometry_msgs::Pose> ik_poses(1, ik_pose);
    std::vector<std::vector<double>> solutions;
    kinematics::KinematicsResult kinematic_result;
    // Find all IK solution within joint limits
    if (!getPositionIK(ik_poses, ik_seed_state, solutions, kinematic_result, options))
    {
      for(int i;i<solutions.size();i++){
        std::cout<<"Solution:::"<<solutions[i][0]
                 <<"Solution:::"<<solutions[i][1]
                 <<"Solution:::"<<solutions[i][2]
                 <<"Solution:::"<<solutions[i][3]
                 <<"Solution:::"<<solutions[i][4]
                 <<"Solution:::"<<solutions[i][5]
                 <<"Solution:::"<<solutions[i][6]
                 <<std::endl;
      }
      ROS_DEBUG_STREAM_NAMED(name_, "No solution whatsoever");
      error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }
    
    // sort solutions by their distance to the seed
    std::vector<LimitObeyingSol> solutions_obey_limits;
    for (std::size_t i = 0; i < solutions.size(); ++i)
    {
      double dist_from_seed = 0.0;
      for (std::size_t j = 0; j < ik_seed_state.size(); ++j)
      {
        dist_from_seed += fabs(ik_seed_state[j] - solutions[i][j]);
      }

      solutions_obey_limits.push_back({ solutions[i], dist_from_seed });
    }
    std::sort(solutions_obey_limits.begin(), solutions_obey_limits.end());

    // check for collisions if a callback is provided
    if (!solution_callback.empty())
    {
      for (std::size_t i = 0; i < solutions_obey_limits.size(); ++i)
      {
        solution_callback(ik_pose, solutions_obey_limits[i].value, error_code);
        if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          solution = solutions_obey_limits[i].value;
          ROS_DEBUG_STREAM_NAMED(name_, "Solution passes callback");
          return true;
        }
      }

      ROS_DEBUG_STREAM_NAMED(name_, "Solution has error code " << error_code);
      return false;
    }
    else
    {
      solution = solutions_obey_limits[0].value;
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      return true;  // no collision check callback provided
    }
  }

  // -------------------------------------------------------------------------------------------------
  // Error Checking
  if (!active_)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != num_joints_)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Seed state must have size " << num_joints_ << " instead of size "
                                                               << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (!consistency_limits.empty() && consistency_limits.size() != num_joints_)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Consistency limits be empty or must have size " << num_joints_ << " instead of size "
                                                                                   << consistency_limits.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // -------------------------------------------------------------------------------------------------
  // Initialize

  KDL::Frame frame;
  tf::poseMsgToKDL(ik_pose, frame);
 

  std::vector<double> vfree(free_params_.size());

  //ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
  int counter = 0;

  double initial_guess = ik_seed_state[free_params_[0]];
  vfree[0] = initial_guess;

  // -------------------------------------------------------------------------------------------------
  // Handle consitency limits if needed
  int num_positive_increments;
  int num_negative_increments;

  if (!consistency_limits.empty())
  {
    // moveit replaced consistency_limit (scalar) w/ consistency_limits (vector)
    // Assume [0]th free_params element for now.  Probably wrong.
    double max_limit = fmin(joint_max_vector_[free_params_[0]], initial_guess + consistency_limits[free_params_[0]]);
    double min_limit = fmax(joint_min_vector_[free_params_[0]], initial_guess - consistency_limits[free_params_[0]]);

    num_positive_increments = (int)((max_limit - initial_guess) / search_discretization_);
    num_negative_increments = (int)((initial_guess - min_limit) / search_discretization_);
  }
  else  // no consitency limits provided
  {
    num_positive_increments = (joint_max_vector_[free_params_[0]] - initial_guess) / search_discretization_;
    num_negative_increments = (initial_guess - joint_min_vector_[free_params_[0]]) / search_discretization_;
  }

  // -------------------------------------------------------------------------------------------------
  // Begin searching

  ROS_DEBUG_STREAM_NAMED(name_, "Free param is " << free_params_[0] << " initial guess is " << initial_guess
                                                 << ", # positive increments: " << num_positive_increments
                                                 << ", # negative increments: " << num_negative_increments);
  if ((search_mode & OPTIMIZE_MAX_JOINT) && (num_positive_increments + num_negative_increments) > 1000)
    ROS_WARN_STREAM_ONCE_NAMED(name_, "Large search space, consider increasing the search discretization");

  double best_costs = -1.0;
  std::vector<double> best_solution;
  int nattempts = 0, nvalid = 0;

  while (true)
  {
    IkSolutionList<IkReal> solutions;
    int numsol = solve(frame, vfree, solutions);

    ROS_DEBUG_STREAM_NAMED(name_, "Found " << numsol << " solutions from IKFast");

    // ROS_INFO("%f",vfree[0]);

    if (numsol > 0)
    {
      for (int s = 0; s < numsol; ++s)
      {
        nattempts++;
        std::vector<double> sol;
        getSolution(solutions, ik_seed_state, s, sol);

        bool obeys_limits = true;
        for (unsigned int i = 0; i < sol.size(); i++)
        {
          if (joint_has_limits_vector_[i] && (sol[i] < joint_min_vector_[i] || sol[i] > joint_max_vector_[i]))
          {
            obeys_limits = false;
            break;
          }
          // ROS_INFO_STREAM_NAMED(name_,"Num " << i << " value " << sol[i] << " has limits " <<
          // joint_has_limits_vector_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i]);
        }
        if (obeys_limits)
        {
          getSolution(solutions, ik_seed_state, s, solution);

          // This solution is within joint limits, now check if in collision (if callback provided)
          if (!solution_callback.empty())
          {
            solution_callback(ik_pose, solution, error_code);
          }
          else
          {
            error_code.val = error_code.SUCCESS;
          }

          if (error_code.val == error_code.SUCCESS)
          {
            nvalid++;
            if (search_mode & OPTIMIZE_MAX_JOINT)
            {
              // Costs for solution: Largest joint motion
              double costs = 0.0;
              for (unsigned int i = 0; i < solution.size(); i++)
              {
                double d = fabs(ik_seed_state[i] - solution[i]);
                if (d > costs)
                  costs = d;
              }
              if (costs < best_costs || best_costs == -1.0)
              {
                best_costs = costs;
                best_solution = solution;
                for(int i=0;i<best_solution.size();i++)
                {
                  
                   std::cout<<"Solution:::"<<best_solution[0]<<"   "
                            <<"Solution:::"<<best_solution[1]<<"   "
                            <<"Solution:::"<<best_solution[2]<<"   "
                            <<"Solution:::"<<best_solution[3]<<"   "
                            <<"Solution:::"<<best_solution[4]<<"   "
                            <<"Solution:::"<<best_solution[5]<<"   "
                            <<"Solution:::"<<best_solution[6]<<"   "
                            <<std::endl;
                }
              }
            }
            else
              // Return first feasible solution
              return true;
          }
        }
      }
    }

    if (!getCount(counter, num_positive_increments, -num_negative_increments))
    {
      // Everything searched
      error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      break;
    }

    vfree[0] = initial_guess + search_discretization_ * counter;
    // ROS_DEBUG_STREAM_NAMED(name_,"Attempt " << counter << " with 0th free joint having value " << vfree[0]);
  }

  ROS_DEBUG_STREAM_NAMED(name_, "Valid solutions: " << nvalid << "/" << nattempts);

  if ((search_mode & OPTIMIZE_MAX_JOINT) && best_costs != -1.0)
  {
    solution = best_solution;
    error_code.val = error_code.SUCCESS;
    return true;
  }

  // No solution found
  error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}

// Used when there are no redundant joints - aka no free params
bool IKFastKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_DEBUG_STREAM_NAMED(name_, "getPositionIK");
  std::cout<<"getPositionIK"<<std::endl;

  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    return false;
  }

  if (ik_seed_state.size() < num_joints_)
  {
    ROS_ERROR_STREAM("ik_seed_state only has " << ik_seed_state.size() << " entries, this ikfast solver requires "
                                               << num_joints_);
    return false;
  }

  // Check if seed is in bound
  for (std::size_t i = 0; i < ik_seed_state.size(); i++)
  {
    // Add tolerance to limit check
    if (joint_has_limits_vector_[i] && ((ik_seed_state[i] < (joint_min_vector_[i] - LIMIT_TOLERANCE)) ||
                                        (ik_seed_state[i] > (joint_max_vector_[i] + LIMIT_TOLERANCE))))
    {
      ROS_DEBUG_STREAM_NAMED("ikseed", "Not in limits! " << (int)i << " value " << ik_seed_state[i]
                                                         << " has limit: " << joint_has_limits_vector_[i] << "  being  "
                                                         << joint_min_vector_[i] << " to " << joint_max_vector_[i]);
      return false;
    }
  }

  std::vector<double> vfree(free_params_.size());
  for (std::size_t i = 0; i < free_params_.size(); ++i)
  {
    int p = free_params_[i];
    ROS_ERROR("%u is %f", p, ik_seed_state[p]);  // DTC
    vfree[i] = ik_seed_state[p];
  }

  KDL::Frame frame;
  tf::poseMsgToKDL(ik_pose, frame);

  IkSolutionList<IkReal> solutions;
  int numsol = solve(frame, vfree, solutions);

  ROS_DEBUG_STREAM_NAMED(name_, "Found " << numsol << " solutions from IKFast");

  std::vector<LimitObeyingSol> solutions_obey_limits;

  if (numsol)
  {
    std::vector<double> solution_obey_limits;
    for (std::size_t s = 0; s < numsol; ++s)
    {
      std::vector<double> sol;
      getSolution(solutions, ik_seed_state, s, sol);
      ROS_DEBUG_NAMED(name_, "Sol %d: %e   %e   %e   %e   %e   %e", (int)s, sol[0], sol[1], sol[2], sol[3], sol[4],
                      sol[5]);

      bool obeys_limits = true;
      for (std::size_t i = 0; i < sol.size(); i++)
      {
        // Add tolerance to limit check
        if (joint_has_limits_vector_[i] && ((sol[i] < (joint_min_vector_[i] - LIMIT_TOLERANCE)) ||
                                            (sol[i] > (joint_max_vector_[i] + LIMIT_TOLERANCE))))
        {
          // One element of solution is not within limits
          obeys_limits = false;
          ROS_DEBUG_STREAM_NAMED(name_, "Not in limits! " << (int)i << " value " << sol[i] << " has limit: "
                                                          << joint_has_limits_vector_[i] << "  being  "
                                                          << joint_min_vector_[i] << " to " << joint_max_vector_[i]);
          break;
        }
      }
      if (obeys_limits)
      {
        // All elements of this solution obey limits
        getSolution(solutions, ik_seed_state, s, solution_obey_limits);
        double dist_from_seed = 0.0;
        for (std::size_t i = 0; i < ik_seed_state.size(); ++i)
        {
          dist_from_seed += fabs(ik_seed_state[i] - solution_obey_limits[i]);
        }

        solutions_obey_limits.push_back({ solution_obey_limits, dist_from_seed });
      }
    }
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(name_, "No IK solution");
  }

  // Sort the solutions under limits and find the one that is closest to ik_seed_state
  if (!solutions_obey_limits.empty())
  {
    std::sort(solutions_obey_limits.begin(), solutions_obey_limits.end());
    solution = solutions_obey_limits[0].value;
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}

bool IKFastKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                           const std::vector<double>& ik_seed_state,
                                           std::vector<std::vector<double>>& solutions,
                                           kinematics::KinematicsResult& result,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_DEBUG_STREAM_NAMED(name_, "getPositionIK with multiple solutions");
  std::cout<<"getPositionIK with multiple solutions"<<std::endl;
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    result.kinematic_error = kinematics::KinematicErrors::SOLVER_NOT_ACTIVE;
    return false;
  }

  if (ik_poses.empty())
  {
    ROS_ERROR("ik_poses is empty");
    result.kinematic_error = kinematics::KinematicErrors::EMPTY_TIP_POSES;
    return false;
  }

  if (ik_poses.size() > 1)
  {
    ROS_ERROR("ik_poses contains multiple entries, only one is allowed");
    result.kinematic_error = kinematics::KinematicErrors::MULTIPLE_TIPS_NOT_SUPPORTED;
    return false;
  }

  if (ik_seed_state.size() < num_joints_)
  {
    ROS_ERROR_STREAM("ik_seed_state only has " << ik_seed_state.size() << " entries, this ikfast solver requires "
                                               << num_joints_);
    return false;
  }

  KDL::Frame frame;
  tf::poseMsgToKDL(ik_poses[0], frame);

  // solving ik
  std::vector<IkSolutionList<IkReal>> solution_set;
  IkSolutionList<IkReal> ik_solutions;
  std::vector<double> vfree;
  int numsol = 0;
  std::vector<double> sampled_joint_vals;
  if (!redundant_joint_indices_.empty())
  {
    // initializing from seed
    sampled_joint_vals.push_back(ik_seed_state[redundant_joint_indices_[0]]);

    // checking joint limits when using no discretization
    if (options.discretization_method == kinematics::DiscretizationMethods::NO_DISCRETIZATION &&
        joint_has_limits_vector_[redundant_joint_indices_.front()])
    {
      double joint_min = joint_min_vector_[redundant_joint_indices_.front()];
      double joint_max = joint_max_vector_[redundant_joint_indices_.front()];

      double jv = sampled_joint_vals[0];
      if (!((jv > (joint_min - LIMIT_TOLERANCE)) && (jv < (joint_max + LIMIT_TOLERANCE))))
      {
        result.kinematic_error = kinematics::KinematicErrors::IK_SEED_OUTSIDE_LIMITS;
        ROS_ERROR_STREAM("ik seed is out of bounds");
        return false;
      }
    }

    // computing all solutions sets for each sampled value of the redundant joint
    if (!sampleRedundantJoint(options.discretization_method, sampled_joint_vals))
    {
      result.kinematic_error = kinematics::KinematicErrors::UNSUPORTED_DISCRETIZATION_REQUESTED;
      return false;
    }

    for (unsigned int i = 0; i < sampled_joint_vals.size(); i++)
    {
      vfree.clear();
      vfree.push_back(sampled_joint_vals[i]);
      numsol += solve(frame, vfree, ik_solutions);
      solution_set.push_back(ik_solutions);
    }
  }
  else
  {
    // computing for single solution set
    numsol = solve(frame, vfree, ik_solutions);
    solution_set.push_back(ik_solutions);
  }

  ROS_DEBUG_STREAM_NAMED(name_, "Found " << numsol << " solutions from IKFast");
  bool solutions_found = false;
  if (numsol > 0)
  {
    /*
      Iterating through all solution sets and storing those that do not exceed joint limits.
    */
    for (unsigned int r = 0; r < solution_set.size(); r++)
    {
      ik_solutions = solution_set[r];
      numsol = ik_solutions.GetNumSolutions();
      for (int s = 0; s < numsol; ++s)
      {
        std::vector<double> sol;
        getSolution(ik_solutions, ik_seed_state, s, sol);

        bool obeys_limits = true;
        for (unsigned int i = 0; i < sol.size(); i++)
        {
          // Add tolerance to limit check
          if (joint_has_limits_vector_[i] && ((sol[i] < (joint_min_vector_[i] - LIMIT_TOLERANCE)) ||
                                              (sol[i] > (joint_max_vector_[i] + LIMIT_TOLERANCE))))
          {
            // One element of solution is not within limits
            obeys_limits = false;
            ROS_DEBUG_STREAM_NAMED(name_, "Not in limits! " << i << " value " << sol[i] << " has limit: "
                                                            << joint_has_limits_vector_[i] << "  being  "
                                                            << joint_min_vector_[i] << " to " << joint_max_vector_[i]);
            break;
          }
        }
        if (obeys_limits)
        {
          // All elements of solution obey limits
          solutions_found = true;
          solutions.push_back(sol);
        }
      }
    }

    if (solutions_found)
    {
      result.kinematic_error = kinematics::KinematicErrors::OK;
      return true;
    }
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(name_, "No IK solution");
  }

  result.kinematic_error = kinematics::KinematicErrors::NO_SOLUTION;
  return false;
}

bool IKFastKinematicsPlugin::sampleRedundantJoint(kinematics::DiscretizationMethod method,
                                                  std::vector<double>& sampled_joint_vals) const
{
  double joint_min = -M_PI;
  double joint_max = M_PI;
  int index = redundant_joint_indices_.front();
  double joint_dscrt = redundant_joint_discretization_.at(index);

  if (joint_has_limits_vector_[redundant_joint_indices_.front()])
  {
    joint_min = joint_min_vector_[index];
    joint_max = joint_max_vector_[index];
  }

  switch (method)
  {
    case kinematics::DiscretizationMethods::ALL_DISCRETIZED:
    {
      int steps = std::ceil((joint_max - joint_min) / joint_dscrt);
      for (unsigned int i = 0; i < steps; i++)
      {
        sampled_joint_vals.push_back(joint_min + joint_dscrt * i);
      }
      sampled_joint_vals.push_back(joint_max);
    }
    break;
    case kinematics::DiscretizationMethods::ALL_RANDOM_SAMPLED:
    {
      int steps = std::ceil((joint_max - joint_min) / joint_dscrt);
      steps = steps > 0 ? steps : 1;
      double diff = joint_max - joint_min;
      for (int i = 0; i < steps; i++)
      {
        sampled_joint_vals.push_back(((diff * std::rand()) / (static_cast<double>(RAND_MAX))) + joint_min);
      }
    }

    break;
    case kinematics::DiscretizationMethods::NO_DISCRETIZATION:

      break;
    default:
      ROS_ERROR_STREAM("Discretization method " << method << " is not supported");
      return false;
  }

  return true;
}

}  // namespace tobor_left_arm_group

// register IKFastKinematicsPlugin as a KinematicsBase implementation
//#include <pluginlib/class_list_macros.hpp>
//PLUGINLIB_EXPORT_CLASS(tobor_left_arm_group::IKFastKinematicsPlugin, kinematics::KinematicsBase);
