#ifndef TOBOR_LEFT_ARM_PLUGIN_H
#define TOBOR_LEFT_ARM_PLUGIN_H


#include <ikfast_common.h>

namespace tobor_left_arm_group
{
#define IKFAST_NO_MAIN  // Don't include main() from IKFast

enum IkParameterizationType
{
  IKP_None = 0,
  IKP_Transform6D = 0x67000001,    ///< end effector reaches desired 6D transformation
  IKP_Rotation3D = 0x34000002,     ///< end effector reaches desired 3D rotation
  IKP_Translation3D = 0x33000003,  ///< end effector origin reaches desired 3D translation
  IKP_Direction3D = 0x23000004,    ///< direction on end effector coordinate system reaches desired direction
  IKP_Ray4D = 0x46000005,          ///< ray on end effector coordinate system reaches desired global ray
  IKP_Lookat3D = 0x23000006,       ///< direction on end effector coordinate system points to desired 3D position
  IKP_TranslationDirection5D = 0x56000007,  ///< end effector origin and direction reaches desired 3D translation and
  /// direction. Can be thought of as Ray IK where the origin of the ray must
  /// coincide.
  IKP_TranslationXY2D = 0x22000008,             ///< 2D translation along XY plane
  IKP_TranslationXYOrientation3D = 0x33000009,  ///< 2D translation along XY plane and 1D rotation around Z axis. The
  /// offset of the rotation is measured starting at +X, so at +X is it 0,
  /// at +Y it is pi/2.
  IKP_TranslationLocalGlobal6D = 0x3600000a,  ///< local point on end effector origin reaches desired 3D global point

  IKP_TranslationXAxisAngle4D = 0x4400000b,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction makes a specific angle with x-axis  like a cone, angle is from
  /// 0-pi. Axes defined in the manipulator base link's coordinate system)
  IKP_TranslationYAxisAngle4D = 0x4400000c,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction makes a specific angle with y-axis  like a cone, angle is from
  /// 0-pi. Axes defined in the manipulator base link's coordinate system)
  IKP_TranslationZAxisAngle4D = 0x4400000d,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction makes a specific angle with z-axis like a cone, angle is from
  /// 0-pi. Axes are defined in the manipulator base link's coordinate system.

  IKP_TranslationXAxisAngleZNorm4D = 0x4400000e,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction needs to be orthogonal to z-axis and be rotated at a
  /// certain angle starting from the x-axis (defined in the manipulator
  /// base link's coordinate system)
  IKP_TranslationYAxisAngleXNorm4D = 0x4400000f,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction needs to be orthogonal to x-axis and be rotated at a
  /// certain angle starting from the y-axis (defined in the manipulator
  /// base link's coordinate system)
  IKP_TranslationZAxisAngleYNorm4D = 0x44000010,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction needs to be orthogonal to y-axis and be rotated at a
  /// certain angle starting from the z-axis (defined in the manipulator
  /// base link's coordinate system)

  IKP_NumberOfParameterizations = 16,  ///< number of parameterizations (does not count IKP_None)

  IKP_VelocityDataBit =
      0x00008000,  ///< bit is set if the data represents the time-derivate velocity of an IkParameterization
  IKP_Transform6DVelocity = IKP_Transform6D | IKP_VelocityDataBit,
  IKP_Rotation3DVelocity = IKP_Rotation3D | IKP_VelocityDataBit,
  IKP_Translation3DVelocity = IKP_Translation3D | IKP_VelocityDataBit,
  IKP_Direction3DVelocity = IKP_Direction3D | IKP_VelocityDataBit,
  IKP_Ray4DVelocity = IKP_Ray4D | IKP_VelocityDataBit,
  IKP_Lookat3DVelocity = IKP_Lookat3D | IKP_VelocityDataBit,
  IKP_TranslationDirection5DVelocity = IKP_TranslationDirection5D | IKP_VelocityDataBit,
  IKP_TranslationXY2DVelocity = IKP_TranslationXY2D | IKP_VelocityDataBit,
  IKP_TranslationXYOrientation3DVelocity = IKP_TranslationXYOrientation3D | IKP_VelocityDataBit,
  IKP_TranslationLocalGlobal6DVelocity = IKP_TranslationLocalGlobal6D | IKP_VelocityDataBit,
  IKP_TranslationXAxisAngle4DVelocity = IKP_TranslationXAxisAngle4D | IKP_VelocityDataBit,
  IKP_TranslationYAxisAngle4DVelocity = IKP_TranslationYAxisAngle4D | IKP_VelocityDataBit,
  IKP_TranslationZAxisAngle4DVelocity = IKP_TranslationZAxisAngle4D | IKP_VelocityDataBit,
  IKP_TranslationXAxisAngleZNorm4DVelocity = IKP_TranslationXAxisAngleZNorm4D | IKP_VelocityDataBit,
  IKP_TranslationYAxisAngleXNorm4DVelocity = IKP_TranslationYAxisAngleXNorm4D | IKP_VelocityDataBit,
  IKP_TranslationZAxisAngleYNorm4DVelocity = IKP_TranslationZAxisAngleYNorm4D | IKP_VelocityDataBit,

  IKP_UniqueIdMask = 0x0000ffff,   ///< the mask for the unique ids
  IKP_CustomDataBit = 0x00010000,  ///< bit is set if the ikparameterization contains custom data, this is only used
  /// when serializing the ik parameterizations
};

// struct for storing and sorting solutions
struct LimitObeyingSol
{
  std::vector<double> value;
  double dist_from_seed;

  bool operator<(const LimitObeyingSol& a) const
  {
    return dist_from_seed < a.dist_from_seed;
  }
};
class IKFastKinematicsPlugin : public kinematics::KinematicsBase
{
    std::vector<std::string> joint_names_;
    std::vector<double> joint_min_vector_;
    std::vector<double> joint_max_vector_;
    std::vector<bool> joint_has_limits_vector_;
    std::vector<std::string> link_names_;
    const size_t num_joints_;
    std::vector<int> free_params_;
    bool active_;  // Internal variable that indicates whether solvers are configured and ready
    const std::string name_{ "ikfast" };

    const std::vector<std::string>& getJointNames() const
    {
        return joint_names_;
    }
    const std::vector<std::string>& getLinkNames() const
    {
        return link_names_;
    }

  // The ikfast and base frame are the start and end of the kinematic chain for which the
  // IKFast analytic solution was generated.
  const std::string IKFAST_TIP_FRAME_ = "robotiq_arg2f_base_link_left";
  const std::string IKFAST_BASE_FRAME_ = "left_arm_base";

  // prefix added to tip- and baseframe to allow different namespaces or multi-robot setups
  std::string link_prefix_;

  // The transform tip and base bool are set to true if this solver is used with a kinematic
  // chain that extends beyond the ikfast tip and base frame. The solution will be valid so
  // long as there are no active, passive, or mimic joints between either the ikfast_tip_frame
  // and the tip_frame of the group or the ikfast_base_frame and the base_frame for the group.
  bool tip_transform_required_;
  bool base_transform_required_;

  // We store the transform from the ikfast_base_frame to the group base_frame as well as the
  // ikfast_tip_frame to the group tip_frame to transform input poses into the solver frame.
  Eigen::Isometry3d chain_base_to_group_base_;
  Eigen::Isometry3d group_tip_to_chain_tip_;

  bool initialized_;  // Internal variable that indicates whether solvers are configured and ready
  

  const std::vector<std::string>& getJointNames() const override
  {
    return joint_names_;
  }
  const std::vector<std::string>& getLinkNames() const override
  {
    return link_names_;
  }

public:
  /** @class
   *  @brief Interface for an IKFast kinematics plugin
   */
  IKFastKinematicsPlugin() : num_joints_(GetNumJoints()), initialized_(false)
  {
    srand(time(nullptr));
    supported_methods_.push_back(kinematics::DiscretizationMethods::NO_DISCRETIZATION);
    supported_methods_.push_back(kinematics::DiscretizationMethods::ALL_DISCRETIZED);
    supported_methods_.push_back(kinematics::DiscretizationMethods::ALL_RANDOM_SAMPLED);
  }

  /**
   * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @return True if a valid solution was found, false otherwise
   */

  // Returns the IK solution that is within joint limits closest to ik_seed_state
  bool
  getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, compute the set joint angles solutions that are able to reach it.
   *
   * This is a default implementation that returns only one solution and so its result is equivalent to calling
   * 'getPositionIK(...)' with a zero initialized seed.
   *
   * @param ik_poses  The desired pose of each tip link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solutions A vector of vectors where each entry is a valid joint solution
   * @param result A struct that reports the results of the query
   * @param options An option struct which contains the type of redundancy discretization used. This default
   *                implementation only supports the KinmaticSearches::NO_DISCRETIZATION method; requesting any
   *                other will result in failure.
   * @return True if a valid set of solutions was found, false otherwise.
   */
  bool getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                     std::vector<std::vector<double>>& solutions, kinematics::KinematicsResult& result,
                     const kinematics::KinematicsQueryOptions& options) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param the distance that the redundancy can be from the current position
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).  The consistency_limit specifies that only certain redundancy positions
   * around those specified in the seed state are admissible and need to be searched.
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param consistency_limit the distance that the redundancy can be from the current position
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a set of joint angles and a set of links, compute their pose
   *
   * @param link_names A set of links for which FK needs to be computed
   * @param joint_angles The state for which FK is being computed
   * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
   * @return True if a valid solution was found, false otherwise
   */
  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::Pose>& poses) const override;

  /**
   * @brief Sets the discretization value for the redundant joint.
   *
   * Since this ikfast implementation allows for one redundant joint then only the first entry will be in the
   *discretization map will be used.
   * Calling this method replaces previous discretization settings.
   *
   * @param discretization a map of joint indices and discretization value pairs.
   */
  void setSearchDiscretization(const std::map<unsigned int, double>& discretization);

  /**
   * @brief Overrides the default method to prevent changing the redundant joints
   */
  bool setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices) override;
  
  bool initialize(rdf_loader::RDFLoaderPtr rdf_loader, const std::string& group_name,
                  const std::string& base_frame, const std::string& tip_frame,
                  double search_discretization);
private:
  bool initialize(const std::string& robot_description, const std::string& group_name, const std::string& base_name,
                  const std::string& tip_name, double search_discretization);

  /**
   * @brief Calls the IK solver from IKFast
   * @return The number of solutions found
   */
  size_t solve(KDL::Frame& pose_frame, const std::vector<double>& vfree, IkSolutionList<IkReal>& solutions) const;

  /**
   * @brief Gets a specific solution from the set
   */
  void getSolution(const IkSolutionList<IkReal>& solutions, int i, std::vector<double>& solution) const;

  /**
   * @brief Gets a specific solution from the set with joints rotated 360° to be near seed state where possible
   */
  void getSolution(const IkSolutionList<IkReal>& solutions, const std::vector<double>& ik_seed_state, int i,
                   std::vector<double>& solution) const;

  /**
   * @brief If the value is outside of min/max then it tries to +/- 2 * pi to put the value into the range
   */
  double enforceLimits(double val, double min, double max) const;

  void fillFreeParams(int count, int* array);
  bool getCount(int& count, const int& max_count, const int& min_count) const;

  /**
   * @brief samples the designated redundant joint using the chosen discretization method
   * @param  method              An enumeration flag indicating the discretization method to be used
   * @param  sampled_joint_vals  Sampled joint values for the redundant joint
   * @return True if sampling succeeded.
   */
  bool sampleRedundantJoint(kinematics::DiscretizationMethod method, std::vector<double>& sampled_joint_vals) const;

  /// Validate that we can compute a fixed transform between from and to links.
  bool computeRelativeTransform(const std::string& from, const std::string& to, Eigen::Isometry3d& transform,
                                bool& differs_from_identity);
  /**
   * @brief  Transforms the input pose to the correct frame for the solver. This assumes that the group includes the
   * entire solver chain and that any joints outside of the solver chain within the group are are fixed.
   * @param  ik_pose             The pose to be transformed which should be in the correct frame for the group.
   * @param  ik_pose_chain       The ik_pose to be populated with the apropriate pose for the solver
   */
  void transformToChainFrame(const geometry_msgs::Pose& ik_pose, KDL::Frame& ik_pose_chain) const;

  robot_model::RobotModelPtr robot_model_;
};  // end class
}
#endif