#include <mrmf_core/multi_robot_group.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace mrmf_core
{
MultiRobotGroup::MultiRobotGroup(const std::string& group, const moveit::core::RobotModelConstPtr& robot_model)
    : group_(group), 
      robot_model_(robot_model),
      joint_model_group_(robot_model_->getJointModelGroup(group_))
{
}

RobotID MultiRobotGroup::addRobot(const std::string& group,  
                                  const std::string& tip_frame,
                                  const Robot::RobotType type)
{
    auto jmg = robot_model_->getJointModelGroup(group);
    
    if (!jmg) return RobotID::make_nil();
    if (!jmg->hasLinkModel(tip_frame)) return RobotID::make_nil();

    const std::string& model_frame = robot_model_->getModelFrame();
    RobotPtr robot = std::make_shared<Robot>(group, tip_frame, model_frame, type);
        
    RobotID id = robot->getID();
    robots_[id.value()] = robot;
    
    return id;
}

RobotPtr MultiRobotGroup::getRobot(const RobotID& id)
{
    return robots_[id.value()];
}

bool MultiRobotGroup::planSynchronousTrajectory(CompositeTrajectory& comp_traj, 
                                                robot_trajectory::RobotTrajectory& output_traj,
                                                moveit::core::RobotState& start_state)
{
    //if (!comp_traj.equalSizes()) return false;
//
    //moveit::core::RobotState state(start_state);
//
    //static double delta_time = 1;
    //output_traj.addSuffixWayPoint(state, delta_time);
//
    //for (int i = 0; i < comp_traj.getLongestDimension(); i++)
    //{
    //    KinematicsQueryContext context;
//
    //    for (const auto& traj : comp_traj)
    //    {
    //        auto current_robot = getRobot(traj->id);
//
    //        context.current_robot = current_robot;
    //        traj->points[i]->describe(context);
//
    //        current_robot->describePersistentConstraints(context);
    //    }
    //    addGlobalConstraints(context);
//
    //    bool success = kinematicsQuery(context, state);
//
    //    if (success)
    //        output_traj.addSuffixWayPoint(state, delta_time);
    //    else
    //        return false;
//
    //}    

    // commented because of strange bug with time parameterization
    //trajectory_processing::IterativeParabolicTimeParameterization time_param;
    //return time_param.computeTimeStamps(output_traj);
    return true;
}



bool MultiRobotGroup::planMultiRobotTrajectory(CompositeTrajectory& comp_traj,
                                               robot_trajectory::RobotTrajectory& output_traj,
                                               moveit::core::RobotState& start_state)
{
    
    return false;
}

bool MultiRobotGroup::kinematicsQuery(KinematicsQueryContext& context, moveit::core::RobotState& seed_state)
{
    bool success = seed_state.setFromIK(
        joint_model_group_,              // joints to be used for IK
        EigenSTL::vector_Isometry3d(),  // empty end_effector positions
        std::vector<std::string>(),     // empty tip link names
        0.05,                          // solver timeout
        moveit::core::GroupStateValidityCallbackFn(),
        context.ik_options                      // ik constraints
    );

    return success;
}

void MultiRobotGroup::addGlobalConstraints(KinematicsQueryContext& context)
{
    auto* minimal_displacement = new bio_ik::MinimalDisplacementGoal();
    context.ik_options.goals.emplace_back(minimal_displacement);

    context.ik_options.replace = true;
    context.ik_options.return_approximate_solution = true;
}

void MultiRobotGroup::setCoordinated(const std::vector<RobotID>& robots)
{
    for (auto& id : robots)
    {
        if (robots_.count(id.value()) == 0)
        {
            ROS_ERROR_STREAM("Robot ID: " << id.value() << " does not exist in group");
            return;
        }
    }

    coordinated_robots_ = robots;
}

} // namespace mrmf_core