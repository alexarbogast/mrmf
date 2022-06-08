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
    RobotPtr robot = std::make_shared<Robot>(group, tip_frame, type);
        
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
    if (!comp_traj.equalSizes()) return false;

    moveit::core::RobotState state(start_state);

    static double delta_time = 0.05;
    output_traj.addSuffixWayPoint(state, delta_time);

    for (int i = 0; i < comp_traj.getLongestDimension(); i++)
    {
        KinematicsQueryContext context;

        for (const auto& traj : comp_traj)
        {
            context.current_robot_ = getRobot(traj->id);
            traj->points[i]->describe(context);
        }
        addGlobalConstraints(context);

        bool success = kinematicsQuery(context, state);

        if (success)
            output_traj.addSuffixWayPoint(state, delta_time);
        else
            return false;

    }    

    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    return time_param.computeTimeStamps(output_traj);
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
    // TODO: loop through robots and add constraints
    auto* minimal_displacement = new bio_ik::MinimalDisplacementGoal();
    context.ik_options.goals.emplace_back(minimal_displacement);

    context.ik_options.replace = true;
    context.ik_options.return_approximate_solution = true;
}

} // namespace mrmf_core