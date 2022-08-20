#include <mrmf_core/multi_robot_group.h>
#include <mrmf_core/sync_trajectory_planner.h>

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
    const std::string& model_frame = robot_model_->getModelFrame();
    return addRobot(group, tip_frame, model_frame, type);
}

RobotID MultiRobotGroup::addRobot(const std::string& group,  
                                  const std::string& tip_frame,
                                  const std::string& base_frame,
                                  const Robot::RobotType type)
{
    auto jmg = robot_model_->getJointModelGroup(group);
    
    if (!jmg) return RobotID::make_nil();
    if (!jmg->hasLinkModel(tip_frame) || !robot_model_->hasLinkModel(base_frame))
    {
        ROS_ERROR("Could not find tip_frame or base_frame in robot model");
        ROS_ERROR("Failed to add robot to MultiRobotGroup");
        return RobotID::make_nil();
    }

    RobotPtr robot = std::make_shared<Robot>(group, tip_frame, base_frame, jmg, type);

    RobotID id = robot->getID();
    robots_[id] = robot;
    
    return id;
}

RobotPtr MultiRobotGroup::getRobot(const RobotID& id)
{
    return robots_.at(id);
}

const RobotPtr MultiRobotGroup::getRobot(const RobotID& id) const
{
    return robots_.at(id);
}

const robot_model::JointModelGroup* MultiRobotGroup::getJointModelGroup(const RobotID& id) const
{
    return getRobot(id)->getJointModelGroup();
}

bool MultiRobotGroup::planMultiRobotTrajectory(SynchronousTrajectory& traj, 
                                               robot_trajectory::RobotTrajectory& output_traj,
                                               robot_state::RobotState& seed_state)
{
    SyncTrajectoryPlanner planner;
    planner.initialize(this->robots_, this->joint_model_group_);

    SyncTrajectoryPlanner::Config config;
    config.max_velocity_scaling_factor = 0.5;

    bool success = planner.plan(traj, output_traj, seed_state, config);
    return success;
}

} // namespace mrmf_core