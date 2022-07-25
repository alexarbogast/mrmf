#ifndef MRMF_CORE_MULTI_ROBOT_GROUP_H
#define MRMF_CORE_MULTI_ROBOT_GROUP_H

#include <moveit/robot_trajectory/robot_trajectory.h>

#include <mrmf_core/robot.h>
#include <mrmf_core/sync_trajectory.h>

namespace mrmf_core
{
class MultiRobotGroup
{
public:

    MultiRobotGroup(const std::string& group, const moveit::core::RobotModelConstPtr& robot_model);

    inline const std::string& getHomePositionName() const { return home_position_name_; }

    RobotID addRobot(const std::string& group, 
                     const std::string& tip_frame,
                     const Robot::RobotType = Robot::RobotType::MANIPULATOR);

    RobotID addRobot(const std::string& group, 
                     const std::string& tip_frame,
                     const std::string& base_frame,
                     const Robot::RobotType = Robot::RobotType::MANIPULATOR);

    RobotPtr getRobot(const RobotID& id);
    const RobotPtr getRobot(const RobotID& id) const;

    const robot_model::JointModelGroup* getJointModelGroup(const RobotID& id) const;

    bool planMultiRobotTrajectory(SynchronousTrajectory& traj, 
                                  robot_trajectory::RobotTrajectory& output_traj,
                                  robot_state::RobotState& seed_state);

    bool kinematicsQuery(KinematicsQueryContext& context, moveit::core::RobotState& seed_state);

private:
    void addGlobalConstraints(KinematicsQueryContext& context);
    
    // temp
    double positionerOptimization(const SyncPointInfo& spi,
                                  const RobotID& positioner,
                                  robot_state::RobotState& seed_state) const;

private:
    std::string group_;
    std::string home_position_name_ = "home";

    std::unordered_map<RobotID, RobotPtr> robots_;

    robot_model::RobotModelConstPtr robot_model_;
    const robot_model::JointModelGroup* joint_model_group_;
};

} // namespace mrmf_core

#endif // MRMF_CORE_MULTI_ROBOT_GROUP_H