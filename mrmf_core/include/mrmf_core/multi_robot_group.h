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

    RobotPtr getRobot(const RobotID& id);

    //bool planSynchronousTrajectory(CompositeTrajectory& traj, 
    //                               robot_trajectory::RobotTrajectory& output_traj,
    //                               moveit::core::RobotState& seed_state);
//
    //bool planMultiRobotTrajectory(CompositeTrajectory& comp_traj,
    //                              robot_trajectory::RobotTrajectory& output_traj,
    //                              moveit::core::RobotState& start_state);

    bool kinematicsQuery(KinematicsQueryContext& context, moveit::core::RobotState& seed_state);

    // test
    void setCoordinated(const std::vector<RobotID>& robots); 
    void clearCoordinated() { coordinated_robots_.clear(); }

private:
    void addGlobalConstraints(KinematicsQueryContext& context);

private:
    std::string group_;
    std::string home_position_name_ = "home";

    std::unordered_map<uint64_t, RobotPtr> robots_;
    moveit::core::RobotModelConstPtr robot_model_;
    const moveit::core::JointModelGroup* joint_model_group_;
    
    std::vector<RobotID> coordinated_robots_;
};

} // namespace mrmf_core

#endif // MRMF_CORE_MULTI_ROBOT_GROUP_H