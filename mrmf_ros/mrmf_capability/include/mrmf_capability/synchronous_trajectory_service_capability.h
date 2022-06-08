#ifndef MRMF_SYNCHRONOUS_TRAJECTORY_SERVICE_CAPABILITY_H
#define MRMF_SYNCHRONOUS_TRAJECTORY_SERVICE_CAPABILITY_H

#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/GetCartesianPath.h>

#include <mrmf_capability/capability_names.h>

namespace mrmf_capability
{
class MoveGroupSynchronousTrajectoryService : public move_group::MoveGroupCapability
{
public:
    MoveGroupSynchronousTrajectoryService();

    virtual void initialize();

private:
    bool computeService(moveit_msgs::GetCartesianPath::Request& req, moveit_msgs::GetCartesianPath::Response& res);

    ros::NodeHandle nh_;
    ros::ServiceServer synchronous_trajectory_service_;

};

} // namespace mrmf_capability

#endif // MRMF_SYNCHRONOUS_TRAJECTORY_SERVICE_CAPABILITY_H