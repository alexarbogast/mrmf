#include <mrmf_capability/synchronous_trajectory_service_capability.h>

namespace mrmf_capability
{
MoveGroupSynchronousTrajectoryService::MoveGroupSynchronousTrajectoryService()
    : MoveGroupCapability("SynchronousTrajectoryService"), nh_("~")
{ 
}

void MoveGroupSynchronousTrajectoryService::initialize()
{
    const std::string node_name = "MoveGroupMrmfTestService";
    ros::NodeHandle rosparam_nh(nh_, node_name);

    synchronous_trajectory_service_ = root_node_handle_.advertiseService(SYNCHRONOUS_TRAJECTORY_SERVICE_NAME, 
        &MoveGroupSynchronousTrajectoryService::computeService, this);

}

bool MoveGroupSynchronousTrajectoryService::computeService(moveit_msgs::GetCartesianPath::Request& req,
                                                           moveit_msgs::GetCartesianPath::Response& res)
{
    ROS_WARN("Called Synchronous Trajectory Service");
    return false;
}

} // namespace mrmf_capability

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(mrmf_capability::MoveGroupSynchronousTrajectoryService, move_group::MoveGroupCapability)
