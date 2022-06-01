#include "mrmf_capability/mrmf_test_service_capability.h"

namespace mrmf_capability
{
MoveGroupMrmfTestService::MoveGroupMrmfTestService()
    : MoveGroupCapability("MrmfTestService"), nh_("~")
{ 
}

void MoveGroupMrmfTestService::initialize()
{
    const std::string node_name = "MoveGroupMrmfTestService";
    ros::NodeHandle rosparam_nh(nh_, node_name);

    mrmf_test_service_ = root_node_handle_.advertiseService(MRMF_TEST_SERVICE_NAME, 
                                                            &MoveGroupMrmfTestService::computeService, this);
}

bool MoveGroupMrmfTestService::computeService(moveit_msgs::GetCartesianPath::Request& req,
                                              moveit_msgs::GetCartesianPath::Response& res)
{
    ROS_WARN("Called MrmfTestService");
    return false;
}

} // namespace mrmf_capability

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(mrmf_capability::MoveGroupMrmfTestService, move_group::MoveGroupCapability)
