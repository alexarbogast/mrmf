#ifndef MRMF_TEST_SERVICE_CAPABILITY_H
#define MRMF_TEST_SERVICE_CAPABILITY_H

#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/GetCartesianPath.h>

#include <mrmf_capability/capability_names.h>

namespace mrmf_capability
{
class MoveGroupMrmfTestService : public move_group::MoveGroupCapability
{
public:
    MoveGroupMrmfTestService();

    virtual void initialize();

private:
    bool computeService(moveit_msgs::GetCartesianPath::Request& req, moveit_msgs::GetCartesianPath::Response& res);

    ros::NodeHandle nh_;
    std::string name_ = "mrmf_test_service_capability";

    ros::ServiceServer mrmf_test_service_;

    std::string current_group_name_;
    std::string current_world_frame_;
    std::string current_tcp_frame_;
};

} // namespace mrmf_capability

#endif // MRMF_TEST_SERVICE_CAPABILITY_H