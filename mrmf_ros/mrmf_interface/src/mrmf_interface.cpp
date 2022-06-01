#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit_msgs/GetCartesianPath.h>

#include <mrmf_interface/mrmf_interface.h>
#include <mrmf_capability/capability_names.h>

namespace mrmf_interface
{
const std::string MrmfInterface::ROBOT_DESCRIPTION = "robot_description";

class MrmfInterface::MrmfInterfaceImpl
{
    friend MrmfInterface;

public:
    MrmfInterfaceImpl(const Options& opt)
        : opt_(opt), node_handle_(opt.node_handle_)
    {
        
        robot_model_ = opt.robot_model_ ? opt.robot_model_ : 
            moveit::planning_interface::getSharedRobotModel(opt.robot_description_);
        if (!getRobotModel())
        {
          std::string error = "Unable to construct robot model. Please make sure all needed information is on the "
                              "parameter server.";
          ROS_FATAL_STREAM(error);
          throw std::runtime_error(error);
        }

        if (!getRobotModel()->hasJointModelGroup(opt.group_name_))
        {
          std::string error = "Group '" + opt.group_name_ + "' was not found.";
          ROS_FATAL_STREAM(error);
          throw std::runtime_error(error);
        }

        //TODO: add robot sub group names to options and check for existance

        joint_model_group_ = getRobotModel()->getJointModelGroup(opt.group_name_);

        test_service_ = 
            node_handle_.serviceClient<moveit_msgs::GetCartesianPath>(mrmf_capability::MRMF_TEST_SERVICE_NAME);
    }

    ~MrmfInterfaceImpl() = default;

    const Options& getOptions() const
    {
        return opt_;
    }

    const moveit::core::RobotModelConstPtr& getRobotModel() const
    {
        return robot_model_;
    }

    bool testService()
    {
        moveit_msgs::GetCartesianPath::Request req;
        moveit_msgs::GetCartesianPath::Response res;

        if (test_service_.call(req, res))
        {
            return true;
        }
        return false;
    }

private:
    Options opt_;
    ros::NodeHandle node_handle_;

    moveit::core::RobotModelConstPtr robot_model_;
    const moveit::core::JointModelGroup* joint_model_group_;

    // ROS communication
    ros::ServiceClient test_service_;
};

MrmfInterface::MrmfInterface(const std::string& group)
    : impl_(new MrmfInterfaceImpl(Options(group)))
{
}

MrmfInterface::MrmfInterface(const MrmfInterface::Options& options)
    : impl_(new MrmfInterfaceImpl(options))
{
}

MrmfInterface::~MrmfInterface() = default;

bool MrmfInterface::testService() const
{
    return impl_->testService();
}

} // namespace mrmf_interface