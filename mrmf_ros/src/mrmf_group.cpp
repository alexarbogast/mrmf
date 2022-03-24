#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

namespace mrmf_group
{
static const std::string NODE_NAME = "mrmf_group";

class MrmfGroupExe
{
public:
    MrmfGroupExe(const moveit_cpp::MoveItCppPtr& moveit_cpp, bool debug)
        : node_handle_("~")
    {

    }
private:
    ros::NodeHandle node_handle_;
};

} // namespace mrmf_ros

int main(int argc, char** argv)
{
    ros::init(argc, argv, mrmf_group::NODE_NAME);

    ros::AsyncSpinner spinner(1);
    spinner.start();    

    // Load MoveItCpp parameters
    ros::NodeHandle pnh("~");
    moveit_cpp::MoveItCpp::Options moveit_cpp_options(pnh);

    // Prepare PlanningPipelineOptions
    moveit_cpp_options.planning_pipeline_options.parent_namespace = pnh.getNamespace() + "/planning_pipelines";
    XmlRpc::XmlRpcValue planning_pipeline_configs;
    if (pnh.getParam("planning_pipelines", planning_pipeline_configs))
    {
        if (planning_pipeline_configs.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_ERROR("Failed to read parameter 'mrmf_group/planning_pipelines'");
        }
        else
        {
            for (std::pair<const std::string, XmlRpc::XmlRpcValue>& config : planning_pipeline_configs)
            {
                moveit_cpp_options.planning_pipeline_options.pipeline_names.push_back(config.first);
            }
        }
    }

    // Initialize MoveItCpp
    const auto tf_buffer = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0));
    const auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(moveit_cpp_options, pnh, tf_buffer);
    const auto planning_scene_monitor = moveit_cpp->getPlanningSceneMonitor();

    if (planning_scene_monitor->getPlanningScene())
    {
        ros::waitForShutdown();
    }
    else
    {
        ROS_ERROR("Planning scene not configured");
    }

    return 0;
}