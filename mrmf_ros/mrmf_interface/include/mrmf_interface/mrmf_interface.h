/**
 * @file mrmf_interface.h
 * @author Alex Arbogast
 * @brief 
 * @version 0.1
 * @date 2022-05-30
 *   
 **/
#include <moveit/robot_model/robot_model.h>

#ifndef MRMF_INTERFACE_H
#define MRMF_INTERFACE_H

namespace mrmf_interface
{
class MrmfInterface
{
public:
    static const std::string ROBOT_DESCRIPTION;

    struct Options
    {
        Options(std::string group_name, std::string desc = ROBOT_DESCRIPTION,
                const ros::NodeHandle& node_handle = ros::NodeHandle())
            : group_name_(std::move(group_name)), robot_description_(desc), node_handle_(node_handle)
        {
        }

        // combined robot planning group name
        std::string group_name_;

        // The robot description parameter name (if different from default)
        std::string robot_description_;

        // Optionally, an instance of the RobotModel to use can be also specified
        moveit::core::RobotModelConstPtr robot_model_;
    
        ros::NodeHandle node_handle_;
    };

    MrmfInterface(const Options& opt);
    MrmfInterface(const std::string& group);

    ~MrmfInterface();

    bool testService() const;

private:
    class MrmfInterfaceImpl;
    std::unique_ptr<MrmfInterfaceImpl> impl_;
};

} // namespace mrmf_interface

#endif // MRMF_INTERFACE_H
