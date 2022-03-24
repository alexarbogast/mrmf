#ifndef MRMF_GROUP_INTERFACE
#define MRMF_GROUP_INTERFACE

#include <string.h>
#include <moveit/macros/class_forward.h>

namespace mrmf
{
namespace planning_interface
{
class MrmfGroupInterface
{
public:
    static const std::string ROBOT_DESCRIPTION;

    MrmfGroupInterface() = default();

};

} // namespace planning_interface

} // namespace mrmf_ros

#endif // MRMF_GROUP_INTERFACE