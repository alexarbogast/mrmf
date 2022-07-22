#include <mrmf_core/robot.h>

namespace mrmf_core
{
Robot::Robot(const std::string& group, 
             const std::string& tip_frame,
             const std::string& base_frame,
             const moveit::core::JointModelGroup* jmg,
             const Robot::RobotType type)
    : group_(group), tip_frame_(tip_frame), base_frame_(base_frame), jmg_(jmg),
      type_(type), id_(RobotID::make_id())
{
}


} // namespace mrmf_core