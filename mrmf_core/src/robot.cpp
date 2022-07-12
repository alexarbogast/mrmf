#include <mrmf_core/robot.h>

namespace mrmf_core
{

Robot::Robot(const std::string& group, 
      const std::string& tip_frame,
      const std::string& default_pose_frame,
      const Robot::RobotType type)
    : group_(group), tip_frame_(tip_frame), default_pose_frame_(default_pose_frame), type_(type), id_(RobotID::make_id())
{
}

void Robot::addPersistentConstraint(const ConstraintPtr& constraint)
{
    constraints_.push_back(constraint);
}

void Robot::describePersistentConstraints(KinematicsQueryContext& context) const
{
    for (auto& constraint : constraints_)
        constraint->describe(context);
}

} // namespace mrmf_core