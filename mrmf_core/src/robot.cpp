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

double Robot::getMaxVariableVelocity() const
{
	double max_vel = 0.0;

	const moveit::core::JointBoundsVector& jbv = jmg_->getActiveJointModelsBounds();
	for (const moveit::core::JointModel::Bounds* bounds : jbv)
	{
		for (const moveit::core::VariableBounds& bound : *bounds)
		{
			if (bound.velocity_bounded_)
				max_vel = std::fmax(max_vel, bound.max_velocity_);
		}
	}

	return max_vel;
}

double Robot::getMinVariableVelocity() const
{
	double min_vel = std::numeric_limits<double>::max();

	const moveit::core::JointBoundsVector& jbv = jmg_->getActiveJointModelsBounds();
	for (const moveit::core::JointModel::Bounds* bounds : jbv)
	{
		for (const moveit::core::VariableBounds& bound : *bounds)
		{
			if (bound.velocity_bounded_)
				min_vel = std::fmin(min_vel, bound.max_velocity_);
		}
	}

	return min_vel;
}

} // namespace mrmf_core