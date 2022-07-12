#include <mrmf_core/constraint.h>
#include <mrmf_core/kinematics_query_context.h>

namespace mrmf_core
{
PlanarConstraint::PlanarConstraint(const Eigen::Vector3d& normal)
    : normal_(normal)
{
}

void PlanarConstraint::describe(KinematicsQueryContext& context) const
{
    auto* min_dist_goal = new bio_ik::MinimizeDistanceGoal();
    min_dist_goal->setLinkName(context.current_robot->getTipFrame());
    min_dist_goal->setNormal(tf2::Vector3(normal_.x(), normal_.y(), normal_.z()));
 
    context.ik_options.goals.emplace_back(min_dist_goal);    
}

} // namespace mrmf_core