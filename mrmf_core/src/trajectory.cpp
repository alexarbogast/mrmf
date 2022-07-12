#include <mrmf_core/trajectory.h>

namespace mrmf_core
{
CartesianTrajectory::CartesianTrajectory(const RobotID& robot, double velocity, const RobotID& positioner)
        : robot_(robot), positioner_(positioner), velocity_(velocity)
{}

std::string CartesianTrajectory::toString() const
{
    std::stringstream ss;
    for (size_t i = 0; i < waypoints_.size(); i++)
        ss << "[" << i << "] " << waypoints_[i]->toString() << "\n\n";

    return ss.str();
}


void CompositeTrajectory::synchronizeTrajectories() const
{
    
}

} // namespace mrmf_core