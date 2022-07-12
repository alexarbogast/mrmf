#include <mrmf_core/trajectory.h>

namespace mrmf_core
{
CartesianTrajectory::CartesianTrajectory(const RobotID& robot, double velocity, const RobotID& positioner)
        : robot_(robot), positioner_(positioner), velocity_(velocity)
{}

void CartesianTrajectory::insertWayPoint(size_t index, const CartesianTrajectoryPtPtr& pt)
{
    waypoints_.insert(waypoints_.begin() + index, pt);
}

double CartesianTrajectory::getWaypointDistanceFromPrevious(size_t i) const
{
    return waypoints_[i]->distance(*waypoints_[i - 1]);
}

double CartesianTrajectory::getWaypointDurationFromPrevious(size_t i) const
{
    return getWaypointDistanceFromPrevious(i) / velocity_;
}

double CartesianTrajectory::getWaypointDistanceFromStart(size_t index) const
{
    double dist = 0.0;
    for (size_t i = 1; i <= index; i++)
        dist += getWaypointDistanceFromPrevious(i);
    
    return dist;
}

double CartesianTrajectory::getWaypointDurationFromStart(size_t index) const
{
    return getWaypointDistanceFromStart(index) / velocity_;
}

std::vector<double> CartesianTrajectory::getWaypointDistancesFromPrevious() const
{
    std::vector<double> dists(waypoints_.size(), 0);
    for (size_t i = 1; i < waypoints_.size(); i++)
        dists[i] = getWaypointDistanceFromPrevious(i);

    return dists;
}

std::vector<double> CartesianTrajectory::getWaypointDistancesFromStart() const
{
    std::vector<double> dists(waypoints_.size(), 0);
    double dist = 0.0;
    for (size_t i = 1; i < waypoints_.size(); i++)
    {
        dist += getWaypointDistanceFromPrevious(i);
        dists[i] = dist;
    }

    return dists;
}

std::vector<double> CartesianTrajectory::getWaypointDurationsFromPrevious() const
{
    auto dists = getWaypointDistancesFromPrevious();
    std::for_each(dists.begin(), dists.end(), [this](double& dist){ dist /= velocity_; });
    return dists;
}

std::vector<double> CartesianTrajectory::getWaypointDurationsFromStart() const
{
    auto dists = getWaypointDistancesFromStart();
    std::for_each(dists.begin(), dists.end(), [this](double& dist){ dist /= velocity_; });
    return dists;
}

std::string CartesianTrajectory::toString() const
{
    std::stringstream ss;
    for (size_t i = 0; i < waypoints_.size(); i++)
        ss << "[" << i << "] " << waypoints_[i]->toString() << "\n\n";

    return ss.str();
}


void CompositeTrajectory::synchronizeTrajectories() const
{
    // create set of unique keypoints
    // keypoints are points in time that at least one robot is at an explicit waypoint
    std::set<double> keypoints;
    for (auto& traj : trajectories_)
    {
        auto times = traj->getWaypointDurationsFromStart();
        keypoints.insert(times.begin(), times.end());
    }      

    // temp
    for (auto& foo : keypoints)
    {
        std::cout << foo << std::endl;
    }
}

} // namespace mrmf_core