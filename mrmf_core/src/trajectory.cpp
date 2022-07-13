#include <mrmf_core/trajectory.h>
#include <mrmf_core/utils.h>

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
    return (getWaypointDistanceFromPrevious(i) / velocity_) + start_time_;
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
    return (getWaypointDistanceFromStart(index) / velocity_) + start_time_;
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
    auto d = getWaypointDistancesFromPrevious();
    std::for_each(d.begin(), d.end(), [this](double& dist){ dist /= velocity_; });
    return d;
}

std::vector<double> CartesianTrajectory::getWaypointDurationsFromStart() const
{
    auto d = getWaypointDistancesFromStart();
    std::for_each(d.begin(), d.end(), [this](double& dist){ dist = (dist / velocity_) + start_time_; });
    return d;
}

CartesianTrajectoryPtr CartesianTrajectory::bisectExpansion(const std::set<double>& times) const
{
    CartesianTrajectoryPtr new_traj = makeTrajectory(robot_, velocity_, positioner_);
    new_traj->setStartTime(start_time_);

    auto old_times = getWaypointDurationsFromStart();

    std::vector<int> test;
    for (auto& time : times)
    {
        int index = binary_search_find_index(old_times, time);
        test.push_back(index);

    }

    std::cout << std::endl;
    std::cout << "Old Times: \n";
    for (auto& t : old_times)
    {
        std::cout << t << std::endl;
    }

    std::cout << std::endl;
    std::cout << "New Times: \n";
    for (auto& t : times)
    {
        std::cout << t << std::endl;
    }

    std::cout << std::endl;
    std::cout << "Insertion Index: \n";
    for (auto& t : test)
    {
        std::cout << t << std::endl;
    }

    return new_traj;
}

std::string CartesianTrajectory::toString() const
{
    std::stringstream ss;
    ss << "CartesianTrajectory: ";
    ss << "\t(Velocity: " <<  velocity_ ;
    ss << "\t Start Time: " << start_time_ << ")\n";

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

        // retain time accuracy down to the ms
        std::transform(times.begin(), times.end(), std::inserter(keypoints, keypoints.begin()), 
                       [](auto& time){ return round(time * 1000) / 1000; });
    }      

    // use keypoints to determine synchronized cartesian coordinates
    std::vector<CartesianTrajectoryPtr> sync_trajs;
    for (auto& traj : trajectories_)
        sync_trajs.push_back(traj->bisectExpansion(keypoints));


    // temp
    for (auto& foo : keypoints)
        std::cout << foo << std::endl;

    std::cout << std::endl;

    for (auto& foo : sync_trajs)
        std::cout << foo->toString() << std::endl;
}

} // namespace mrmf_core