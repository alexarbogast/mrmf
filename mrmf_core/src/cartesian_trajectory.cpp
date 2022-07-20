#include <mrmf_core/cartesian_trajectory.h>

namespace mrmf_core
{
CartesianTrajectory::CartesianTrajectory(const RobotID& robot, double velocity, const RobotID& positioner)
        : robot_(robot), positioner_(positioner), velocity_(velocity)
{}

CartesianTrajectory::CartesianTrajectory(const CartesianTrajectory& other, bool deepcopy)
{
    *this = other;
    if (deepcopy)
    {
        this->waypoints_.clear();
        for (const auto& waypoint : other.waypoints_)
        {
            this->waypoints_.emplace_back(waypoint->clone());
        }
    }
}

void CartesianTrajectory::insertWayPoint(size_t index, const CartesianWaypointPtr& pt)
{
    waypoints_.insert(waypoints_.begin() + index, pt);
}

CartesianWaypointPtr CartesianTrajectory::getWaypoint(std::size_t index) const
{
    return CartesianWaypointPtr(waypoints_[index]->clone());
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

void CartesianTrajectory::makeDense(double max_step)
{
    std::deque<CartesianWaypointPtr> new_waypoints;
    new_waypoints.push_back(waypoints_[0]);

    for (std::size_t i = 1; i < size(); i++)
    {
        const auto& current = waypoints_[i];
        const auto& prev = waypoints_[i - 1];

        double distance = (current->translation() - prev->translation()).norm();
        std::size_t steps = ceil(distance / max_step);
        double perc_step = 1.0 / (double)steps;

        for (std::size_t j = 1; j <= steps; j++)
        {
            double perc = (double)j * perc_step;
            new_waypoints.push_back(CartesianWaypointPtr(prev->interpolate(current.get(), perc)));
        }
    }

    waypoints_ = new_waypoints;
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

CartesianTrajectory* CartesianTrajectory::emptyCopy() const
{
    return new CartesianTrajectory(robot_, velocity_, positioner_);
}

} // namespace mrmf_core