#include <mrmf_core/trajectory.h>
#include <mrmf_core/utils.h>

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

void CartesianTrajectory::insertWayPoint(size_t index, const CartesianTrajectoryPtPtr& pt)
{
    waypoints_.insert(waypoints_.begin() + index, pt);
}

CartesianTrajectoryPtPtr CartesianTrajectory::getWaypoint(std::size_t index) const
{
    return CartesianTrajectoryPtPtr(waypoints_[index]->clone());
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


SynchronizedTrajectory::SynchronizedTrajectory(const std::vector<CartesianTrajectoryPtr>& trajs)
{
    initialize(trajs);
} 

void SynchronizedTrajectory::initialize(const std::vector<CartesianTrajectoryPtr>& trajs)
{
    // create set of unique keypoints
    // keypoints are points in time that at least one robot is at an explicit waypoint
    std::vector<std::vector<double>> original_times;
    for (auto& traj : trajs)
    {
        auto times = traj->getWaypointDurationsFromStart();

        // retain time accuracy down to the ms
        std::transform(times.begin(), times.end(), std::inserter(keypoints_, keypoints_.begin()),
            [](auto& time){ return time = round(time * 1000) / 1000; });

        original_times.emplace_back(std::move(times));
    }      

    // use keypoints to determine synchronized cartesian coordinates
    for (int i = 0; i < trajs.size(); i++)
    {
        // copy original trajectory data
        CartesianTrajectoryPtr new_traj(trajs[i]->emptyCopy());
        new_traj->addPrefixWayPoint(trajs[i]->getWaypoint(0));

        // populate new trajectory with interpolated waypoints
        bool found_end = false;
        size_t start_idx = 0, end_idx = 0, keypoint_idx = 0;
        for (const auto& keypoint : keypoints_)
        {
            int index = binary_search_find_index_inclusive(original_times[i], keypoint);
            
            if (index == 0)
                start_idx++;
            else if (index == -1)
            {
                end_idx = found_end ? end_idx : keypoint_idx - 1;
                found_end = true;
            }
            else
            {
                double prev_time = original_times[i][index - 1], cur_time = original_times[i][index];
                double t = (keypoint - prev_time) / (cur_time - prev_time);
                
                auto prev_point = trajs[i]->getWaypoint(index - 1);
                auto cur_point = trajs[i]->getWaypoint(index);

                auto interp = prev_point->interpolate(cur_point.get(), t);
                new_traj->addSuffixWayPoint(CartesianTrajectoryPtPtr(interp));
            }
            keypoint_idx++;
        }
        start_idx--;
        end_idx = end_idx == 0 ? keypoint_idx - 1 : end_idx;
        sync_trajs_.push_back(new_traj);
    }

    for (const auto& traj : sync_trajs_)
    {
        std::cout << traj->toString() << std::endl;
    }
}

} // namespace mrmf_core