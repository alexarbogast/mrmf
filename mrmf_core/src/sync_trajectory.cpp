#include <mrmf_core/sync_trajectory.h>
#include <mrmf_core/utils.h>

namespace mrmf_core
{

SynchronousTrajectory::SynchronousTrajectory(const std::vector<CartesianTrajectoryPtr>& trajs)
{
    initialize(trajs);
} 

void SynchronousTrajectory::initialize(const std::vector<CartesianTrajectoryPtr>& trajs)
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
                new_traj->addSuffixWayPoint(CartesianWaypointPtr(interp));
            }
            keypoint_idx++;
        }
        start_idx--;
        end_idx = end_idx == 0 ? keypoint_idx - 1 : end_idx;

        start_end_idx.push_back({start_idx, end_idx});
        sync_trajs_.push_back(new_traj);
    }
}

SyncPointInfo SynchronousTrajectory::getSyncPointInfo(size_t index) const
{
    SyncPointInfo spi;
    
    auto it = keypoints_.begin();
    std::advance(it, index);
    spi.time = *it;
    
    for (size_t i = 0; i < sync_trajs_.size(); i++)
    {
        if (index >= start_end_idx[i][0] && index <= start_end_idx[i][1])
        {
            auto& traj = sync_trajs_[i];
            spi.robots.push_back(traj->getRobot());
            spi.positioners.push_back(traj->getPositioner());
            spi.waypoints.push_back(traj->getWaypoint(index - start_end_idx[i][0]));
        }
    }
    
    return spi;
}

std::vector<RobotID> SynchronousTrajectory::getRobotIDs() const
{
    // remove duplicate if there is more than one trajectory for a robot
    std::set<RobotID> ids_set;
    for (const auto& traj : sync_trajs_)
        ids_set.insert(traj->getRobot());

    std::vector<RobotID> ids(ids_set.begin(), ids_set.end());
    return ids;
}

std::vector<RobotID> SynchronousTrajectory::getPositionerIDs() const
{
    // remove duplicate if there is more than one trajectory for a robot
    std::set<RobotID> ids_set;
    for (const auto& traj : sync_trajs_)
        ids_set.insert(traj->getPositioner());

    std::vector<RobotID> ids(ids_set.begin(), ids_set.end());
    return ids;
}

std::vector<RobotID> SynchronousTrajectory::getAllUniqueIDs() const
{
    std::set<RobotID> ids_set;
    for (const auto& traj : sync_trajs_)
    {
        ids_set.insert(traj->getRobot());
        ids_set.insert(traj->getPositioner());
    }

    std::vector<RobotID> ids(ids_set.begin(), ids_set.end());
    return ids;
}

std::string SynchronousTrajectory::toString() const
{
    std::stringstream ss;
    for (const auto& traj : sync_trajs_)
    {
        ss << traj->toString();
    }
    
    return ss.str();
}

} // namespace mrmf_core