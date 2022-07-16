#include <mrmf_core/sync_trajectory.h>
#include <mrmf_core/utils.h>

namespace mrmf_core
{

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
                new_traj->addSuffixWayPoint(CartesianWaypointPtr(interp));
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