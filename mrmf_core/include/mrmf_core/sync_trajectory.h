#ifndef MRMF_CORE_SYNC_TRAJECTORY_H
#define MRMF_CORE_SYNC_TRAJECTORY_H

#include <mrmf_core/cartesian_trajectory.h>

namespace mrmf_core
{
class SynchronizedTrajectory
{
public:
    SynchronizedTrajectory(const std::vector<CartesianTrajectoryPtr>& trajs);
    
    inline size_t size() const { return keypoints_.size(); }

private:
    void initialize(const std::vector<CartesianTrajectoryPtr>& trajs);

private:
    std::set<double> keypoints_;
    std::vector<CartesianTrajectoryPtr> sync_trajs_;
};

} // namespace mrmf_core

#endif // MRMF_CORE_SYNC_TRAJECTORY_H