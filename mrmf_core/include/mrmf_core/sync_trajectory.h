#ifndef MRMF_CORE_SYNC_TRAJECTORY_H
#define MRMF_CORE_SYNC_TRAJECTORY_H

#include <set>
#include <mrmf_core/cartesian_trajectory.h>

namespace mrmf_core
{
struct SyncPointInfo
{
    double time;
    std::vector<CartesianWaypointPtr> waypoints;
    std::vector<RobotID> robots;
    std::vector<RobotID> positioners;

    inline unsigned int nPoints() const { return waypoints.size(); }
};


class SynchronousTrajectory
{
public:
    SynchronousTrajectory(const std::vector<CartesianTrajectoryPtr>& trajs);
    
    inline size_t size() const { return keypoints_.size(); }

    SyncPointInfo getSyncPointInfo(size_t index) const;
    std::vector<RobotID> getRobotIDs() const;
    std::vector<RobotID> getPositionerIDs() const;
    std::vector<RobotID> getAllUniqueIDs() const;

    std::string toString() const;

private:
    void initialize(const std::vector<CartesianTrajectoryPtr>& trajs);
private:
    std::set<double> keypoints_;
    std::vector<CartesianTrajectoryPtr> sync_trajs_;

    std::vector<std::array<size_t, 2>> start_end_idx;
};

} // namespace mrmf_core

#endif // MRMF_CORE_SYNC_TRAJECTORY_H