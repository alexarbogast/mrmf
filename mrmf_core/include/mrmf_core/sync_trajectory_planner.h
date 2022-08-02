#ifndef MRMF_CORE_SYNC_PLANNER_H
#define MRMF_CORE_SYNC_PLANNER_H

#include <unordered_map>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <mrmf_core/robot.h>
#include <mrmf_core/sync_trajectory.h>
#include <mrmf_core/composite_interpolator.h>

namespace mrmf_core
{
class SyncTrajectoryPlanner
{
public:
    SyncTrajectoryPlanner() = default;
    void initialize(const std::unordered_map<RobotID, RobotPtr>& id_map);
    void reset();

    bool plan(SynchronousTrajectory& traj,
              robot_trajectory::RobotTrajectory& output_traj,
              robot_state::RobotState& seed_state,
              const moveit::core::MaxEEFStep max_step);

private:
    struct SyncState
    {
        std::vector<InterpType> interp_types;
        std::vector<int> coordinated_indices;
        std::vector<bool> planned_flags;
        robot_state::RobotState state;
        double time = 0.0;

        // default initialize interp_types to JOINT and coordinated_indices to -1
        SyncState(std::size_t n_robots, const robot_state::RobotState& s)
            : interp_types(n_robots, InterpType::JOINT), 
              coordinated_indices(n_robots, -1),
              planned_flags(n_robots, false),
              state(s)
        {
        }
    };
    
    bool planSyncStates(SynchronousTrajectory& traj,
                        robot_state::RobotState& seed_state,
                        std::vector<SyncState>& sync_states);

    bool planSyncState(const SyncPointInfo& spi, SyncState& sync_state_out) const;

    double positionerOptimization(const SyncPointInfo& spi,
                                  const RobotID& positioner,
                                  robot_state::RobotState& seed_state) const;

private:
    std::unordered_map<RobotID, RobotPtr> id_map_;

    // per plan data
    std::size_t n_sync_points_;
    std::vector<RobotID> unique_ids_;
};

} // namespace mrmf_core

#endif // MRMF_CORE_SYNC_PLANNER_H