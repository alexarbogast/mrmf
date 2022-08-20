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
    struct Config
    {
        Config() 
        : max_step(0.001), max_velocity_scaling_factor(1.0)
        {
        }

        moveit::core::MaxEEFStep max_step;
        double max_velocity_scaling_factor;
        // orientation_type
        // positioner_optimization_type
    };

    SyncTrajectoryPlanner() = default;
    void initialize(const std::unordered_map<RobotID, RobotPtr>& id_map, 
                    const moveit::core::JointModelGroup* jmg);
    void resetPlanData();

    bool plan(SynchronousTrajectory& traj,
              robot_trajectory::RobotTrajectory& output_traj,
              robot_state::RobotState& seed_state,
              const Config& config);

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
                        std::vector<SyncState>& sync_states,
                        const Config& config) const;

    bool planSyncState(const SyncPointInfo& spi, SyncState& sync_state_out) const;
    bool interpUnplannedStates(std::vector<SyncState>& sync_states, 
                               robot_state::RobotState& seed_state,
                               const Config& config) const;

    double positionerOptimization(const SyncPointInfo& spi,
                                  const RobotID& positioner,
                                  robot_state::RobotState& seed_state) const;

    double computeMinMoveTime(const robot_state::RobotState& start,
                              const robot_state::RobotState& end,
                              const moveit::core::JointModelGroup* group) const;

private:
    std::unordered_map<RobotID, RobotPtr> id_map_;
    const moveit::core::JointModelGroup* jmg_;

    // per plan data
    std::size_t n_sync_points_;
    std::vector<RobotID> unique_ids_;
};

} // namespace mrmf_core

#endif // MRMF_CORE_SYNC_PLANNER_H