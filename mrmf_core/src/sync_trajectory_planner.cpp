#include <mrmf_core/sync_trajectory_planner.h>
#include <mrmf_core/positioner_opt.h>

namespace mrmf_core
{
void SyncTrajectoryPlanner::initialize(const std::unordered_map<RobotID, RobotPtr>& id_map)
{
    id_map_ = id_map;
}

void SyncTrajectoryPlanner::reset()
{
    this->n_sync_points_ = 0;
    this->unique_ids_.clear();
}

bool SyncTrajectoryPlanner::plan(SynchronousTrajectory& traj,
                                  robot_trajectory::RobotTrajectory& output_traj,
                                  robot_state::RobotState& seed_state,
                                  const moveit::core::MaxEEFStep max_step)
{
    this->reset();
    n_sync_points_ = traj.size();
    unique_ids_ = traj.getAllUniqueIDs();

    // find the robot states at each synchronized point
    std::vector<SyncState> sync_states;
    if (!planSyncStates(traj, seed_state, sync_states))
    {
        ROS_ERROR("Failed to plan sync state");
        return false;
    }
 
    // find robot ptrs for each unique robot in the trajectory
    std::vector<RobotPtr> robots;
    for (auto& bot_id : unique_ids_)
        robots.push_back(id_map_.at(bot_id));

    // perform composite interpolation between each sync point
    output_traj.clear();
    output_traj.addSuffixWayPoint(seed_state, 0);           // start joint state
    output_traj.addSuffixWayPoint(sync_states[0].state, 3); // first sync point 

    for (std::size_t i = 1; i < sync_states.size(); i++)
    {
        auto& start_state = sync_states[i - 1];
        auto& end_state = sync_states[i];

        std::vector<robot_state::RobotStatePtr> robot_states;
        bool success = CompositeInterpolator::interpolate(&start_state.state, &end_state.state, 
                            robots, end_state.interp_types, end_state.coordinated_indices, max_step, robot_states);

        if (!success)
        {
            ROS_ERROR("Failed during composite interpolation");
            return false;
        }

        // TODO: find duration between each point and add to output trajectory
        double dt = (end_state.time - start_state.time) / robot_states.size();
        for (auto& robot_state : robot_states)
        {
            output_traj.addSuffixWayPoint(robot_state, dt);
        }
    }

    return true;
}

bool SyncTrajectoryPlanner::planSyncStates(SynchronousTrajectory& traj,
                                           robot_state::RobotState& seed_state,
                                           std::vector<SyncState>& sync_states)
{
    robot_state::RobotState current_state = seed_state;
    for (std::size_t i = 0; i < n_sync_points_; i++)
    {
        SyncPointInfo spi = traj.getSyncPointInfo(i);
        SyncState sync_state(unique_ids_.size(), current_state);
        
        if (!planSyncState(spi, sync_state))
            return false;

        sync_states.push_back(sync_state);
        current_state = sync_state.state;
    }

    // TODO: interpolate joint movement that have not been planned
    return true;
}

/* 
 * Use sync point info to solve IK and fill in missing information for a sync state
 */
bool SyncTrajectoryPlanner::planSyncState(const SyncPointInfo& spi, SyncState& sync_state_out) const
{
    robot_state::RobotState& current_state = sync_state_out.state;
    sync_state_out.time = spi.time;

    // find the positioners used at this sync point
    std::set<RobotID> unique_pos(spi.positioners.begin(), spi.positioners.end());
    for (auto& p : unique_pos)
    {
        if (!p.is_nil())
        {
            auto jmg = id_map_.at(p)->getJointModelGroup();

            double pos_value = positionerOptimization(spi, p, current_state);
            current_state.setJointGroupPositions(jmg, {pos_value});
        }
    }

    current_state.update();
    for (unsigned int i = 0; i < spi.nPoints(); i++)
    {
        auto& robot = spi.robots[i];
        auto& p = spi.positioners[i];

        // mark all robots with sync points as planned and indicate coordinated units
        auto robot_it = std::find(unique_ids_.begin(), unique_ids_.end(), robot);
        std::size_t robot_idx = std::distance(unique_ids_.begin(), robot_it);
        
        sync_state_out.interp_types[robot_idx] = InterpType::LINEAR;
        sync_state_out.planned_flags[robot_idx] = true;

        if (!p.is_nil())
        {
            // convert waypoints to coordinated frame
            auto& T_world_pos = current_state.getFrameTransform(id_map_.at(p)->getTipFrame());
            spi.waypoints[i]->getPose() = T_world_pos * spi.waypoints[i]->getPose();

            auto pos_it = std::find(unique_ids_.begin(), unique_ids_.end(), p);
            int pos_id = std::distance(unique_ids_.begin(), pos_it);
            sync_state_out.coordinated_indices[robot_idx] = pos_id;
        }

        // solve ik
        //auto& current_robot = id_map_.at(spi.robots[i]);
        auto& robot_ptr = id_map_.at(robot);
        bool success = current_state.setFromIK(robot_ptr->getJointModelGroup(), 
                                               spi.waypoints[i]->getPose(),
                                               robot_ptr->getTipFrame(),
                                               0.0);

        if (!success)
            return false;
    }

    return true;
}

/* 
 * Use the positioner to minimize the 2D distance between each
 * robot end effector and the robot base 
 */
double SyncTrajectoryPlanner::positionerOptimization(const SyncPointInfo& spi,
                                                     const RobotID& positioner,
                                                     robot_state::RobotState& seed_state) const
{
    EigenSTL::vector_Vector3d interest_points;
    EigenSTL::vector_Vector3d base_positions;

    // update frame transformations in seed state
    seed_state.update();

    // store the static base frame positions of robots coordinated with this positioner
    for (int i = 0; i < spi.nPoints(); i++)
    {
        if (spi.positioners[i] == positioner)
        {
            interest_points.push_back(spi.waypoints[i]->translation());
            
            const std::string& base_name = id_map_.at(spi.robots[i])->getBaseFrame();
            base_positions.push_back(seed_state.getFrameTransform(base_name).translation());
        }
    }

    return positionerOptDist2D(interest_points, base_positions, seed_state.getVariablePosition(0));
}

} // namespace mrmf_core