#include <mrmf_core/sync_trajectory_planner.h>
#include <mrmf_core/positioner_opt.h>

namespace mrmf_core
{
void SyncTrajectoryPlanner::initialize(const std::unordered_map<RobotID, RobotPtr>& id_map,
                                       const moveit::core::JointModelGroup* jmg)
{
    id_map_ = id_map;
    jmg_ = jmg;
}

void SyncTrajectoryPlanner::resetPlanData()
{
    // these member variables are for ease of access between member functions
    this->n_sync_points_ = 0;
    this->unique_ids_.clear();
}

bool SyncTrajectoryPlanner::plan(SynchronousTrajectory& traj,
                                 robot_trajectory::RobotTrajectory& output_traj,
                                 robot_state::RobotState& seed_state,
                                 const Config& config)
{
    this->resetPlanData();
    n_sync_points_ = traj.size();
    unique_ids_ = traj.getAllUniqueIDs();

    // find the robot states at each synchronized point
    std::vector<SyncState> sync_states;
    if (!planSyncStates(traj, seed_state, sync_states, config))
    {
        ROS_ERROR("Failed to plan sync states");
        return false;
    }

    // find robot ptrs for each unique robot in the trajectory
    std::vector<RobotPtr> robots;
    for (auto& bot_id : unique_ids_)
        robots.push_back(id_map_.at(bot_id));

    // add seed state and first sync point to output trajectory
    output_traj.clear();
    output_traj.addSuffixWayPoint(seed_state, 0);
    double first_move_time = computeMinMoveTime(seed_state, sync_states[0].state, jmg_) / 
        config.max_velocity_scaling_factor;
    output_traj.addSuffixWayPoint(sync_states[0].state, first_move_time);

    // perform composite interpolation between each sync point
    for (std::size_t i = 1; i < sync_states.size(); i++)
    {
        auto& start_state = sync_states[i - 1];
        auto& end_state = sync_states[i];

        std::vector<robot_state::RobotStatePtr> robot_states;
        bool success = CompositeInterpolator::interpolate(&start_state.state, &end_state.state, 
                        robots, end_state.interp_types, end_state.coordinated_indices, config.max_step, robot_states);

        if (!success)
        {
            ROS_ERROR_STREAM("Failed during composite interpolation between sync_states: " << i - 1 << " and " << i);
            return false;
        }

        // TODO: find duration between each point and add to output trajectory
        double dt = (end_state.time - start_state.time) / robot_states.size();
        for (auto& robot_state : robot_states)
        {
            output_traj.addSuffixWayPoint(robot_state, dt);
        }
    }

    double last_move_time = computeMinMoveTime(sync_states.back().state, seed_state, jmg_) /
        config.max_velocity_scaling_factor;
    output_traj.addSuffixWayPoint(seed_state, last_move_time);

    return true;
}

bool SyncTrajectoryPlanner::planSyncStates(SynchronousTrajectory& traj,
                                           robot_state::RobotState& seed_state,
                                           std::vector<SyncState>& sync_states,
                                           const Config& config) const
{
    robot_state::RobotState current_state = seed_state;
    for (std::size_t i = 0; i < n_sync_points_; i++)
    {
        SyncPointInfo spi = traj.getSyncPointInfo(i);
        SyncState sync_state(unique_ids_.size(), current_state);
        
        if (!planSyncState(spi, sync_state))
        {
            ROS_ERROR_STREAM("Failed to plan sync state at index: " << i);
            return false;
        }

        sync_states.push_back(sync_state);
        current_state = sync_state.state;
    }

    interpUnplannedStates(sync_states, seed_state, config);
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

            int pos_idx = std::distance(unique_ids_.begin(), std::find(unique_ids_.begin(), unique_ids_.end(), p));
            sync_state_out.planned_flags[pos_idx] = true;
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

            int pos_idx = std::distance(unique_ids_.begin(), std::find(unique_ids_.begin(), unique_ids_.end(), p));
            sync_state_out.coordinated_indices[robot_idx] = pos_idx;
        }

        // solve ik
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
 * Interpolate any states with false planned_flags for a synchronous trajectory
 * these are typically joint movements to and from cartesian paths
 */
bool SyncTrajectoryPlanner::interpUnplannedStates(std::vector<SyncState>& sync_states, 
                                                  robot_state::RobotState& seed_state,
                                                  const Config& config) const
{
    for (std::size_t i = 0; i < unique_ids_.size(); i++)
    {
        auto& robot_ptr = id_map_.at(unique_ids_[i]);
        for (std::size_t j = 0; j < n_sync_points_; j++)
        {
            auto& sync_state = sync_states[j];
            if (!sync_state.planned_flags[i])
            {
                auto sync_state_start_it = std::next(sync_states.begin(), j);
                auto sync_state_end_it = sync_state_start_it;

                // find the next state that is planned for this id
                for (; sync_state_end_it != sync_states.end(); ++sync_state_end_it)
                    if ((*sync_state_end_it).planned_flags[i])
                        break;

                if (sync_state_start_it == sync_states.begin())  // travel move from seed to cartesian start
                {
                    double min_move_time = computeMinMoveTime(sync_state_end_it->state, seed_state,
                        robot_ptr->getJointModelGroup()) / config.max_velocity_scaling_factor;
                    
                    auto sync_state_end_rit = std::make_reverse_iterator(sync_state_end_it);
                    const auto& front_planned = std::prev(sync_state_end_rit);
                    front_planned->coordinated_indices[i] = -1;
                    front_planned->interp_types[i] = InterpType::JOINT;

                    for (; sync_state_end_rit != sync_states.rend(); ++sync_state_end_rit)
                    {
                        double perc = (front_planned->time - sync_state_end_rit->time) / min_move_time;
                        perc = perc < 1 ? perc : 1;

                        front_planned->state.interpolate(seed_state, perc, sync_state_end_rit->state, 
                            robot_ptr->getJointModelGroup());

                        sync_state_end_rit->planned_flags[i] = true;
                    }
                }
                else if (sync_state_end_it == sync_states.end()) // travel move from cartesian end to seed
                {
                    const auto& back_planned = std::prev(sync_state_start_it);

                    double min_move_time = computeMinMoveTime(back_planned->state, seed_state,
                        robot_ptr->getJointModelGroup()) / config.max_velocity_scaling_factor;

                    for (; sync_state_start_it != sync_states.end(); ++sync_state_start_it)
                    {
                        double perc = (sync_state_start_it->time - back_planned->time) / min_move_time;
                        perc = perc < 1 ? perc : 1;

                        back_planned->state.interpolate(seed_state, perc, sync_state_start_it->state,
                            robot_ptr->getJointModelGroup());

                        sync_state_start_it->planned_flags[i] = true;
                    }
                }
                else // intermediate travel
                {
                    const auto& back_planned = std::prev(sync_state_start_it);
                    double dt = sync_state_end_it->time - back_planned->time;

                    Eigen::Isometry3d start_pose = back_planned->state.getGlobalLinkTransform(robot_ptr->getTipFrame());
                    Eigen::Isometry3d end_pose = sync_state_end_it->state.getGlobalLinkTransform(robot_ptr->getTipFrame());

                    Eigen::Quaterniond start_quaternion(start_pose.linear());
                    Eigen::Quaterniond end_quaternion(end_pose.linear());

                    for (; sync_state_start_it != sync_state_end_it; ++sync_state_start_it)
                    {
                        double perc = (sync_state_start_it->time - back_planned->time) / dt;
                        perc = perc < 1 ? perc : 1;

                        Eigen::Isometry3d pose(start_quaternion.slerp(perc, end_quaternion));
                        pose.translation() = perc * end_pose.translation() + (1 - perc) * start_pose.translation();

                        sync_state_start_it->state.setFromIK(robot_ptr->getJointModelGroup(), pose, robot_ptr->getTipFrame(), 0.0);
                        sync_state_start_it->planned_flags[i] = true;
                        sync_state_start_it->interp_types[i] = InterpType::LINEAR;
                    }
                }
            }
        }
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

/* 
 * Get the minimum move time between the start and end state that does not exceed the maximum
 * velocity threshold of the joint model group
 */
double SyncTrajectoryPlanner::computeMinMoveTime(const robot_state::RobotState& start, 
                                                 const robot_state::RobotState& end,
                                                 const moveit::core::JointModelGroup* group) const
{
    using namespace moveit::core;
    double min_time = 0.0;

    const std::vector<const JointModel*>& jm = group->getActiveJointModels();
    for (const JointModel* joint_id : jm)
    {
        const int idx = joint_id->getFirstVariableIndex();
        const std::vector<VariableBounds>& bounds = joint_id->getVariableBounds();

        const double* start_position = start.getVariablePositions();
        const double* end_position = end.getVariablePositions();

        for (std::size_t var_id = 0; var_id < joint_id->getVariableCount(); ++var_id)
        {
            const double dtheta = std::abs(*(start_position + idx + var_id) - *(end_position + idx + var_id));
            min_time = std::fmax(min_time, dtheta / bounds[var_id].max_velocity_);
        }
    }

    return min_time;
}

} // namespace mrmf_core