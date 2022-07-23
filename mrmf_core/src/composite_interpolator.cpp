#include <mrmf_core/composite_interpolator.h>

namespace mrmf_core
{

bool CompositeInterpolator::interpolate(robot_state::RobotState* start_state, robot_state::RobotState* end_state,
                                        std::vector<RobotPtr>& robots, std::vector<InterpType> interp_types,
                                        std::vector<int>& coordinated_indices,
                                        const moveit::core::MaxEEFStep max_step,
                                        std::vector<robot_state::RobotStatePtr>& traj)
{
    if (!(robots.size() == interp_types.size()))
    {
        ROS_ERROR("Interpolation failed because robots and interp_types must be equal lengths");
        return false;
    }

    if (max_step.translation <= 0.0 && max_step.rotation <= 0.0)
    {
        ROS_ERROR("Invalid MaxEEFStep passed into computeCartesianPath. Both the MaxEEFStep.rotation and "
                  "MaxEEFStep.translation components must be non-negative and at least one component must be "
                  "greater than zero");
        return false;
    }

    std::size_t n = robots.size();

    // determine the farthest linear movement and store start and end points for cartesian interpolation
    std::vector<std::size_t> lin_move_ind;
    std::vector<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> start_end_pose;
    std::vector<std::pair<Eigen::Quaterniond, Eigen::Quaterniond>> start_end_rot;

    std::size_t steps = 0;
    for (std::size_t i = 0; i < n; i++)
    {
        if (interp_types[i] == InterpType::LINEAR)
        {
            Eigen::Isometry3d start_pose = start_state->getGlobalLinkTransform(robots[i]->getTipFrame());
            Eigen::Isometry3d end_pose = end_state->getGlobalLinkTransform(robots[i]->getTipFrame());

            // convert pose to coordinated frame if necessary 
            if (coordinated_indices[i] > 0)
            {
                std::string coord_tip_frame = robots[coordinated_indices[i]]->getTipFrame();
                Eigen::Isometry3d T_world_pos_start = start_state->getGlobalLinkTransform(coord_tip_frame);
                Eigen::Isometry3d T_world_pos_end = end_state->getGlobalLinkTransform(coord_tip_frame);

                start_pose = T_world_pos_start.inverse() * start_pose;
                end_pose = T_world_pos_end.inverse() * end_pose;
            }

            Eigen::Quaterniond start_quaternion(start_pose.linear());
            Eigen::Quaterniond end_quaternion(end_pose.linear());

            double translation_distance = (end_pose.translation() - start_pose.translation()).norm();
            double rotation_distance = start_quaternion.angularDistance(end_quaternion);

            std::size_t translation_steps = 0, rotation_steps = 0;
            if (max_step.translation > 0.0)
                translation_steps = floor(translation_distance / max_step.translation);

            if (max_step.rotation > 0.0)
                rotation_steps = floor(rotation_distance / max_step.rotation);

            steps = fmax(steps, fmax(translation_steps, rotation_steps));

            // store start and end points for cartesian interpolation
            start_end_pose.emplace_back(start_pose, end_pose);
            start_end_rot.emplace_back(start_quaternion, end_quaternion);
            lin_move_ind.push_back(i);
        }
    }

    traj.clear();
    traj.push_back(std::make_shared<robot_state::RobotState>(*start_state));

    // joint and cartesian interpolation at each step
    robot_state::RobotState current_state = *start_state;
    for (std::size_t i = 1; i <= steps; ++i)
    {
        double percentage = (double)i / (double)steps;
        start_state->interpolate(*end_state, percentage, current_state);

        // replace joint values with IK solution for robots with linear movements
        for (std::size_t j = 0; j < lin_move_ind.size(); j++)
        {
            const auto& se_pose = start_end_pose[j];
            const auto& se_rot = start_end_rot[j];

            Eigen::Isometry3d pose(se_rot.first.slerp(percentage, se_rot.second));
            pose.translation() = percentage * se_pose.second.translation() + 
                                 (1 - percentage) * se_pose.first.translation();

            // convert pose to coordinated frame if necessary 
            if (coordinated_indices[lin_move_ind[j]] > 0)
            {
                std::string coord_tip_frame = robots[coordinated_indices[lin_move_ind[j]]]->getTipFrame();
                Eigen::Isometry3d T_world_pos = current_state.getGlobalLinkTransform(coord_tip_frame);

                pose = T_world_pos * pose;
            }

            auto& robot = robots[lin_move_ind[j]];
            if (!current_state.setFromIK(robot->getJointModelGroup(), pose, robot->getTipFrame(), 0.0))
                return false;   
        }

        traj.push_back(std::make_shared<robot_state::RobotState>(current_state));
    }

    return true;
}

} // namespace mrmf_core