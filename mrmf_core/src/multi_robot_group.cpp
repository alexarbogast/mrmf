#include <mrmf_core/multi_robot_group.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace mrmf_core
{
MultiRobotGroup::MultiRobotGroup(const std::string& group, const moveit::core::RobotModelConstPtr& robot_model)
    : group_(group), 
      robot_model_(robot_model),
      joint_model_group_(robot_model_->getJointModelGroup(group_))
{
}

RobotID MultiRobotGroup::addRobot(const std::string& group,  
                                  const std::string& tip_frame,
                                  const Robot::RobotType type)
{
    const std::string& model_frame = robot_model_->getModelFrame();
    return addRobot(group, tip_frame, model_frame, type);
}

RobotID MultiRobotGroup::addRobot(const std::string& group,  
                                  const std::string& tip_frame,
                                  const std::string& base_frame,
                                  const Robot::RobotType type)
{
    auto jmg = robot_model_->getJointModelGroup(group);
    
    if (!jmg) return RobotID::make_nil();
    if (!jmg->hasLinkModel(tip_frame) || !robot_model_->hasLinkModel(base_frame))
    {
        ROS_ERROR("Failed to add robot to MultiRobotGroup");
        return RobotID::make_nil();
    }

    RobotPtr robot = std::make_shared<Robot>(group, tip_frame, base_frame, type);
        
    RobotID id = robot->getID();
    robots_[id.value()] = robot;
    
    return id;
}

RobotPtr MultiRobotGroup::getRobot(const RobotID& id)
{
    return robots_.at(id.value());
}

const RobotPtr MultiRobotGroup::getRobot(const RobotID& id) const
{
    return robots_.at(id.value());
}

bool MultiRobotGroup::planMultiRobotTrajectory(SynchronousTrajectory& traj, 
                                               robot_trajectory::RobotTrajectory& output_traj,
                                               moveit::core::RobotState& seed_state)
{
    robot_state::RobotState current_state = seed_state;
    size_t n = traj.size();

    for (size_t i = 0; i < n; i++)
    {
        SyncPointInfo spi = traj.getSyncPointInfo(i);

        // solver for coordinated robots first using unique positioner ids
        std::set<RobotID> unique_pos(spi.positioners.begin(), spi.positioners.end());
        for (auto& p : unique_pos)
        {
            auto jmg = robot_model_->getJointModelGroup(getRobot(p)->getGroup());
            
            // Solve the positioner optimization problem
            double pos_value = positionerOptimization(spi, p, current_state);
            current_state.setJointGroupPositions(jmg, {pos_value});
        }
    }

    return false;
}

bool MultiRobotGroup::kinematicsQuery(KinematicsQueryContext& context, robot_state::RobotState& seed_state)
{
    bool success = seed_state.setFromIK(
        joint_model_group_,              // joints to be used for IK
        EigenSTL::vector_Isometry3d(),  // empty end_effector positions
        std::vector<std::string>(),     // empty tip link names
        0.05,                          // solver timeout
        moveit::core::GroupStateValidityCallbackFn(),
        context.ik_options                      // ik constraints
    );

    return success;
}

void MultiRobotGroup::addGlobalConstraints(KinematicsQueryContext& context)
{
    auto* minimal_displacement = new bio_ik::MinimalDisplacementGoal();
    context.ik_options.goals.emplace_back(minimal_displacement);

    context.ik_options.replace = true;
    context.ik_options.return_approximate_solution = true;
}


/* Use the positioner to minimize the 2D distance between each
 * robot end effector and the robot base 
 * 
 * for now we will assume there is only one positioner
 */
double MultiRobotGroup::positionerOptimization(const SyncPointInfo& spi,
                                               const RobotID& positioner,
                                               robot_state::RobotState& seed_state) const
{
    EigenSTL::vector_Vector3d interest_points;
    EigenSTL::vector_Vector3d base_positions;

    // update frame transformations in seed state
    seed_state.update();

    for (int i = 0; i < spi.nPoints(); i++)
    {
        if (spi.positioners[i] == positioner)
        {
            interest_points.push_back(spi.waypoints[i]->translation());
            
            const std::string& base_name = getRobot(spi.robots[i])->getBaseFrame();
            base_positions.push_back(seed_state.getFrameTransform(base_name).translation());
        }
    }

    // create objective function
    auto objective = [interest_points, base_positions](double theta)
    {
        double cost = 0.0;

        Eigen::Rotation2Dd rot2(theta);
        for (int i = 0; i < interest_points.size(); i++)
        {
            Eigen::Vector2d pt2 = interest_points[i].head<2>();
            cost += ((rot2 * pt2) - base_positions[i].head<2>()).norm();
        }
        return cost;
    };

    auto gradient = [objective](double x_old, double step)
    {
        double x_new = x_old + step;  
        return (objective(x_new) - objective(x_old)) / step;
    };

    // gradient descent
    static unsigned int max_iter = 1000;
    static double tol = 1e-5, step_size = 0.1, gradient_step = 0.00001;

    unsigned int iter_count = 0;
    double gradientMagnitude = 1.0;

    double xk = seed_state.getVariablePosition(0);
    while ((iter_count < max_iter) && (gradientMagnitude > tol))
    {
        double gr = gradient(xk, gradient_step);
        gradientMagnitude = sqrt(gr * gr);
        xk += -(gr * step_size);

        iter_count++;
    }

    std::cout << "Iterations: " << iter_count << std::endl;
    std::cout << "Gradient mag: " << gradientMagnitude << std::endl;
    std::cout << "Sol: " << xk << std::endl << std::endl;

    return xk;
}

} // namespace mrmf_core