#ifndef  MRMF_CORE_COMPOSITE_INTERPOLATOR
#define MRMF_CORE_COMPOSITE_INTERPOLATOR

#include <moveit/robot_state/cartesian_interpolator.h>
#include <mrmf_core/robot.h>

namespace mrmf_core
{
enum class InterpType { JOINT, LINEAR };

class CompositeInterpolator
{
public:
    static bool interpolate(robot_state::RobotState* start_state, robot_state::RobotState* end_state,
                            std::vector<RobotPtr>& robots, std::vector<InterpType> interp_types,
                            std::vector<int>& coordinated_indices,
                            const moveit::core::MaxEEFStep max_step, 
                            std::vector<robot_state::RobotStatePtr>& traj);

};

} // namespace mrmf_core 

#endif // MRMF_CORE_COMPOSITE_INTERPOLATOR