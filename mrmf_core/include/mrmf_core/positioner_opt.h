#ifndef MRMF_CORE_POSITIONER_OPTIMIZATION
#define MRMF_CORE_POSITIONER_OPTIMIZATION

#include <moveit/robot_state/robot_state.h>

namespace mrmf_core
{
double positionerOptDist2D(EigenSTL::vector_Vector3d& interest_points,
                           EigenSTL::vector_Vector3d& base_positions, 
                           double initial_guess);

} // namespace mrmf_core

#endif // MRMF_CORE_POSITIONER_OPTIMIZATION