#ifndef MRMF_KINEMATICS_QUERY_CONTEXT_H
#define MRMF_KINEMATICS_QUERY_CONTEXT_H

#include <mrmf_core/goal_types.h>
#include <mrmf_core/robot.h>

namespace mrmf_core
{
struct KinematicsQueryContext
{
    KinematicsQueryContext() = default;

    bio_ik::BioIKKinematicsQueryOptions ik_options;
    RobotPtr current_robot;
};

} // namespace mrmf_core

#endif // MRMF_KINEMATICS_QUERY_CONTEXT_H