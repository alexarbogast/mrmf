#ifndef MRMF_KINEMATICS_QUERY_CONTEXT_H
#define MRMF_KINEMATICS_QUERY_CONTEXT_H

#include <mrmf_core/goal_types.h>
#include <unordered_map>
#include <mrmf_core/robot.h>

namespace mrmf_core
{
struct KinematicsQueryContext
{
    KinematicsQueryContext() = default;

    bio_ik::BioIKKinematicsQueryOptions ik_options;
    
    RobotConstPtr current_robot_;
    //const std::unordered_map<uint64_t, RobotPtr>* robots; 
    //RobotID current_id;
};

} // namespace mrmf_core

#endif // MRMF_KINEMATICS_QUERY_CONTEXT_H