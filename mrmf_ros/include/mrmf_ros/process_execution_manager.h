#ifndef PROCESS_EXECUTION_MANAGER_H
#define PROCESS_EXECUTION_MANAGER_H

#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

namespace mrmf_ros
{
class ProcessExecutionManager
{
public:
    ProcessExecutionManager() = default;

private:
    trajectory_execution_manager::TrajectoryExecutionManager trajectory_execution_manager_;
    
};

} // namespace mrmf_core

#endif // PROCESS_EXECUTION_MANAGER_H