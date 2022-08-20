#ifndef MRMF_CORE_ROBOT_H
#define MRMF_CORE_ROBOT_H

#include <string>
#include <vector>

#include <moveit/robot_model/joint_model_group.h>
#include <mrmf_core/unique_id.h>

namespace mrmf_core
{
class Robot
{
public:
    enum class RobotType { MANIPULATOR, POSITIONER };

    Robot(const std::string& group, 
          const std::string& tip_frame,
          const std::string& base_frame,
          const moveit::core::JointModelGroup* jmg,
          const Robot::RobotType type = Robot::RobotType::MANIPULATOR);

    const std::string& getGroup() const { return group_; }
    const moveit::core::JointModelGroup* getJointModelGroup() const { return jmg_; }

    const std::string& getTipFrame() const { return tip_frame_; }
    void setTipFrame(const std::string& frame) { tip_frame_ = frame; }

    const std::string& getBaseFrame() const { return base_frame_; }
    void setBaseFrame(const std::string& frame) { base_frame_ = frame; }

    inline const Robot::RobotType& getType() const { return type_; }
    inline void setType(const Robot::RobotType& type) { type_ = type; }

    double getMaxVariableVelocity() const;
    double getMinVariableVelocity() const; 

    const RobotID& getID() const { return id_; }

protected:
    std::string group_;
    std::string tip_frame_;
    std::string base_frame_;
    Robot::RobotType type_ = RobotType::MANIPULATOR;

    const moveit::core::JointModelGroup* jmg_;

    RobotID id_;
};

typedef std::shared_ptr<Robot> RobotPtr;
typedef std::shared_ptr<const Robot> RobotConstPtr;

} // namespace mrmf_core

#endif // MRMF_CORE_ROBOT_H