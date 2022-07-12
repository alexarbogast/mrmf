#ifndef MRMF_CORE_ROBOT_H
#define MRMF_CORE_ROBOT_H

#include <string>
#include <vector>

#include <mrmf_core/constraint.h>
#include <mrmf_core/unique_id.h>

namespace mrmf_core
{
class Robot
{
public:
    enum class RobotType { MANIPULATOR, POSITIONER };

    Robot(const std::string& group, 
          const std::string& tip_frame,
          const std::string& default_pose_frame,
          const Robot::RobotType type = Robot::RobotType::MANIPULATOR);

    const std::string& getGroup() const { return group_; }
    void setGroup(const std::string& group) { group_ = group; }

    const std::string& getTipFrame() const { return tip_frame_; }
    void setTipFrame(const std::string& frame) { tip_frame_ = frame; }

    const std::string& getDefaultPoseFrame() const { return default_pose_frame_; }
    void setDefaultPoseFrame(const std::string& frame) { default_pose_frame_ = frame; }

    inline const Robot::RobotType& getType() const { return type_; }
    inline void setType(const Robot::RobotType& type) { type_ = type; }

    const RobotID& getID() const { return id_; }

    void addPersistentConstraint(const ConstraintPtr& constraint);
    void describePersistentConstraints(KinematicsQueryContext& context) const;

protected:
    std::string group_;
    std::string tip_frame_;
    std::string default_pose_frame_;
    Robot::RobotType type_ = RobotType::MANIPULATOR;

    std::vector<ConstraintPtr> constraints_;
    RobotID id_;
};

typedef std::shared_ptr<Robot> RobotPtr;
typedef std::shared_ptr<const Robot> RobotConstPtr;

} // namespace mrmf_core

#endif // MRMF_CORE_ROBOT_H