#ifndef MRMF_CORE_ROBOT_H
#define MRMF_CORE_ROBOT_H

#include <mrmf_core/unique_id.h>
#include <string>

namespace mrmf_core
{
class Robot
{
public:
    enum class RobotType { MANIPULATOR, POSITIONER };

    Robot(const std::string& group, 
          const std::string& tip_frame,
          const Robot::RobotType type = Robot::RobotType::MANIPULATOR)
        : group_(group), tip_frame_(tip_frame), type_(type), id_(RobotID::make_id())
    {
    }

    inline const Robot::RobotType& getType() const { return type_; }
    inline void setType(const Robot::RobotType& type) { type_ = type; }

    const std::string& getTipFrame() const { return tip_frame_; }
    void setTipFrame(const std::string& frame) { tip_frame_ = frame; }

    const RobotID& getID() const { return id_; }

protected:
    std::string group_;
    std::string tip_frame_;
    Robot::RobotType type_ = RobotType::MANIPULATOR;

    RobotID id_;
};

typedef std::shared_ptr<Robot> RobotPtr;
typedef std::shared_ptr<const Robot> RobotConstPtr;

} // namespace mrmf_core

#endif // MRMF_CORE_ROBOT_H