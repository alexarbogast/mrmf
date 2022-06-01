#ifndef MRMF_CORE_ROBOT_H
#define MRMF_CORE_ROBOT_H

#include <string>

namespace mrmf_core
{
class Robot
{
public:
    enum class RobotType { MANIPULATOR, POSITIONER };

    Robot() = default;

    Robot(const std::string& group)
        : group_(group_)
    {
    }

    inline const Robot::RobotType& getType() const { return type_; }
    inline void setType(const Robot::RobotType& type) { type_ = type; }

protected:
    Robot::RobotType type_ = RobotType::MANIPULATOR;
    std::string group_;
    std::string tip_link_;
};

class Manipulator : public Robot
{
public:
    Manipulator(const std::string& group)
        : Robot(group)
    {
    }

private:
    std::string tip_link_;
};

class Positioner : public Robot
{

};

} // namespace mrmf_core

#endif // MRMF_CORE_ROBOT_H