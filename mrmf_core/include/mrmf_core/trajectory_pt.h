#ifndef MRMF_TRAJECTORY_POINT_H
#define MRMF_TRAJECTORY_POINT_H

#include <mrmf_core/kinematics_query_context.h>
#include <Eigen/Core>

namespace mrmf_core
{
class TrajectoryPt
{
public:
    TrajectoryPt() = default;
    virtual ~TrajectoryPt() = default;

    virtual void describe(KinematicsQueryContext& context) const = 0;

private:
};
typedef std::shared_ptr<TrajectoryPt> TrajectoryPtPtr;
typedef std::unique_ptr<TrajectoryPt> TrajectoryPtUniquePtr;


class CartesianTrajectoryPt : public TrajectoryPt
{
public:
    CartesianTrajectoryPt()
        : position_(0, 0, 0)
    {
    }

    CartesianTrajectoryPt(const Eigen::Vector3d& position, const std::string& frame = "")
        : position_(position), frame_(frame)
    {
    }

    virtual ~CartesianTrajectoryPt() = default;

    virtual void describe(KinematicsQueryContext& context) const override
    {
        auto* goal1 = new bio_ik::RelativePositionGoal();
        goal1->setBaseFrame(frame_);
        goal1->setLinkName(context.current_robot_->getTipFrame());
        goal1->setPosition(tf2::Vector3(position_.x(), position_.y(), position_.z()));

        context.ik_options.goals.emplace_back(goal1);
    }

private:
    Eigen::Vector3d position_;
    std::string frame_;
};


class AxialSymmetricPt : public TrajectoryPt
{
public:

    enum class Axis { X, Y, Z };

    AxialSymmetricPt()
    : position_(0, 0, 0), axis_(0, 0, 0)
    {
    }

    AxialSymmetricPt(const Eigen::Vector3d& position,
                     const Axis& axis,
                     const Eigen::Vector3d& direction,
                     const std::string& frame = "")
        : position_(position), direction_(direction), frame_(frame)
    {
        Eigen::Vector3d ax(0, 0, 0);

        switch (axis)
        {
            case Axis::X:
                ax = {1, 0, 0};
                break;
            case Axis::Y:
                ax = {0, 1, 0};
                break;
            case Axis::Z:
                ax = {0, 0, 1};
                break;    
            default:
                break;
        }

        axis_ = ax;
    }

    AxialSymmetricPt(const Eigen::Vector3d& position,
                     const Eigen::Vector3d& axis,
                     const Eigen::Vector3d& direction,
                     const std::string& frame = "")
    : position_(position), axis_(axis), direction_(direction), frame_(frame)
    {
    }

    virtual ~AxialSymmetricPt() = default;

    virtual void describe(KinematicsQueryContext& context) const override
    {
        auto* position_goal = new bio_ik::RelativePositionGoal();
        position_goal->setBaseFrame(frame_);
        position_goal->setLinkName(context.current_robot_->getTipFrame());
        position_goal->setPosition(tf2::Vector3(position_.x(), position_.y(), position_.z()));
        
        auto* direction_goal = new bio_ik::DirectionGoal();
        direction_goal->setLinkName(context.current_robot_->getTipFrame());
        direction_goal->setAxis(tf2::Vector3(axis_.x(), axis_.y(), axis_.z()));
        direction_goal->setDirection(tf2::Vector3(direction_.x(), direction_.y(), direction_.z()));

        context.ik_options.goals.emplace_back(position_goal);
        context.ik_options.goals.emplace_back(direction_goal);
    }

private:
    Eigen::Vector3d position_;
    Eigen::Vector3d axis_;
    Eigen::Vector3d direction_;

    std::string frame_;
};

} // namespace mrmf_core

#endif // MRMF_TRAJECTORY_POINT_H