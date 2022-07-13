#ifndef MRMF_TRAJECTORY_POINT_H
#define MRMF_TRAJECTORY_POINT_H

#include <mrmf_core/macros.h>
#include <mrmf_core/kinematics_query_context.h>
#include <Eigen/Core>

namespace mrmf_core
{
MRMF_CLASS_FORWARD(TrajectoryPt)
class TrajectoryPt
{
public:
    TrajectoryPt() = default;
    virtual ~TrajectoryPt() = default;

    // describe a trajectory point as a combination of kinematic constraints
    virtual void describe(KinematicsQueryContext& context) const = 0;
    virtual std::string toString() const = 0;
    
    inline bool isInitialized() const { return initialized_; }

protected:
    bool initialized_ = false;
    std::string frame_;
};


MRMF_CLASS_FORWARD(CartesianTrajectoryPt)
class CartesianTrajectoryPt : public TrajectoryPt
{
public:
    CartesianTrajectoryPt() = default;
    CartesianTrajectoryPt(const Eigen::Isometry3d& pose);

    virtual ~CartesianTrajectoryPt() = default;

    virtual void describe(KinematicsQueryContext& context) const override
    {
        //auto* goal1 = new bio_ik::RelativePositionGoal();
        //goal1->setBaseFrame(frame_);
        //goal1->setLinkName(context.current_robot->getTipFrame());
        //goal1->setPosition(tf2::Vector3(position_.x(), position_.y(), position_.z()));

        //context.ik_options.goals.emplace_back(goal1);
    }

    virtual std::string toString() const override
    {
        std::stringstream ss;
        ss << "CartesianTrajectoryPt \n" << pose_.matrix();
        return ss.str();
    }

    inline const auto translation() const { return pose_.translation(); }
    inline const auto linear() const { return pose_.linear(); }

    double distance(const CartesianTrajectoryPt& other) const;

protected:
    Eigen::Isometry3d pose_;
};


class AxialSymmetricPt : public CartesianTrajectoryPt
{
public:
    struct Axis
    {
        static const Eigen::Vector3d X;
        static const Eigen::Vector3d Y;
        static const Eigen::Vector3d Z;
    };

    struct Direction
    {
        static const double ALIGNED;
        static const double REVERSED;
    }; 

    AxialSymmetricPt() = default;
    AxialSymmetricPt(const Eigen::Vector3d& position, const Eigen::Vector3d& axis, double direction);

    virtual ~AxialSymmetricPt() = default;

    virtual void describe(KinematicsQueryContext& context) const override
    {
        //auto* position_goal = new bio_ik::RelativePositionGoal();
        //position_goal->setBaseFrame(frame_);
        //position_goal->setLinkName(context.current_robot->getTipFrame());
        //position_goal->setPosition(tf2::Vector3(position_.x(), position_.y(), position_.z()));
        
        //auto* direction_goal = new bio_ik::DirectionGoal();
        //direction_goal->setLinkName(context.current_robot->getTipFrame());
        //direction_goal->setAxis(tf2::Vector3(axis_.x(), axis_.y(), axis_.z()));
        //direction_goal->setDirection(tf2::Vector3(direction_.x(), direction_.y(), direction_.z()));

        //context.ik_options.goals.emplace_back(position_goal);
        //context.ik_options.goals.emplace_back(direction_goal);
    }

    virtual std::string toString() const override
    {
        std::stringstream ss;
        ss << "AxialSymmetricPt \n" << pose_.translation();
        return ss.str();
    }

private:
    Eigen::Vector3d axis_;
    double direction_;
};

} // namespace mrmf_core

#endif // MRMF_TRAJECTORY_POINT_H