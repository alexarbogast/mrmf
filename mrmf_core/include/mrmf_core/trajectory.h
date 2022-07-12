#ifndef MRMF_CORE_TRAJECTORY_H
#define MRMF_CORE_TRAJECTORY_H

#include <mrmf_core/trajectory_pt.h>

namespace mrmf_core
{
MRMF_CLASS_FORWARD(CartesianTrajectory)
class CartesianTrajectory
{
public:
    CartesianTrajectory(const RobotID& robot, double velocity, const RobotID& positioner = RobotID::make_nil());

    inline size_t size() const { return waypoints_.size(); }

    void addSuffixWayPoint(const CartesianTrajectoryPtPtr& pt) { waypoints_.push_back(pt); }
    void addSuffixWaypoint(CartesianTrajectoryPtPtr&& pt) { waypoints_.emplace_back(pt); }

    std::string toString() const;
    
    static std::shared_ptr<CartesianTrajectory> makeTrajectory(const RobotID& robot, 
                                                               double velocity,
                                                               const RobotID& positioner = RobotID::make_nil())
    {
        return std::make_shared<CartesianTrajectory>(robot, velocity, positioner);
    }

private:
    std::deque<CartesianTrajectoryPtPtr> waypoints_;
    double velocity_;

    RobotID robot_;
    RobotID positioner_; // positioner id for coordinated movements
};


class CompositeTrajectory
{
public: 
    CompositeTrajectory() = default;

    inline size_t size() const { return trajectories_.size(); } 
    inline CartesianTrajectoryPtr& operator[] (size_t pos) { return trajectories_[pos]; }
    //inline CartesianTrajectoryConstPtr& operator[] (size_t pos) const { return trajectories_[pos]; }

    std::vector<CartesianTrajectoryPtr>::iterator begin() { return trajectories_.begin(); }
    std::vector<CartesianTrajectoryPtr>::iterator end() { return trajectories_.end(); }
    std::vector<CartesianTrajectoryPtr>::const_iterator cbegin() const { return trajectories_.begin(); }
    std::vector<CartesianTrajectoryPtr>::const_iterator cend() const { return trajectories_.end(); }

    void addTrajectory(const CartesianTrajectoryPtr x) { trajectories_.push_back(x); }
    void addTrajectory(CartesianTrajectoryPtr&& x) { trajectories_.emplace_back(x); }

    void synchronizeTrajectories() const;

private:

    std::vector<CartesianTrajectoryPtr> trajectories_;
};

} // namespace mrmf_core

#endif // MRMF_CORE_TRAJECTORY_H