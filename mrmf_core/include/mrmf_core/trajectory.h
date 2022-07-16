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
    CartesianTrajectory(const CartesianTrajectory& other, bool deepcopy);

    inline size_t size() const { return waypoints_.size(); }

    std::deque<CartesianTrajectoryPtPtr>::iterator begin() { return waypoints_.begin(); }
    std::deque<CartesianTrajectoryPtPtr>::iterator end() { return waypoints_.end(); }
    std::deque<CartesianTrajectoryPtPtr>::const_iterator cbegin() const { return waypoints_.begin(); }
    std::deque<CartesianTrajectoryPtPtr>::const_iterator cend() const { return waypoints_.end(); }

    inline void setVelocity(double vel) { velocity_ = vel; }
    inline double getVelocity() const { return velocity_; }

    inline void setStartTime(double t) { start_time_ = t; }
    inline double getStartTime() const { return start_time_; }

    inline void setRobot(const RobotID& robot) { robot_ = robot; }
    inline const RobotID& getRobot() const { return robot_; }

    inline void setPositioner(const RobotID& positioner) { positioner_ = positioner; }
    inline const RobotID& getPositioner() const { return positioner_; }

    inline bool isCoordinated() const { return positioner_.is_nil(); }

    inline void addPrefixWayPoint(const CartesianTrajectoryPtPtr& pt) { waypoints_.push_front(pt); }
    inline void addSuffixWayPoint(const CartesianTrajectoryPtPtr& pt) { waypoints_.push_back(pt); }
    void insertWayPoint(size_t index, const CartesianTrajectoryPtPtr& pt);

    CartesianTrajectoryPtPtr getWaypoint(std::size_t index) const;

    // for now assume 0 < i < waypoints_.size()
    double getWaypointDistanceFromPrevious(size_t index) const;
    double getWaypointDurationFromPrevious(size_t index) const;
    double getWaypointDistanceFromStart(size_t index) const;
    double getWaypointDurationFromStart(size_t index) const;

    std::vector<double> getWaypointDistancesFromPrevious() const;
    std::vector<double> getWaypointDistancesFromStart() const;
    std::vector<double> getWaypointDurationsFromPrevious() const;
    std::vector<double> getWaypointDurationsFromStart() const;

    std::string toString() const;

    /* Returns a new trajectory with identical parameter and empty waypoints */
    CartesianTrajectory* emptyCopy() const;
    
private:
    std::deque<CartesianTrajectoryPtPtr> waypoints_;
    double velocity_;
    double start_time_;

    RobotID robot_;
    RobotID positioner_; // positioner id for coordinated movements
};


class CompositeTrajectory
{
public: 
    CompositeTrajectory() = default;

    inline size_t size() const { return trajectories_.size(); } 
    inline CartesianTrajectoryPtr& operator[] (size_t pos) { return trajectories_[pos]; }

    std::vector<CartesianTrajectoryPtr>::iterator begin() { return trajectories_.begin(); }
    std::vector<CartesianTrajectoryPtr>::iterator end() { return trajectories_.end(); }
    std::vector<CartesianTrajectoryPtr>::const_iterator cbegin() const { return trajectories_.begin(); }
    std::vector<CartesianTrajectoryPtr>::const_iterator cend() const { return trajectories_.end(); }

    void addTrajectory(const CartesianTrajectoryPtr x) { trajectories_.push_back(x); }
    void addTrajectory(CartesianTrajectoryPtr&& x) { trajectories_.emplace_back(x); }

private:

    std::vector<CartesianTrajectoryPtr> trajectories_;
};


class SynchronizedTrajectory
{
public:
    SynchronizedTrajectory(const std::vector<CartesianTrajectoryPtr>& trajs);
    
    inline size_t size() const { return keypoints_.size(); }

private:
    void initialize(const std::vector<CartesianTrajectoryPtr>& trajs);

private:
    std::set<double> keypoints_;
    std::vector<CartesianTrajectoryPtr> sync_trajs_;
};

} // namespace mrmf_core

#endif // MRMF_CORE_TRAJECTORY_H