#ifndef MRMF_CORE_CART_TRAJECTORY_H
#define MRMF_CORE_CART_TRAJECTORY_H

#include <mrmf_core/waypoint.h>

namespace mrmf_core
{
MRMF_CLASS_FORWARD(CartesianTrajectory)
class CartesianTrajectory
{
public:
    CartesianTrajectory(const RobotID& robot, double velocity, const RobotID& positioner = RobotID::make_nil());
    CartesianTrajectory(const CartesianTrajectory& other, bool deepcopy);

    /* Returns a new trajectory with identical parameter and empty waypoints */
    CartesianTrajectory* emptyCopy() const;

    inline size_t size() const { return waypoints_.size(); }

    std::deque<CartesianWaypointPtr>::iterator begin() { return waypoints_.begin(); }
    std::deque<CartesianWaypointPtr>::iterator end() { return waypoints_.end(); }
    std::deque<CartesianWaypointPtr>::const_iterator cbegin() const { return waypoints_.begin(); }
    std::deque<CartesianWaypointPtr>::const_iterator cend() const { return waypoints_.end(); }

    inline void setVelocity(double vel) { velocity_ = vel; }
    inline double getVelocity() const { return velocity_; }

    inline void setStartTime(double t) { start_time_ = t; }
    inline double getStartTime() const { return start_time_; }

    inline void setRobot(const RobotID& robot) { robot_ = robot; }
    inline const RobotID& getRobot() const { return robot_; }

    inline void setPositioner(const RobotID& positioner) { positioner_ = positioner; }
    inline const RobotID& getPositioner() const { return positioner_; }

    inline bool isCoordinated() const { return positioner_.is_nil(); }

    inline void addPrefixWayPoint(const CartesianWaypointPtr& pt) { waypoints_.push_front(pt); }
    inline void addSuffixWayPoint(const CartesianWaypointPtr& pt) { waypoints_.push_back(pt); }
    void insertWayPoint(size_t index, const CartesianWaypointPtr& pt);

    CartesianWaypointPtr getWaypoint(std::size_t index) const;

    double getWaypointDistanceFromPrevious(size_t index) const;
    double getWaypointDurationFromPrevious(size_t index) const;
    double getWaypointDistanceFromStart(size_t index) const;
    double getWaypointDurationFromStart(size_t index) const;

    std::vector<double> getWaypointDistancesFromPrevious() const;
    std::vector<double> getWaypointDistancesFromStart() const;
    std::vector<double> getWaypointDurationsFromPrevious() const;
    std::vector<double> getWaypointDurationsFromStart() const;

    std::string toString() const;
    
private:
    std::deque<CartesianWaypointPtr> waypoints_;
    double velocity_;
    double start_time_;

    RobotID robot_;
    RobotID positioner_; // positioner id for coordinated movements
};

} // namespace mrmf_core

#endif // MRMF_CORE_CART_TRAJECTORY_H