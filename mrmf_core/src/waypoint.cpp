#include <mrmf_core/waypoint.h>

namespace mrmf_core
{
CartesianWaypoint::CartesianWaypoint(const Eigen::Isometry3d& pose)
    : pose_(pose)
{
    initialized_ = true;
}

double CartesianWaypoint::distance(const CartesianWaypoint& other) const
{
    return (pose_.translation() - other.translation()).norm();
}

CartesianWaypoint* CartesianWaypoint::clone() const
{
    return new CartesianWaypoint(*this);
}

CartesianWaypoint* CartesianWaypoint::interpolate(CartesianWaypoint* to, double t) const
{
    auto trans = translation() + t * (to->translation() - translation());
    auto rotation = Eigen::Quaterniond(linear()).slerp(t, Eigen::Quaterniond(to->linear()));
    
    Eigen::Isometry3d interp = Eigen::Translation3d(trans) * rotation;
    return new CartesianWaypoint(interp);
}

std::string CartesianWaypoint::toString() const
{
    std::stringstream ss;
    ss << "CartesianWaypoint \n" << pose_.matrix();
    return ss.str();
}


const Eigen::Vector3d AxialSymmetricWaypoint::Axis::X = {1.0, 0.0, 0.0};
const Eigen::Vector3d AxialSymmetricWaypoint::Axis::Y = {0.0, 1.0, 0.0};
const Eigen::Vector3d AxialSymmetricWaypoint::Axis::Z = {0.0, 0.0, 1.0};

const double AxialSymmetricWaypoint::Direction::ALIGNED  =  1.0;
const double AxialSymmetricWaypoint::Direction::REVERSED = -1.0;

AxialSymmetricWaypoint::AxialSymmetricWaypoint(const Eigen::Vector3d& position, const Eigen::Vector3d& axis, double direction)
    : axis_(axis), direction_(direction)
{
    pose_.translation() = position; 
    initialized_ = true;  
}

AxialSymmetricWaypoint* AxialSymmetricWaypoint::clone() const
{
    return new AxialSymmetricWaypoint(*this);
}

AxialSymmetricWaypoint* AxialSymmetricWaypoint::interpolate(CartesianWaypoint* to, double t) const
{
    auto trans = translation() + t * (to->translation() - translation());
    return new AxialSymmetricWaypoint(trans, this->axis_, this->direction_);
}

std::string AxialSymmetricWaypoint::toString() const
{
    std::stringstream ss;
    ss << "AxialSymmetricWaypoint \n" << pose_.translation();
    return ss.str();
}

} // namespace mrmf_core