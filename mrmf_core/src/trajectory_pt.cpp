#include <mrmf_core/trajectory_pt.h>

namespace mrmf_core
{
CartesianTrajectoryPt::CartesianTrajectoryPt(const Eigen::Isometry3d& pose)
    : pose_(pose)
{
    initialized_ = true;
}

double CartesianTrajectoryPt::distance(const CartesianTrajectoryPt& other) const
{
    return (pose_.translation() - other.translation()).norm();
}

CartesianTrajectoryPt* CartesianTrajectoryPt::clone() const
{
    return new CartesianTrajectoryPt(*this);
}

CartesianTrajectoryPt* CartesianTrajectoryPt::interpolate(CartesianTrajectoryPt* to, double t) const
{
    auto trans = translation() + t * (to->translation() - translation());
    auto rotation = Eigen::Quaterniond(linear()).slerp(t, Eigen::Quaterniond(to->linear()));
    
    Eigen::Isometry3d interp = Eigen::Translation3d(trans) * rotation;
    return new CartesianTrajectoryPt(interp);
}

std::string CartesianTrajectoryPt::toString() const
{
    std::stringstream ss;
    ss << "CartesianTrajectoryPt \n";
    
    if (isInitialized())
        ss << pose_.matrix();
    else
        ss << "EMPTY";

    return ss.str();
}


const Eigen::Vector3d AxialSymmetricPt::Axis::X = {1.0, 0.0, 0.0};
const Eigen::Vector3d AxialSymmetricPt::Axis::Y = {0.0, 1.0, 0.0};
const Eigen::Vector3d AxialSymmetricPt::Axis::Z = {0.0, 0.0, 1.0};

const double AxialSymmetricPt::Direction::ALIGNED  =  1.0;
const double AxialSymmetricPt::Direction::REVERSED = -1.0;

AxialSymmetricPt::AxialSymmetricPt(const Eigen::Vector3d& position, const Eigen::Vector3d& axis, double direction)
    : axis_(axis), direction_(direction)
{
    pose_.translation() = position; 
    initialized_ = true;  
}

AxialSymmetricPt* AxialSymmetricPt::clone() const
{
    return new AxialSymmetricPt(*this);
}

AxialSymmetricPt* AxialSymmetricPt::interpolate(CartesianTrajectoryPt* to, double t) const
{
    auto trans = translation() + t * (to->translation() - translation());
    return new AxialSymmetricPt(trans, this->axis_, this->direction_);
}

std::string AxialSymmetricPt::toString() const
{
    std::stringstream ss;
    ss << "AxialSymmetricPt \n";

    if (isInitialized())
        ss << pose_.translation();
    else
        ss << "EMPTY";

    return ss.str();
}

} // namespace mrmf_core