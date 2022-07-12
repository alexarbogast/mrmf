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

} // namespace mrmf_core