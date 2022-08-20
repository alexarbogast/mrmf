#ifndef MRMF_CORE_WAYPOINT_H
#define MRMF_CORE_WAYPOINT_H

#include <mrmf_core/macros.h>
#include <Eigen/Geometry>

namespace mrmf_core
{
MRMF_CLASS_FORWARD(Waypoint)
class Waypoint
{
public:
    Waypoint() = default;
    virtual ~Waypoint() = default;

    virtual std::string toString() const = 0;

    virtual Waypoint* clone() const = 0;
    
    inline bool isInitialized() const { return initialized_; }

protected:
    bool initialized_ = false;
};


MRMF_CLASS_FORWARD(CartesianWaypoint)
class CartesianWaypoint : public Waypoint
{
public:
    CartesianWaypoint() = default;
    CartesianWaypoint(const Eigen::Isometry3d& pose);

    virtual ~CartesianWaypoint() = default;

    virtual CartesianWaypoint* clone() const override;
    virtual CartesianWaypoint* interpolate(CartesianWaypoint* to, double t) const;

    virtual std::string toString() const override;

    inline const auto translation() const { return pose_.translation(); }
    inline const auto linear() const { return pose_.linear(); }
    inline auto translation() { return pose_.translation(); }
    inline auto linear() { return pose_.linear(); }

    virtual Eigen::Isometry3d& getPose() { return pose_; }
    virtual const Eigen::Isometry3d& getPose() const { return pose_; }

    double distance(const CartesianWaypoint& other) const;

protected:
    Eigen::Isometry3d pose_;
};


class AxialSymmetricWaypoint : public CartesianWaypoint
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

    AxialSymmetricWaypoint() = default;
    AxialSymmetricWaypoint(const Eigen::Vector3d& position, const Eigen::Vector3d& axis, double direction);

    virtual ~AxialSymmetricWaypoint() = default;

    virtual AxialSymmetricWaypoint* clone() const override;
    virtual AxialSymmetricWaypoint* interpolate(CartesianWaypoint* to, double t) const override;
    
    virtual std::string toString() const override;

private:
    Eigen::Vector3d axis_;
    double direction_;
};

} // namespace mrmf_core

#endif // MRMF_CORE_WAYPOINT_H