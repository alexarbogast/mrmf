#ifndef MRMF_CONSTRAINT_H
#define MRMF_CONSTRAINT_H

#include <memory>
#include <Eigen/Core>

namespace mrmf_core
{
class KinematicsQueryContext;

class Constraint
{
public:
    Constraint() = default;
    ~Constraint() = default;

    virtual void describe(KinematicsQueryContext& context) const = 0;

protected:
    // std::unique_ptr<bio_ik::Goal> goal_;
};


// make other constraints
// AxialSymmetricConstraint
// PositionConstraint
// OrientationConsraint
// PoseConstraint


class PlanarConstraint : public Constraint
{
public:
    PlanarConstraint(const Eigen::Vector3d& normal);

    virtual void describe(KinematicsQueryContext& context) const override;
private:
    Eigen::Vector3d normal_;
};

typedef std::shared_ptr<Constraint> ConstraintPtr;
typedef std::shared_ptr<const Constraint> ConstraintConstPtr;

} // namespace mrmf_core

#endif // MRMF_CONSTRAINT_H