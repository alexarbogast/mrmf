#ifndef MRMF_GOAL_TYPES_H
#define MRMF_GOAL_TYPES_H

#include "bio_ik/bio_ik.h"

namespace bio_ik
{
/**
 * @brief Constrain a frames position with respect to a second frame
 * 
 * @param position position of link "frame" in link "base_frame"
 * @param frame name of link to position
 * @param base_frame name of link used as reference frame
 */
class RelativePositionGoal : public LinkGoalBase
{
    tf2::Vector3 position_;
    std::string base_frame_;

public:
    RelativePositionGoal()
        : position_(0, 0, 0)
    { 
    }

    RelativePositionGoal(const std::string& link_name, const tf2::Vector3& position, const std::string& frame, const std::string& base_frame, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position_(position)
        , base_frame_(base_frame)
    {
    }

    inline const tf2::Vector3& getPosition() const { return position_; }
    inline void setPosition(const tf2::Vector3& position) { position_ = position; }
    inline const std::string& getBaseFrame() const { return base_frame_; }
    inline void setBaseFrame(const std::string& frame) { base_frame_ = frame; }

    virtual void describe(GoalContext& context) const override
    {
        Goal::describe(context);
        
        context.addLink(getLinkName());
        context.addLink(base_frame_);
    }

    virtual double evaluate(const GoalContext& context) const override
    {
        Frame base_frame = context.getLinkFrame(1);
        Frame target(getPosition(), tf2::Quaternion());

        tf2::Vector3 goal_position = (base_frame * target).getPosition();

        return context.getLinkFrame().getPosition().distance2(goal_position);
    } 
};

/**
 * @brief Minimize the distance between a frame origin and a plane
 * @param normal_ normal vector defining plane through origin 
 */
class MinimizeDistanceGoal : public LinkGoalBase
{
    tf2::Vector3 normal_;

public:
    MinimizeDistanceGoal()
        : normal_(0, 0, 0)
    {
    }

    MinimizeDistanceGoal(const std::string& link_name, const tf2::Vector3& normal, double weight = 1.0, bool secondary = true)
        : LinkGoalBase(link_name, weight)
    {
        this->normal_ = normal.normalized();
        this->secondary_ = secondary;
    }

    const tf2::Vector3& getNormal() const { return normal_; }
    void setNormal(const tf2::Vector3& a) { normal_ = a.normalized(); }

    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        //double d = fb.getPosition() * normal_;
        double d = fb.getPosition().dot(normal_);
        return d * d;
    }
};

} // namespace bio_ik

#endif // MRMF_GOAL_TYPES_H