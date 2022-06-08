#ifndef MRMF_CORE_TRAJECTORY_H

#include <mrmf_core/trajectory_pt.h>
#include <mrmf_core/unique_id.h>
#include <vector>
#include <memory>
#include <string>
#include <algorithm>

namespace mrmf_core
{
struct RobotTrajectory
{
    RobotID id;
    std::vector<TrajectoryPtPtr> points;
    
    RobotTrajectory() = default;

    RobotTrajectory(const RobotID& rid)
    : id(rid)
    {}

    inline size_t size() const { return points.size(); }

    void addTrajectoryPt(const TrajectoryPtPtr& pt) { points.push_back(pt); }
    void addTrajectoryPt(TrajectoryPtPtr&& pt) { points.emplace_back(pt); }

    static std::shared_ptr<RobotTrajectory> makeTrajectory(const RobotID& id)
    {
        return std::make_shared<RobotTrajectory>(id);
    }
};
typedef std::shared_ptr<RobotTrajectory> RobotTrajectoryPtr;
typedef const std::shared_ptr<RobotTrajectory> RobotTrajectoryConstPtr;


class CompositeTrajectory
{
public: 
    CompositeTrajectory() = default;

    inline size_t size() const { return path_.size(); } 
    inline RobotTrajectoryPtr& operator[] (size_t pos) { return path_[pos]; }
    inline RobotTrajectoryConstPtr& operator[] (size_t pos) const { return path_[pos]; }

    std::vector<RobotTrajectoryPtr>::iterator begin() { return path_.begin(); }
    std::vector<RobotTrajectoryPtr>::iterator end() { return path_.end(); }
    std::vector<RobotTrajectoryPtr>::const_iterator cbegin() const { return path_.begin(); }
    std::vector<RobotTrajectoryPtr>::const_iterator cend() const { return path_.end(); }

    void addTrajectory(const RobotTrajectoryPtr x) { path_.push_back(x); }
    void addTrajectory(RobotTrajectoryPtr&& x) { path_.emplace_back(x); }

    bool equalSizes() const
    {
        return std::all_of(cbegin(), cend(),
            [this] (const RobotTrajectoryPtr& x) { return x->size() == path_.front()->size(); });
    }

    size_t getLongestDimension() const
    {
        auto it = std::max_element(cbegin(), cend(), 
            [] (const RobotTrajectoryPtr& a, const RobotTrajectoryPtr& b) { return a->size() < b->size(); });
        return (*it)->size();
    }

    size_t getShortestDimension() const
    {
        auto it = std::min_element(cbegin(), cend(), 
            [] (const RobotTrajectoryPtr& a, const RobotTrajectoryPtr& b) { return a->size() < b->size(); });
        return (*it)->size();
    }

private:

    std::vector<RobotTrajectoryPtr> path_;
};

} // namespace mrmf_core

#endif // MRMF_CORE_TRAJECTORY_H