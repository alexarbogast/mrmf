#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <string>

namespace mrmf_core
{
struct ManipulatorConfig
{
    std::string group_name;
    std::string world_frame;
    std::string base_link;
    std::string tip_link;
};

class Manipulator
{
public:
    Manipulator(const ManipulatorConfig& config);
private:
    ManipulatorConfig config_;
};

} // namespace mrmf_core

#endif // MANIPULATOR_H