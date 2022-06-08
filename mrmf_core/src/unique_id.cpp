#include "mrmf_core/unique_id.h"

namespace mrmf_core
{

// RobotID generator specialization
template<>
uint64_t detail::IdGenerator<uint64_t, Robot>::counter_(1);

} // namespace mrmf_core