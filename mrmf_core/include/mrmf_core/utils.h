#ifndef MRMF_CORE_UTILS_H
#define MRMF_CORE_UTILS_H

#include <vector>
#include <algorithm>

namespace mrmf_core
{
template <typename T>
int binary_search_find_index(const std::vector<T>& v, T data)
{
    auto it = std::lower_bound(v.begin(), v.end(), data);
    if (it == v.end())
        return -1;
    
    std::size_t index = std::distance(v.begin(), it);
    return index;
}

} // namespace mrmf_core

#endif // MRMF_CORE_UTILS_H