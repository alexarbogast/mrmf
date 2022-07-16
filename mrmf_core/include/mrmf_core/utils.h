#ifndef MRMF_CORE_UTILS_H
#define MRMF_CORE_UTILS_H

#include <vector>
#include <algorithm>

namespace mrmf_core
{
/*
 * this search returns an insertion index for "data" in sorted vector "v"
 * if "data" already exists in "v", -2 is returned 
 * if "data" does bisect any interval in "v", -1 is returned
*/
template <typename T>
int binary_search_find_index(const std::vector<T>& v, T data)
{
    auto it = std::lower_bound(v.begin(), v.end(), data);
    if (it == v.end())
        return -1;
    else if (*it == data)
        return -2;
    
    std::size_t index = std::distance(v.begin(), it);
    return index;
}

/*
 * this search returns an insertion index for "data" in sorted vector "v"
 * if "data" already exists in "v", the index of the item is returned 
*/
template <typename T>
int binary_search_find_index_inclusive(const std::vector<T>& v, T data)
{
    auto it = std::lower_bound(v.begin(), v.end(), data);
    if (it == v.end())
        return -1;
    
    std::size_t index = std::distance(v.begin(), it);
    return index;
}

} // namespace mrmf_core

#endif // MRMF_CORE_UTILS_H