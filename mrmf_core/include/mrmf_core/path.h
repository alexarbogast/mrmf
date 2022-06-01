#ifndef MRMF_CORE_PATH_H
#define MRMF_CORE_PATH_H

#include <eigen_stl_containers/eigen_stl_containers.h>

namespace mrmf_core
{
struct CartesianPath
{
    EigenSTL::vector_Vector3d path;
    std::string tip_link;
    std::string base_frame;

    inline size_t size() const { return path.size(); } 
    inline Eigen::Vector3d& operator[] (size_t pos) { return path[pos]; }
    inline const Eigen::Vector3d& operator[] (size_t pos) const { return path[pos]; }

    EigenSTL::vector_Vector3d::iterator begin() { return path.begin(); }
    EigenSTL::vector_Vector3d::iterator end() { return path.end(); }
    EigenSTL::vector_Vector3d::const_iterator begin() const { return path.begin(); }
    EigenSTL::vector_Vector3d::const_iterator end() const { return path.end(); }
};

} // namespace mrmf_core

#endif // MRMF_CORE_PATH_H