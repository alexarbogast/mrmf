#ifndef MRMF_UTILS_H
#define MRMF_UTILS_H

#include <memory>

#define MRMF_DECLARE_PTR(Name, Type)                                                                        \
    typedef std::shared_ptr<Type> Name##Ptr;                                                                \
    typedef std::shared_ptr<const Type> Name##ConstPtr;                                                     \
    typedef std::unique_ptr<Type> Name##UniquePtr;                                                          \
    typedef std::unique_ptr<const Type> Name##ConstUniquePtr;

#define MRMF_CLASS_FORWARD(C)                                                                               \
    class C;                                                                                                \
    MRMF_DECLARE_PTR(C, C)                                                                                  \

#endif // MRMF_UTILS_H