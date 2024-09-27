#ifndef IAUVPLANNER_CLASS_FORWARD_
#define IAUVPLANNER_CLASS_FORWARD_

#include <memory> // for std::shared_ptr
#include <string>

/** \file */

/**
 * \def IAUVPLANNER_CLASS_FORWARD
 * \brief Macro that forward declares a class and defines two shared ptrs types:
 *  - ${Class}Ptr      = shared_ptr<${Class}>
 *  - ${Class}ConstPtr = shared_ptr<const ${Class}>
 */
#define IAUVPLANNER_CLASS_FORWARD(C)   \
    class C;                           \
    typedef std::shared_ptr<C> C##Ptr; \
    typedef std::shared_ptr<const C> C##ConstPtr;
#endif