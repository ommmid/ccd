
/* Author: Omid Heidari */

#ifndef CCD_UTIL_CLASS_FORWARD_  // this name could be anything
#define CCD_UTIL_CLASS_FORWARD_
// ifndef: IF Not DEFined
// explanation: if FABRIK_UTIL_CLASS_FORWARD_ is not defined, go ahead and define one
// whatever is between #define and #endif is going to be consider as FABRIK_UTIL_CLASS_FORWARD_
// This stops this header file to be compiled multiple times.

#include <memory>

/** \brief Macro that defines a forward declaration for a class, and
 *  shared pointers to the class. 
 */
#define CCD_CLASS_FORWARD(C)                                                                                        \
  class C;                                                                                                             \
  CCD_DECLARE_PTR(C, C);

#define CCD_DECLARE_PTR(Name, Type)                                                                                 \
  typedef std::shared_ptr<Type> Name##Ptr;                                                                             \
  typedef std::shared_ptr<const Type> Name##ConstPtr;                                                                  \
  typedef std::weak_ptr<Type> Name##WeakPtr;                                                                           \
  typedef std::weak_ptr<const Type> Name##ConstWeakPtr;                                                                \
  typedef std::unique_ptr<Type> Name##UniquePtr;                                                                       \
  typedef std::unique_ptr<const Type> Name##ConstUniquePtr;

#endif // CCD_UTIL_CLASS_FORWARD_