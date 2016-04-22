#ifndef PY_BINDINGS_OMPL_PY_MORSE_
#define PY_BINDINGS_OMPL_PY_MORSE_

#include "ompl/base/ScopedState.h"
#include "ompl/extensions/morse/MorseStateSpace.h"
#include "py_std_function.hpp"

#define DeclareStateType(T) \
    inline int __dummy##T() \
    { \
        return sizeof(ompl::base::ScopedState<T##StateSpace>) + \
        sizeof(ompl::base::T##StateSpace::StateType); \
    }

namespace ompl
{
    namespace base
    {
        DeclareStateType(Morse);
    }
}

#endif
