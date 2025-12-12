#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <iostream>

#include "ompl/base/ScopedState.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/State.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/base/spaces/SO3StateSpace.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"
#include "ompl/base/spaces/SpaceTimeStateSpace.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"

namespace nb = nanobind;
namespace ob = ompl::base;

inline ob::ScopedState<> state2ScopedState(ob::StateSpacePtr space, const ob::State *s)
{
    auto stateType = space->getType();
    switch (stateType)
    {
        case ob::STATE_SPACE_REAL_VECTOR:
        {
            return ob::ScopedState<ob::RealVectorStateSpace>(space, s);
        }
        case ob::STATE_SPACE_SO2:
        {
            return ob::ScopedState<ob::SO2StateSpace>(space, s);
        }
        case ob::STATE_SPACE_SO3:
        {
            return ob::ScopedState<ob::SO3StateSpace>(space, s);
        }
        case ob::STATE_SPACE_SE2:
        {
            return ob::ScopedState<ob::SE2StateSpace>(space, s);
        }
        case ob::STATE_SPACE_SE3:
        {
            return ob::ScopedState<ob::SE3StateSpace>(space, s);
        }
        case ob::STATE_SPACE_DISCRETE:
        {
            return ob::ScopedState<ob::DiscreteStateSpace>(space, s);
        }
        case ob::STATE_SPACE_TIME:
        {
            return ob::ScopedState<ob::TimeStateSpace>(space, s);
        }
       
        default:

            throw std::runtime_error("Unsupported or unhandled state space type: " + std::to_string(stateType));
    }


    // if (auto o = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(space))
    //     return ob::ScopedState<ob::RealVectorStateSpace>(o, s);
    // if (auto o = std::dynamic_pointer_cast<ob::SO2StateSpace>(space))
    //     return ob::ScopedState<ob::SO2StateSpace>(o, s);
    // if (auto o = std::dynamic_pointer_cast<ob::SO3StateSpace>(space))
    //     return ob::ScopedState<ob::SO3StateSpace>(o, s);
    // if (auto o = std::dynamic_pointer_cast<ob::SE2StateSpace>(space))
    //     return ob::ScopedState<ob::SE2StateSpace>(o, s);
    // if (auto o = std::dynamic_pointer_cast<ob::SE3StateSpace>(space))
    //     return ob::ScopedState<ob::SE3StateSpace>(o, s);
    // if (auto o = std::dynamic_pointer_cast<ob::DiscreteStateSpace>(space))
    //     return ob::ScopedState<ob::DiscreteStateSpace>(o, s);
    // if (auto o = std::dynamic_pointer_cast<ob::TimeStateSpace>(space))
    //     return ob::ScopedState<ob::TimeStateSpace>(o, s);
    // if (auto o = std::dynamic_pointer_cast<ob::SpaceTimeStateSpace>(space))
    //     return ob::ScopedState<ob::SpaceTimeStateSpace>(o, s);  
    // if (auto o = std::dynamic_pointer_cast<ob::ConstrainedStateSpace>(space))
    //     return ob::ScopedState<ob::ConstrainedStateSpace>(o, s);
    // throw std::runtime_error("Unsupported or unhandled state space type: " + std::to_string(space->getType()));

}