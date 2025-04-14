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


namespace nb = nanobind;
namespace ob = ompl::base;


inline ob::ScopedState<> state2ScopedState(ob::StateSpacePtr space, const ob::State* s)
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
        default:
            throw std::runtime_error("Unsupported or unhandled state space type.");
    }
}