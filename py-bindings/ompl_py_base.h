/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Mark Moll */

#ifndef PY_BINDINGS_OMPL_PY_BASE_
#define PY_BINDINGS_OMPL_PY_BASE_

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"
#include "ompl/base/spaces/TimeStateSpace.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#if OMPL_HAVE_NUMPY
#include "ompl/base/spaces/constraint/AtlasStateSpace.h"
#include "ompl/base/spaces/constraint/ProjectedStateSpace.h"
#include "ompl/base/spaces/constraint/TangentBundleStateSpace.h"
#endif
#include "ompl/base/Goal.h"
#include "ompl/base/PlannerData.h"
#include "py_std_function.hpp"

#define DeclareStateType(T) \
    inline int __dummy##T() \
    { \
        return sizeof(ompl::base::ScopedState<T##StateSpace>) + \
        sizeof(ompl::base::T##StateSpace::StateType); \
    }

#define DeclareSpecificParamType(n, T) \
    inline int __dummySP##n() \
    { \
        return sizeof(ompl::base::SpecificParam<T>("dummy", \
            ompl::base::SpecificParam<T>::SetterFn())); \
    }

namespace ompl
{
    namespace base
    {
        DeclareStateType();
        DeclareStateType(Compound);
        DeclareStateType(RealVector);
        DeclareStateType(SO2);
        DeclareStateType(SO3);
        DeclareStateType(SE2);
        DeclareStateType(SE3);
        DeclareStateType(Discrete);
        DeclareStateType(Time);
        DeclareStateType(Dubins);
        DeclareStateType(ReedsShepp);
        DeclareStateType(Wrapper);
#if OMPL_HAVE_NUMPY
        DeclareStateType(Constrained);
        DeclareStateType(Atlas);
        DeclareStateType(Projected);
        DeclareStateType(TangentBundle);
#endif

        DeclareSpecificParamType(0, bool);
        DeclareSpecificParamType(1, char);
        DeclareSpecificParamType(2, int);
        DeclareSpecificParamType(3, unsigned int);
        DeclareSpecificParamType(4, float);
        DeclareSpecificParamType(5, double);
        DeclareSpecificParamType(6, long double);
        DeclareSpecificParamType(7, std::string);

        inline int dummySTLContainerSize()
        {
            return sizeof(std::vector<ompl::base::PlannerSolution>)
                + sizeof(std::map<unsigned int, ompl::base::PlannerDataEdge const *>)
                + sizeof(std::pair<ompl::base::State*, double>);
        }
    }
}

#endif
