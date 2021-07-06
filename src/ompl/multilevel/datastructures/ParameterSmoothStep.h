/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_PARAMETER_SMOOTH_STEP_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_PARAMETER_SMOOTH_STEP_

#include <ompl/util/Exception.h>
#include <ompl/multilevel/datastructures/Parameter.h>

namespace ompl
{
    /**  \brief ParameterSmoothStep represents a smooth interpolation between two
     * parameter values using a hermite polynomial interpolation. */
    class ParameterSmoothStep : public Parameter
    {
    public:
        ParameterSmoothStep() = default;

        ParameterSmoothStep(double initValue) : Parameter(initValue){};

        ParameterSmoothStep(double initValue, double targetValue) : Parameter(initValue, targetValue){};

        /**  \brief Evaluate interpolation at counter using a third-order hermite
         * polynomial */
        double operator()(void)
        {
            // map to [0,1]
            double t = (getCounter() - getCounterInit()) / (double)(getCounterTarget() - getCounterInit());

            incrementCounter();

            if (t < 0)
                return getValueInit();
            if (t >= 1)
                return getValueTarget();

            return getValueInit() + (getValueTarget() - getValueInit()) * (t * t * (3 - 2 * t));
        }
    };
}
#endif
