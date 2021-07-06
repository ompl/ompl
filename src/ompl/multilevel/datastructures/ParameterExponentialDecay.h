/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_EXPONENTIAL_DECAY__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_EXPONENTIAL_DECAY__

#include <ompl/util/Exception.h>
#include <ompl/multilevel/datastructures/Parameter.h>

namespace ompl
{
    /**  \brief ParameterExponentialDecay represents a smooth interpolation between two
     * parameter values using an exponential decay as interpolation. This decay
     * depends on a paramter lambda, which can be tuned to either converge
     * slow or fast to valueTarget. */
    class ParameterExponentialDecay : public Parameter
    {
    public:
        ParameterExponentialDecay() = default;

        ParameterExponentialDecay(double lambda) : Parameter(), lambda_(lambda)
        {
            setLambda(lambda);
        };

        ParameterExponentialDecay(double lambda, double valueInit) : Parameter(valueInit), lambda_(lambda)
        {
            setLambda(lambda);
        };

        ParameterExponentialDecay(double lambda, double valueInit, double valueTarget)
          : Parameter(valueInit, valueTarget), lambda_(lambda)
        {
            setLambda(lambda);
        };

        /** \brief Set lambda decay parameter (default: 0.1) */
        void setLambda(double lambda)
        {
            if (lambda < 0)
            {
                throw ompl::Exception("ExponentialDecay requires non-negative lambda");
            }
            else
            {
                lambda_ = lambda;
            }
        }

        /** \brief Evaluate exponential decay at counter */
        double operator()(void)
        {
            double value = (getValueInit() - getValueTarget()) * exp(-lambda_ * getCounter()) + getValueTarget();
            incrementCounter();
            return value;
        }

    private:
        /** \brief Decay parameter */
        double lambda_{0.1};
    };
}
#endif
