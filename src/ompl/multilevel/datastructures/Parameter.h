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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_PARAMETER_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_PARAMETER_

#include <ompl/util/Exception.h>

namespace ompl
{
    /**  \brief Parameter represents a smooth interpolation between two
     * parameter values, namely valueInit and valueTarget. The default class
     * keeps a counter to track how often it was called. Starting at counterInit
     * we then count towards counterTarget and smoothly interpolate parameter
     * values inbetween.*/
    class Parameter
    {
    public:
        Parameter() = default;

        Parameter(double valueInit) : valueInit_(valueInit){};

        Parameter(double valueInit, double valueTarget) : valueInit_(valueInit), valueTarget_(valueTarget){};

        /** \brief Set initial value (default: 0.0) */
        void setValueInit(double valueInit)
        {
            valueInit_ = valueInit;
        }
        /** \brief Set target value (default: 1.0) */
        void setValueTarget(double valueTarget)
        {
            valueTarget_ = valueTarget;
        }
        /** \brief Set counter init value (default: 0) */
        void setCounterInit(unsigned long long counterInit)
        {
            counterInit_ = counterInit;
        }
        /** \brief Set counter target value (default: 100) */
        void setCounterTarget(unsigned long long counterTarget)
        {
            counterTarget_ = counterTarget;
        }

        /** \brief Get init value */
        double getValueInit()
        {
            return valueInit_;
        }
        /** \brief Get target value */
        double getValueTarget()
        {
            return valueTarget_;
        }
        /** \brief Get counter init value */
        unsigned long long getCounterInit()
        {
            return counterInit_;
        }
        /** \brief Get counter target value */
        unsigned long long getCounterTarget()
        {
            return counterTarget_;
        }

        /** \brief Call parameter and increase counter */
        double operator()(void)
        {
            if (counter_ > counterTarget_)
                return valueTarget_;

            // map to [0,1]
            double t = (counter_ - counterInit_) / (double)(counterTarget_ - counterInit_);

            double d = valueInit_ + t * (valueTarget_ - valueInit_);
            counter_++;
            return d;
        }

        /** \brief Reset counter to init counter */
        void reset()
        {
            counter_ = counterInit_;
        }

        /** \brief Get current counter */
        unsigned long long getCounter()
        {
            return counter_;
        }
        /** \brief Increment counter */
        void incrementCounter()
        {
            counter_++;
        }

    private:
        /** \brief Init value */
        double valueInit_{0.0};
        /** \brief Target value */
        double valueTarget_{1.0};

        unsigned long long counter_{0};
        unsigned long long counterInit_{0};
        unsigned long long counterTarget_{100};
    };
}
#endif
