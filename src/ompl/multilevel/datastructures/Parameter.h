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
    class Parameter
    {
    public:
        Parameter() = default;

        Parameter(double valueInit) : valueInit_(valueInit){};

        Parameter(double valueInit, double valueTarget) : valueInit_(valueInit), valueTarget_(valueTarget){};

        void setValueInit(double valueInit)
        {
            valueInit_ = valueInit;
        }
        void setValueTarget(double valueTarget)
        {
            valueTarget_ = valueTarget;
        }
        void setCounterTarget(unsigned long long counterTarget)
        {
            counterTarget_ = counterTarget;
        }
        void setCounterInit(unsigned long long counterInit)
        {
            counterInit_ = counterInit;
        }

        double getValueInit()
        {
            return valueInit_;
        }
        double getValueTarget()
        {
            return valueTarget_;
        }
        unsigned long long getCounterTarget()
        {
            return counterTarget_;
        }
        unsigned long long getCounterInit()
        {
            return counterInit_;
        }

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

        void reset()
        {
            counter_ = 0;
        }

        unsigned long long getCounter()
        {
            return counter_;
        }
        void incrementCounter()
        {
            counter_++;
        }

    private:
        double valueInit_{0.0};
        double valueTarget_{1.0};

        unsigned long long counter_{0};

        unsigned long long counterInit_{0};
        unsigned long long counterTarget_{100};
    };
}
#endif
