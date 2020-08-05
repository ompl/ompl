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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_GRAPHSAMPLER_EXPONENTIAL_DECAY__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_GRAPHSAMPLER_EXPONENTIAL_DECAY__

class ExponentialDecay
{
public:
    ExponentialDecay() = default;
    ExponentialDecay(double lambda) : lambda_(lambda){};
    ExponentialDecay(double lambda, double lowerBound) : lambda_(lambda), lowerBound_(lowerBound){};
    ExponentialDecay(double lambda, double lowerBound, double upperBound)
      : lambda_(lambda), lowerBound_(lowerBound), upperBound_(upperBound){};

    void setLowerBound(double lowerBound)
    {
        lowerBound_ = lowerBound;
    }
    void setUpperBound(double upperBound)
    {
        upperBound_ = upperBound;
    }
    void setLambda(double lambda)
    {
        lambda_ = lambda;
    }

    double operator()(void)
    {
        return (upperBound_ - lowerBound_) * exp(-lambda_ * counter_++) + lowerBound_;
    }

    void reset()
    {
        counter_ = 0;
    }

private:
    double lambda_{0.1};
    double lowerBound_{0.0};
    double upperBound_{1.0};
    unsigned long long counter_{0};
};
#endif
