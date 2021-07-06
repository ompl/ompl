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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_GRAPHSAMPLER_SAMPLER_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_GRAPHSAMPLER_SAMPLER_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/datastructures/ParameterExponentialDecay.h>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceGraphSampler
        {
        protected:
            using Vertex = ompl::multilevel::BundleSpaceGraph::Vertex;

        public:
            BundleSpaceGraphSampler() = delete;

            BundleSpaceGraphSampler(BundleSpaceGraph *);

            virtual void sample(base::State *xRandom);

            void setPathBiasStartSegment(double);

            double getPathBiasStartSegment();

            void disableSegmentBias();

            void disablePathBias();

            virtual void clear();

        protected:
            virtual void sampleImplementation(base::State *xRandom) = 0;

            using RNGType = boost::minstd_rand;
            RNGType rng_boost;
            RNG rng_;

            BundleSpaceGraph *bundleSpaceGraph_;

            double epsilonGraphThickening_{0.0};

            double epsilonGraphThickeningFraction_{1e-3};

            double pathBiasFixed_{0.1};

            double pathBiasStartSegment_{0.0};

            bool segmentBias_{true};

            double exponentialDecayLambda_{1e-5};

            unsigned long long counterPathSampling_{0};

            unsigned long long counterGraphSampling_{0};

            ParameterExponentialDecay pathBiasDecay_;
            ParameterExponentialDecay graphThickeningGrowth_;
            ParameterExponentialDecay pathThickeningGrowth_;
        };
    }  // namespace multilevel
}  // namespace ompl
#endif
