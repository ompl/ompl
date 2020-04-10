#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_DATASTRUCTURES_GRAPHSAMPLER_SAMPLER_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_DATASTRUCTURES_GRAPHSAMPLER_SAMPLER_
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceGraph.h>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>
#include "ExponentialDecay.h"

namespace ompl
{
    namespace geometric
    {

        class BundleSpaceGraphSampler
        {
          protected:
            using Vertex = ompl::geometric::BundleSpaceGraph::Vertex;

          public:

            BundleSpaceGraphSampler() = delete;

            BundleSpaceGraphSampler(BundleSpaceGraph*); 

            void sample(base::State *xRandom);

            void setPathBiasStartSegment(double);
            double getPathBiasStartSegment();

          protected:

            virtual void sampleImplementation(base::State *xRandom) = 0;

            using RNGType = boost::minstd_rand;
            RNGType rng_boost;
            RNG rng_;

            BundleSpaceGraph* bundleSpaceGraph_;

            double epsilonGraphThickening_{0.0};

            double epsilonGraphThickeningFraction_{1e-3};

            double pathBiasFixed_{0.1};

            double pathBiasStartSegment_{0.0};

            double exponentialDecayLambda_{1e-5};

            unsigned long long counterPathSampling_{0};

            unsigned long long counterGraphSampling_{0};

            ExponentialDecay pathBiasDecay_;
            ExponentialDecay graphThickeningDecay_;
            ExponentialDecay pathThickeningDecay_;
        };
    }  // namespace geometric
}  // namespace ompl
#endif
