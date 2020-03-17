#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_DATASTRUCTURES_GRAPHSAMPLER_SAMPLER_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_DATASTRUCTURES_GRAPHSAMPLER_SAMPLER_
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceGraph.h>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

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

        protected:

          virtual void sampleImplementation(base::State *xRandom) = 0;

          using RNGType = boost::minstd_rand;
          RNGType rng_boost;
          RNG rng_;

          BundleSpaceGraph* bundleSpaceGraph_;

          double epsilonGraphThickening_{0.1};

          double pathBias_{0.2};

          double exponentialDecayLambda_{1e-3};

          unsigned int exponentialDecayCtr_{0};
      };
    }  // namespace geometric
}  // namespace ompl
#endif
