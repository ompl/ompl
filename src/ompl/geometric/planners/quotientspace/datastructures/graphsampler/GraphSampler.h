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

          virtual void sample(base::State *xRandom) = 0;

        protected:

          using RNGType = boost::minstd_rand;
          RNGType rng_boost;
          RNG rng_;

          BundleSpaceGraph* bundleSpaceGraph_;
      };
    }  // namespace geometric
}  // namespace ompl

#endif

