#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_Edge_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_Edge_
#include <ompl/geometric/planners/quotientspace/datastructures/graphsampler/GraphSampler.h>

namespace ompl
{
    namespace geometric
    {
      class BundleSpaceGraphSamplerRandomEdge: public BundleSpaceGraphSampler
      {
          using BaseT = BundleSpaceGraphSampler;
        public:
          BundleSpaceGraphSamplerRandomEdge() = delete;
          BundleSpaceGraphSamplerRandomEdge(BundleSpaceGraph*); 

        protected:
          virtual void sampleImplementation(base::State *xRandom) override;
      };
    }  // namespace geometric
}  // namespace ompl

#endif



