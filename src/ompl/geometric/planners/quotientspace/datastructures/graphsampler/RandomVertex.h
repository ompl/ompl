#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_VERTEX_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_VERTEX_
#include <ompl/geometric/planners/quotientspace/datastructures/graphsampler/GraphSampler.h>

namespace ompl
{
    namespace geometric
    {
      class BundleSpaceGraphSamplerRandomVertex: public BundleSpaceGraphSampler
      {
          using BaseT = BundleSpaceGraphSampler;
        public:
          BundleSpaceGraphSamplerRandomVertex() = delete;
          BundleSpaceGraphSamplerRandomVertex(BundleSpaceGraph*); 

          virtual void sample(base::State *xRandom) override;
      };
    }  // namespace geometric
}  // namespace ompl

#endif


