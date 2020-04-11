#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_DEGREEVERTEX_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_DEGREEVERTEX_
#include <ompl/geometric/planners/multilevel/datastructures/graphsampler/GraphSampler.h>

namespace ompl
{
    namespace geometric
    {
      class BundleSpaceGraphSamplerRandomDegreeVertex: public BundleSpaceGraphSampler
      {
          using BaseT = BundleSpaceGraphSampler;
        public:
          BundleSpaceGraphSamplerRandomDegreeVertex() = delete;
          BundleSpaceGraphSamplerRandomDegreeVertex(BundleSpaceGraph*); 

        protected:
          virtual void sampleImplementation(base::State *xRandom) override;
      };
    }  // namespace geometric
}  // namespace ompl

#endif


