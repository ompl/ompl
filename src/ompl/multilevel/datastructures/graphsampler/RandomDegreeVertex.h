#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_DEGREEVERTEX_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_DEGREEVERTEX_
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceGraphSamplerRandomDegreeVertex : public BundleSpaceGraphSampler
        {
            using BaseT = BundleSpaceGraphSampler;

        public:
            BundleSpaceGraphSamplerRandomDegreeVertex() = delete;
            BundleSpaceGraphSamplerRandomDegreeVertex(BundleSpaceGraph *);

        protected:
            virtual void sampleImplementation(base::State *xRandom) override;
        };
    }  // namespace multilevel
}  // namespace ompl

#endif
