#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_Edge_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_Edge_
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceGraphSamplerRandomEdge : public BundleSpaceGraphSampler
        {
            using BaseT = BundleSpaceGraphSampler;

        public:
            BundleSpaceGraphSamplerRandomEdge() = delete;
            BundleSpaceGraphSamplerRandomEdge(BundleSpaceGraph *);

        protected:
            virtual void sampleImplementation(base::State *xRandom) override;
        };
    }  // namespace multilevel
}  // namespace ompl

#endif
