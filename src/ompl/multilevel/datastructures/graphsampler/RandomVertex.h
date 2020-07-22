#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_VERTEX_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_VERTEX_
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceGraphSamplerRandomVertex : public BundleSpaceGraphSampler
        {
            using BaseT = BundleSpaceGraphSampler;

        public:
            BundleSpaceGraphSamplerRandomVertex() = delete;
            BundleSpaceGraphSamplerRandomVertex(BundleSpaceGraph *);

        protected:
            virtual void sampleImplementation(base::State *xRandom) override;
        };
    }  // namespace multilevel
}  // namespace ompl

#endif
