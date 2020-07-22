#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PROPAGATORS_Dynamic__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PROPAGATORS_Dynamic__
#include <ompl/multilevel/datastructures/propagators/BundleSpacePropagator.h>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpacePropagatorDynamic : public BundleSpacePropagator
        {
            using BaseT = BundleSpacePropagator;

        public:
            BundleSpacePropagatorDynamic() = delete;
            BundleSpacePropagatorDynamic(BundleSpaceGraph *);

            virtual ~BundleSpacePropagatorDynamic() override;

            virtual bool steer(const Configuration *from, const Configuration *to, Configuration *result) override;
        };
    }
}

#endif
