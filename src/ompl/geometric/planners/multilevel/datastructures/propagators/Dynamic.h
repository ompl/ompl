#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_PROPAGATORS_Dynamic__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_PROPAGATORS_Dynamic__
#include <ompl/geometric/planners/multilevel/datastructures/propagators/BundleSpacePropagator.h>

namespace ompl
{
    namespace geometric
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
