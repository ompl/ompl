#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_IMPORTANCE_EXPONENTIAL_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_IMPORTANCE_EXPONENTIAL_
#include <ompl/geometric/planners/multilevel/datastructures/importance/BundleSpaceImportance.h>

namespace ompl
{
    namespace geometric
    {
        class BundleSpaceImportanceExponential : public BundleSpaceImportance
        {
            using BaseT = BundleSpaceImportance;

        public:
            BundleSpaceImportanceExponential() = delete;
            BundleSpaceImportanceExponential(BundleSpaceGraph *graph);

            ~BundleSpaceImportanceExponential() = default;

            virtual double eval() override;
        };
    }
}

#endif
