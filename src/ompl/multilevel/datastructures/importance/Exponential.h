#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_IMPORTANCE_EXPONENTIAL_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_IMPORTANCE_EXPONENTIAL_
#include <ompl/multilevel/datastructures/importance/BundleSpaceImportance.h>

namespace ompl
{
    namespace multilevel
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
