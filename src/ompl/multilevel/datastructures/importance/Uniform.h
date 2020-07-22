#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_IMPORTANCE_Uniform_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_IMPORTANCE_Uniform_
#include <ompl/multilevel/datastructures/importance/BundleSpaceImportance.h>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceImportanceUniform : public BundleSpaceImportance
        {
            using BaseT = BundleSpaceImportance;

        public:
            BundleSpaceImportanceUniform() = delete;
            BundleSpaceImportanceUniform(BundleSpaceGraph *graph);

            ~BundleSpaceImportanceUniform() = default;

            virtual double eval() override;
        };
    }
}

#endif
