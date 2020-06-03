#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_IMPORTANCE_Uniform_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_IMPORTANCE_Uniform_
#include <ompl/geometric/planners/multilevel/datastructures/importance/BundleSpaceImportance.h>

namespace ompl
{
    namespace geometric
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
