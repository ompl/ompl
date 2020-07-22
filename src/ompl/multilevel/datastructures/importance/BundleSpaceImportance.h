#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_IMPORTANCE_IMPORTANCE_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_IMPORTANCE_IMPORTANCE_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <memory>

namespace ompl
{
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(BundleSpaceGraph);

        class BundleSpaceImportance
        {
        public:
            using Configuration = BundleSpaceGraph::Configuration;
            BundleSpaceImportance() = delete;
            BundleSpaceImportance(BundleSpaceGraph *graph) : bundleSpaceGraph_(graph){};

            virtual ~BundleSpaceImportance() = default;

            virtual double eval() = 0;

            virtual void reset(){};

        protected:
            BundleSpaceGraph *bundleSpaceGraph_;
        };
    }
}

#endif
