#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_IMPORTANCE_GREEDY_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_IMPORTANCE_GREEDY_
#include <ompl/geometric/planners/quotientspace/datastructures/importance/BundleSpaceImportance.h>

namespace ompl
{
  namespace geometric
  {
    class BundleSpaceImportanceGreedy: public BundleSpaceImportance
    {
        using BaseT = BundleSpaceImportance;
      public:

        BundleSpaceImportanceGreedy() = delete;
        BundleSpaceImportanceGreedy(BundleSpaceGraph* graph);

        ~BundleSpaceImportanceGreedy() = default;

        virtual double eval() override;
    };
  }
}


#endif


