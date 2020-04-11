#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_IMPORTANCE_GREEDY_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_IMPORTANCE_GREEDY_
#include <ompl/geometric/planners/multilevel/datastructures/importance/BundleSpaceImportance.h>

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
      private:
        double getLevelConstant();
        double epsilon{0.1};
    };
  }
}


#endif


