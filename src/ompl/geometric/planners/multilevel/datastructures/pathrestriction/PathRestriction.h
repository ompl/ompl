#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_PATH_SECTION_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_PATH_SECTION_
#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceGraph.h>

namespace ompl
{
  
  namespace base
  {
      OMPL_CLASS_FORWARD(Path);
  }
  namespace geometric
  {
      OMPL_CLASS_FORWARD(BundleSpaceGraph);
      OMPL_CLASS_FORWARD(PathGeometric);

      /// \brief Representation of path restriction (set of all elements of bundle space
      //which projects onto a given base path --- i.e. a union of fibers over base path). 
      //
      //This class has additional
      //functionalities to find path sections (paths lying inside path
      //restriction) using different interpolation methods (shortest L1, L2
      //paths)

      class BundleSpacePathRestriction
      {
        public:
          using Configuration = ompl::geometric::BundleSpaceGraph::Configuration;
          BundleSpacePathRestriction() = delete;
          BundleSpacePathRestriction(BundleSpaceGraph*); 

          virtual ~BundleSpacePathRestriction();

          void setBasePath(base::PathPtr);
          void setBasePath(std::vector<base::State*>);

          bool checkSection(
                Configuration* const xStart,
                Configuration* const xGoal);

          bool checkSectionRecursiveRepair(
                Configuration* const xStart,
                Configuration* const xGoal,
                const std::vector<base::State*> basePath,
                unsigned int depth=0,
                double startLength=0.0,
                bool interpolateL1=true);

          bool sideStepAlongFiber(const base::State* xBase, base::State* xBundle);

          void sanityCheckSection();

          Configuration* addFeasibleSegment(
              const Configuration* xLast, 
              base::State *sNext);

          void addFeasibleGoalSegment( 
              Configuration* const xLast, 
              Configuration* const xGoal);

          //Note: 
          //const ptr* means that the pointer itself is const
          //ptr* const means that the content of the pointer is const (but ptr
          //can change)

          bool hasFeasibleSection(Configuration* const, Configuration* const);

          //\brief Interpolate along restriction using L2 metric
          //  ---------------
          //            ____x
          //       ____/
          //   ___/
          //  x
          //  ---------------
          std::vector<base::State*> interpolateSectionL2(
                const base::State* xFiberStart,
                const base::State* xFiberGoal,
                const std::vector<base::State*> basePath);

          //\brief Interpolate along restriction using L1 metric
          //  ---------------
          //                x
          //                |
          //                |
          //  x_____________|
          //  ---------------
          std::vector<base::State*> interpolateSectionL1(
                const base::State* xFiberStart,
                const base::State* xFiberGoal,
                const std::vector<base::State*> basePath);

          //\brief Interpolate along restriction using L1 metric, but first
          //interpolate along fiber
          //  ---------------
          //   _____________x
          //  |
          //  |
          //  x
          //  ---------------
          std::vector<base::State*> interpolateSectionL1_FiberFirst(
                const base::State* xFiberStart,
                const base::State* xFiberGoal,
                const std::vector<base::State*> basePath);

        protected:

          BundleSpaceGraph* bundleSpaceGraph_;

          std::vector<base::State*> basePath_;

          double lengthBasePath_{0.0};
          std::vector<double> intermediateLengthsBasePath_;

          base::State *xBaseTmp_{nullptr};
          base::State *xBundleTmp_{nullptr};

          base::State *xFiberStart_{nullptr};
          base::State *xFiberGoal_{nullptr};
          base::State *xFiberTmp_{nullptr};

          std::vector<base::State*> section_;

          std::pair<base::State*, double> lastValid_;
      };
  }
}

#endif
