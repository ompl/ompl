#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_PATH_SECTION_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_PATH_SECTION_
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceGraph.h>

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
      //which projects onto a given base path). This class has additional
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

          bool hasFeasibleSection(Configuration* const, Configuration* const);

          //  ---------------
          //            ____x
          //       ____/
          //   ___/
          //  x
          //  ---------------
          std::vector<base::State*> interpolateSectionL2(
                const base::State* xFiberStart,
                const base::State* xFiberGoal);

          //  ---------------
          //                x
          //                |
          //                |
          //  x_____________|
          //  ---------------
          std::vector<base::State*> interpolateSectionL1(
                const base::State* xFiberStart,
                const base::State* xFiberGoal);

          //  ---------------
          //   _____________x
          //  |
          //  |
          //  x
          //  ---------------
          std::vector<base::State*> interpolateSectionL1_FiberFirst(
                const base::State* xFiberStart,
                const base::State* xFiberGoal);

        protected:

          BundleSpaceGraph* bundleSpaceGraph_;

          std::vector<base::State*> basePath_;

          double lengthBasePath_{0.0};
          std::vector<double> intermediateLengthsBasePath_;

          base::State *xFiberStart_{nullptr};
          base::State *xFiberGoal_{nullptr};
          base::State *xFiberTmp_{nullptr};

          std::vector<base::State*> bundleSection_;
      };
  }
}

#endif
