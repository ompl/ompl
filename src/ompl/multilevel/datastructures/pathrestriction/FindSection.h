#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_FIND_SECTION_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_FIND_SECTION_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/datastructures/ExponentialDecay.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(Path);
    }
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathGeometric);
    }
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(BundleSpaceGraph);
        OMPL_CLASS_FORWARD(PathRestriction);
        OMPL_CLASS_FORWARD(BasePathHead);

        using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;

        class FindSection
        {
        public:

            FindSection() = delete;
            FindSection(PathRestriction*);

            virtual ~FindSection();

            virtual bool solve(BasePathHeadPtr& head) = 0;

            /** \brief Sample state on fiber while keeping base state fixed */
            bool findFeasibleStateOnFiber(
                const base::State *xBase, 
                base::State *xBundle);
        protected:
            /** \brief Pointer to associated bundle space */
            PathRestriction *restriction_;

            base::State *xBaseTmp_{nullptr};
            base::State *xBundleTmp_{nullptr};

            base::State *xFiberStart_{nullptr};
            base::State *xFiberGoal_{nullptr};
            base::State *xFiberTmp_{nullptr};

            /** \brief Radius of restriction neighborhood */
            ExponentialDecay neighborhoodRadiusBaseSpace_;

            double neighborhoodRadiusBaseSpaceLambda_{1.0};

            double neighborhoodRadiusBaseSpaceTarget_{0.5};

            /** \brief Step size to check validity */
            double validBaseSpaceSegmentLength_;

            double validBundleSpaceSegmentLength_;

            double validFiberSpaceSegmentLength_;
        };
    }
}

#endif

