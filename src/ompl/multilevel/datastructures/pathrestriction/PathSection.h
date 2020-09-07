#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_SECTION__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_SECTION__
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/util/ClassForward.h>

namespace ompl
{
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(PathRestriction);
        OMPL_CLASS_FORWARD(BasePathHead);

        /** \brief Representation of a path section (not necessarily feasible). 
         *
         *  This class provides convenience methods to interpolate different
         *  path section over a given path restriction */

        class PathSection
        {
          public:
            using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;

            PathSection() = delete;
            PathSection(PathRestriction*);
            virtual ~PathSection();
            
            /** \brief Interpolate along restriction using L2 metric
              *  ---------------
              *            ____x
              *       ____/
              *   ___/
              *  x
              *  --------------- */
            void interpolateL2();
                // const base::State* fiberStart, 
                // const base::State* fiberGoal);

            /** \brief Interpolate along restriction using L1 metric
              * (Fiber first)
              *   ---------------
              *    _____________x
              *   |
              *   |
              *   x
              *   --------------- */
            void interpolateL1FiberFirst(BasePathHeadPtr&);

            /** \brief Interpolate along restriction using L1 metric (Fiber Last)
              *   ---------------
              *                 x
              *                 |
              *                 |
              *   x_____________|
              *   --------------- */
            void interpolateL1FiberLast(BasePathHeadPtr&);

            /** \brief Checks if section is feasible
             *
             *  @retval True if feasible and false if only partially feasible
             *  @retval Basepathheadptr Return last valid
             */
            bool checkMotion(BasePathHeadPtr&);

            /** \brief checks if section is feasible */
            void sanityCheck();

            // double getLastValidBasePathLocation();

            int getLastValidBasePathIndex();
            int getLastValidSectionPathIndex();

            base::State* at(int k) const;

            int size() const;

            /** \brief Add vertex for sNext and edge to xLast by assuming motion
             * is valid  */
            Configuration *addFeasibleSegment(Configuration *xLast, base::State *sNext);

            void addFeasibleGoalSegment(
                Configuration *xLast, 
                Configuration *xGoal);

            Configuration* getLastValidConfiguration();

            friend std::ostream &operator<<(std::ostream &, const PathSection&);

            void print() const;

          protected:

            PathRestriction *restriction_;

            BasePathHeadPtr head_;

            /** \brief Interpolated section along restriction */
            std::vector<base::State*> section_;

            std::vector<int> sectionBaseStateIndices_;

            Configuration *xBundleLastValid_;

            /** \brief Last valid state on feasible segment */
            std::pair<base::State *, double> lastValid_;

            double lastValidLocationOnBasePath_;
            int lastValidIndexOnBasePath_;
            int lastValidIndexOnSectionPath_;

            base::State *xBaseTmp_{nullptr};
            base::State *xBundleTmp_{nullptr};

            base::State *xFiberStart_{nullptr};
            base::State *xFiberGoal_{nullptr};
            base::State *xFiberTmp_{nullptr};
        };
    }
}
#endif
