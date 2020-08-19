#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_BASEPATHHEAD__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_BASEPATHHEAD__
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>

namespace ompl
{
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(BundleSpaceGraph);
        OMPL_CLASS_FORWARD(PathRestriction);

        using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;


        /** \brief A pointer to a specific location on the base path of the path
         * restriction
         *
         * This class contains information about the current location over the
         * base path, including the distance traveled from start, the last index
         * of the base path states and information about the fiber space and
         * fiber element at the current location. Can be used to interpolate
         * from current location to goal of path restriction.
         *
        */

        class BasePathHead
        {
          public:
            BasePathHead(
                PathRestriction *restriction,
                Configuration* const xStart,
                Configuration* const xGoal);

            ~BasePathHead();

            base::State* getFiberElementStart();
            base::State* getFiberElementGoal();

            Configuration* getStartConfiguration();
            Configuration* getGoalConfiguration();

            int getNumberOfRemainingStates();

            //relative to where the head points 
            const base::State* getBaseStateAt(int k);
            int getBaseStateIndexAt(int k);

            double getLocationOnBasePath();
            void setLocationOnBasePath(double d);

            int getLastValidBasePathIndex();
            void setLastValidBasePathIndex(int k);

          private:
            double locationOnBasePath_{0.0};
            double locationFromLastIndex_{0.0};
            int lastValidIndexOnBasePath_{0};

            PathRestriction* restriction_{nullptr};
            Configuration *xStart_{nullptr};
            Configuration *xGoal_{nullptr};

            base::State *xBaseStart_{nullptr};
            base::State *xFiberStart_{nullptr};
            base::State *xFiberGoal_{nullptr};
        };
    }
}

#endif
