#ifndef OMPL_GEOMETRIC_PLANNERS_OPTIMIZER_PATHCONTROL__
#define OMPL_GEOMETRIC_PLANNERS_OPTIMIZER_PATHCONTROL__

#include <ompl/control/PathControl.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include "ompl/util/RandomNumbers.h"
#include "ompl/base/DynamicalMotionValidator.h"



namespace ompl
{
    namespace control
    {
        class PathControlOptimizer
        {
          public:

            PathControlOptimizer(base::SpaceInformationPtr si, const base::OptimizationObjectivePtr& obj=nullptr);
            void simplify(PathControl* path);
	    void reduceVertices(PathControl &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps =0, double rangeRatio=0.9);
	    void collapseCloseVertices(PathControl &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps =0);
	        //void subdivide(PathControl *path) ;
	        //void smoothBSpline(PathControl &path, unsigned int maxSteps = 5,double minChange = std::numeric_limits<double>::epsilon());

          protected:

            base::SpaceInformationPtr si_;

            base::OptimizationObjectivePtr obj_;
            
            bool freeStates_ ;
            
            RNG rng_ ;
        };
    }
}

#endif
