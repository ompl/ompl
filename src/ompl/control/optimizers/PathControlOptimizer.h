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

            PathControlOptimizer(base::SpaceInformationPtr si, ompl::base::State* goalState  ,const base::OptimizationObjectivePtr& obj=nullptr);
            void simplify(PathControl* path);

            void reduceVertices(PathControl &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps =0, double rangeRatio=0.9);
            
            void addIntermediaryStates( PathControl &path) ;
            
            bool connectConsecutiveStates(unsigned int position, ompl::control::PathControl &path, ompl::base::State* state, control::SpaceInformation* siC, ModifiedDirectedControlSamplerPtr sampler ) ;
            
			bool connectStateToGoal(unsigned int position, ompl::control::PathControl &path, ompl::base::State* state, control::SpaceInformation* siC, ModifiedDirectedControlSamplerPtr sampler ) ;
			
			bool connectStates(unsigned int initial, unsigned int goal , ompl::control::PathControl &path, ompl::base::State* state, control::SpaceInformation* siC, ModifiedDirectedControlSamplerPtr sampler, ompl::base::State* reached_State) ;
			
			bool connectStates(unsigned int initial, unsigned int goal , ompl::control::PathControl &path, ompl::base::State* state, control::SpaceInformation* siC, ModifiedDirectedControlSamplerPtr sampler) ;


          protected:

            base::SpaceInformationPtr si_;

            base::OptimizationObjectivePtr obj_;
            
            bool freeStates_ ;
            
            RNG rng_ ;
            
            ompl::base::State* goalState_ ; 
        };
    }
}

#endif
