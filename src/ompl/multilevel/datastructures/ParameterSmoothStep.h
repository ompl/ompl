#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_PARAMETER_SMOOTH_STEP_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_PARAMETER_SMOOTH_STEP_

#include <ompl/util/Exception.h>
#include <ompl/multilevel/datastructures/Parameter.h>

namespace ompl
{
    class ParameterSmoothStep : public Parameter
    {
    public:
        ParameterSmoothStep() = default;

        ParameterSmoothStep(double initValue) : Parameter(initValue){};

        ParameterSmoothStep(double initValue, double targetValue) : Parameter(initValue, targetValue){};

        double operator()(void)
        {
            // map to [0,1]
            double t = (getCounter() - getCounterInit()) / (double)(getCounterTarget() - getCounterInit());

            incrementCounter();

            if (t < 0)
                return getValueInit();
            if (t >= 1)
                return getValueTarget();

            return getValueInit() + (getValueTarget() - getValueInit()) * (t * t * (3 - 2 * t));
        }
    };
}
#endif
