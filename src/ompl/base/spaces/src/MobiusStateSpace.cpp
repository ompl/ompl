#include "ompl/base/spaces/MobiusStateSpace.h"
#include "ompl/tools/config/MagicConstants.h"
#include <cstring>

#include <boost/math/constants/constants.hpp>

using namespace boost::math::double_constants;

ompl::base::State *ompl::base::MobiusStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void ompl::base::MobiusStateSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

void ompl::base::MobiusStateSpace::interpolate(
    const State *from, const State *to, double t, State *state) const
{
    const double xFrom = from->as<StateType>()->getX();
    const double sFrom = from->as<StateType>()->getS1();

    const double xTo = to->as<StateType>()->getX();
    const double sTo = to->as<StateType>()->getS1();

    StateType *result = state->as<StateType>();
    // double& xResult = state->as<StateType>()->getX();
    // double& sResult = state->as<StateType>()->getS1();

    //Interpolate along S1 (independently)
    const base::State *sFromSO2 = 
      from->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    const base::State *sToSO2 = 
      to->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);

    base::State *stateSO2 = 
      state->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);

    const base::SO2StateSpace *SO2 = this->as<base::SO2StateSpace>(0);
    SO2->interpolate(sFromSO2, sToSO2, t, stateSO2);

    //Interpolate along T (depends on S1)
    double sDiff = sTo - sFrom;
    double xDiff = xTo - xFrom;
    if (fabs(sDiff) <= pi)
        result->setX(xFrom + t * xDiff);
    else
    {
        OMPL_ERROR("NYI");
    }
}

// double ompl::base::MobiusStateSpace::distance(const State *state1, const State *state2) const
// {
//     OMPL_ERROR("NYI");
// }
