/* MorseProjection.cpp */

#include "ompl/extensions/morse/MorseProjection.h"
#include "ompl/util/Exception.h"

ompl::base::MorseProjection::MorseProjection(const StateSpacePtr &space)
: ProjectionEvaluator(space), space_(dynamic_cast<MorseStateSpace*>(space.get()))
{
    if (!space_)
        throw Exception("MORSE State Space needed for Morse Projection");
}

void ompl::base::MorseProjection::setup(void)
{
    ProjectionEvaluator::setup();
}

unsigned int ompl::base::MorseProjection::getDimension(void) const
{
    return 2*space_->getEnvironment()->rigidBodies_;
}

void ompl::base::MorseProjection::defaultCellSizes(void)
{
    cellSizes_.resize(getDimension());
    for (unsigned int i = 0; i < getDimension(); i++)
    {
        cellSizes_[i] = 1.0;
    }
}

// this projection uses the x and y coordinates of every rigid body
void ompl::base::MorseProjection::project(const State *state, EuclideanProjection &projection) const
{
    const MorseStateSpace::StateType *mstate = state->as<MorseStateSpace::StateType>();
    projection.resize(getDimension());
    for (unsigned int i = 0; i < space_->getEnvironment()->rigidBodies_; i++)
    {
        const double *values = mstate->as<RealVectorStateSpace::StateType>(4*i)->values;
        projection[2*i] = values[0];
        projection[2*i+1] = values[1];
    }
}

