#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"
#include <algorithm>

double ompl::SpaceInformationKinematic::distance(const StateKinematic_t s1, const StateKinematic_t s2)
{
    double dist = 0.0;
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
    {
	double diff = s1->values[i] - s2->values[i];
	dist += diff * diff;
    }
    return dist;
}

void ompl::SpaceInformationKinematic::sample(StateKinematic_t state)
{
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	state->values[i] = random_utils::uniform(&m_rngState, m_stateComponent[i].minValue, m_stateComponent[i].maxValue);	    
}

void ompl::SpaceInformationKinematic::sampleNear(StateKinematic_t state, const StateKinematic_t near, double rho)
{
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	state->values[i] = m_stateComponent[i].type == StateComponent::FIXED ?
	    near->values[i] :
	    random_utils::uniform(&m_rngState, 
				  std::max(m_stateComponent[i].minValue, near->values[i] - rho), 
				  std::min(m_stateComponent[i].maxValue, near->values[i] + rho));
}
