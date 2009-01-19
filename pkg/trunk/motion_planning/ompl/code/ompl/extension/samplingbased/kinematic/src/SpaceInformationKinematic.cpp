/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* \author Ioan Sucan */

#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"
#include <angles/angles.h>
#include <cstring>
#include <valarray>
#include <algorithm>
#include <queue>
#include <cstring>

bool ompl::SpaceInformationKinematic::GoalRegionKinematic::isSatisfied(State_t s, double *distance)
{
    double d2g = distanceGoal(static_cast<StateKinematic_t>(s));
    if (distance)
	*distance = d2g;
    return d2g < threshold;
}

void ompl::SpaceInformationKinematic::GoalRegionKinematic::print(std::ostream &out) const
{
    out << "Goal region, threshold = " << threshold << ", memory address = " << reinterpret_cast<const void*>(this) << std::endl;
}

double ompl::SpaceInformationKinematic::GoalStateKinematic::distanceGoal(StateKinematic_t s)
{
    return static_cast<SpaceInformationKinematic_t>(m_si)->distance(s, state);
}

void ompl::SpaceInformationKinematic::GoalStateKinematic::print(std::ostream &out) const
{
    out << "Goal state, threshold = " << threshold << ", memory address = " << reinterpret_cast<const void*>(this) << ", state = ";
    static_cast<SpaceInformationKinematic_t>(m_si)->printState(state, out);
}

void ompl::SpaceInformationKinematic::PathKinematic::freeMemory(void)
{
    for (unsigned int i = 0 ; i < states.size() ; ++i)
	delete states[i];
}

void ompl::SpaceInformationKinematic::copyState(StateKinematic_t destination, const StateKinematic_t source)
{
    memcpy(destination->values, source->values, sizeof(double) * m_stateDimension);
}

double ompl::SpaceInformationKinematic::StateKinematicL2SquareDistanceEvaluator::operator()(const State_t s1, const State_t s2)
{
    const StateKinematic_t sk1 = static_cast<const StateKinematic_t>(s1);
    const StateKinematic_t sk2 = static_cast<const StateKinematic_t>(s2);
    const unsigned int     dim = m_si->getStateDimension();
    
    double dist = 0.0;
    for (unsigned int i = 0 ; i < dim ; ++i)
    {	 
	double diff = m_si->getStateComponent(i).type == StateComponent::WRAPPING_ANGLE ? 
	    angles::shortest_angular_distance(sk1->values[i], sk2->values[i]) : sk1->values[i] - sk2->values[i];
	dist += diff * diff;
    }
    return dist;
}

unsigned int ompl::SpaceInformationKinematic::getStateDimension(void) const
{
    return m_stateDimension;
}

const ompl::SpaceInformationKinematic::StateComponent& ompl::SpaceInformationKinematic::getStateComponent(unsigned int index) const
{
    return m_stateComponent[index];
}

void ompl::SpaceInformationKinematic::setup(void)
{
    assert(m_stateDimension > 0);
    assert(m_stateComponent.size() == m_stateDimension);
    assert(m_stateValidityChecker);
    assert(m_stateDistanceEvaluator);
    SpaceInformation::setup();
}

void ompl::SpaceInformationKinematic::printState(const StateKinematic_t state, std::ostream &out) const
{
    if (state)
    {
	for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	    out << state->values[i] << " ";
	out << std::endl;
    }
    else
	out << "NULL" << std::endl;
}

void ompl::SpaceInformationKinematic::sample(StateKinematic_t state)
{
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	if (m_stateComponent[i].type == StateComponent::QUATERNION)
	{
	    random_utils::quaternion(&m_rngState, state->values + i);
	    i += 3;
	}
	else
	state->values[i] = random_utils::uniform(&m_rngState, m_stateComponent[i].minValue, m_stateComponent[i].maxValue);	    
}

void ompl::SpaceInformationKinematic::sampleNear(StateKinematic_t state, const StateKinematic_t near, const double rho)
{
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	if (m_stateComponent[i].type == StateComponent::QUATERNION)
	{
	    /* no notion of 'near' is employed for quaternions */
	    random_utils::quaternion(&m_rngState, state->values + i);
	    i += 3;
	}
	else
	    state->values[i] =
		random_utils::uniform(&m_rngState, 
				      std::max(m_stateComponent[i].minValue, near->values[i] - rho), 
				      std::min(m_stateComponent[i].maxValue, near->values[i] + rho));
}

void ompl::SpaceInformationKinematic::sampleNear(StateKinematic_t state, const StateKinematic_t near, const std::vector<double> &rho)
{
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	if (m_stateComponent[i].type == StateComponent::QUATERNION)
	{
	    /* no notion of 'near' is employed for quaternions */
	    random_utils::quaternion(&m_rngState, state->values + i);
	    i += 3;
	}
	else
	    state->values[i] = 
		random_utils::uniform(&m_rngState, 
				      std::max(m_stateComponent[i].minValue, near->values[i] - rho[i]), 
				      std::min(m_stateComponent[i].maxValue, near->values[i] + rho[i]));
}

bool ompl::SpaceInformationKinematic::checkMotionSubdivision(const StateKinematic_t s1, const StateKinematic_t s2)
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!isValid(s2))
	return false;

    /* find out how many divisions we need */
    int nd = 1;
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
    {
	int d = 1 + (int)(fabs(s1->values[i] - s2->values[i]) / m_stateComponent[i].resolution);
	if (nd < d)
	    nd = d;
    }

    /* find out the step size as a vector */
    std::valarray<double> step(m_stateDimension);
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
    	step[i] = (s2->values[i] - s1->values[i]) / (double)nd;
    
    /* initialize the queue of test positions */
    std::queue< std::pair<int, int> > pos;
    if (nd >= 2)
	pos.push(std::make_pair(1, nd - 1));
    
    /* temporary storage for the checked state */
    StateKinematic test(m_stateDimension);

    /* repeatedly subdivide the path segment in the middle (and check the middle) */
    while (!pos.empty())
    {
	std::pair<int, int> x = pos.front();

	int mid = (x.first + x.second) / 2;

	for (unsigned int j = 0 ; j < m_stateDimension ; ++j)
	    test.values[j] = s1->values[j] + (double)mid * step[j];

	if (!isValid(&test))
	    return false;

	pos.pop();
	
	if (x.first < mid)
	    pos.push(std::make_pair(x.first, mid - 1));
	if (x.second > mid)
	    pos.push(std::make_pair(mid + 1, x.second));
    }
            
    return true;
}

bool ompl::SpaceInformationKinematic::checkMotionIncremental(const StateKinematic_t s1, const StateKinematic_t s2)
{   
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!isValid(s2))
	return false;
    
    /* find out how many divisions we need */
    int nd = 1;
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
    {
	int d = 1 + (int)(fabs(s1->values[i] - s2->values[i]) / m_stateComponent[i].resolution);
	if (nd < d)
	    nd = d;
    }

    /* find out the step size as a vector */
    std::valarray<double> step(m_stateDimension);
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
    	step[i] = (s2->values[i] - s1->values[i]) / (double)nd;
    
    /* temporary storage for the checked state */
    StateKinematic test(m_stateDimension);
    
    for (int j = 1 ; j < nd ; ++j)
    {
	for (unsigned int k = 0 ; k < m_stateDimension ; ++k)
	    test.values[k] = s1->values[k] + (double)j * step[k];
	if (!isValid(&test))
	    return false;
    }

    return true;
}

void ompl::SpaceInformationKinematic::interpolatePath(PathKinematic_t path, double factor)
{
    std::vector<StateKinematic_t> newStates;
    const int n1 = path->states.size() - 1;
    
    for (int i = 0 ; i < n1 ; ++i)
    {
	StateKinematic_t s1 = path->states[i];
	StateKinematic_t s2 = path->states[i + 1];
	
	newStates.push_back(s1);
	
	/* find out how many divisions we need */
	int nd = 1;
	for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	{
	    int d = 1 + (int)(fabs(s1->values[i] - s2->values[i]) / (factor * m_stateComponent[i].resolution));
	    if (nd < d)
		nd = d;
	}
	
	/* find out the step size as a vector */
	std::valarray<double> step(m_stateDimension);
	for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	    step[i] = (s2->values[i] - s1->values[i]) / (double)nd;
	
	/* find the states in between */
	for (int j = 1 ; j < nd ; ++j)
	{
	    StateKinematic_t state = new StateKinematic(m_stateDimension);
	    for (unsigned int k = 0 ; k < m_stateDimension ; ++k)
		state->values[k] = s1->values[k] + (double)j * step[k];
	    newStates.push_back(state);
	}
    }
    newStates.push_back(path->states[n1]);
    
    path->states.swap(newStates);
}

double ompl::SpaceInformationKinematic::distance(const StateKinematic_t s1, const StateKinematic_t s2)
{
    return (*m_stateDistanceEvaluator)(static_cast<const State_t>(s1), static_cast<const State_t>(s2));
}

bool ompl::SpaceInformationKinematic::isValid(const StateKinematic_t state)
{
    return (*m_stateValidityChecker)(static_cast<const State_t>(state));
}
	
void ompl::SpaceInformationKinematic::printSettings(std::ostream &out) const
{
    out << "Kinematic state space settings:" << std::endl;
    out << "  - dimension = " << m_stateDimension << std::endl;
    out << "  - start states:" << std::endl;
    for (unsigned int i = 0 ; i < getStartStateCount() ; ++i)
	printState(dynamic_cast<const StateKinematic_t>(getStartState(i)), out);
    if (m_goal)
	m_goal->print(out);
    else
	out << "  - goal = NULL" << std::endl;
    out << "  - bounding box:" << std::endl;
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	out << "[" << m_stateComponent[i].minValue << ", " <<  m_stateComponent[i].maxValue << "](" << m_stateComponent[i].resolution << ") ";
    out << std::endl;
}
