#include "ompl/extension/samplingbased/kinematic/extension/rrt/RRT.h"

bool ompl::RRT::solve(double solveTime)
{
    SpaceInformationKinematic_t                          si = dynamic_cast<SpaceInformationKinematic_t>(m_si); 
    SpaceInformationKinematic::GoalRegionKinematic_t goal_r = dynamic_cast<SpaceInformationKinematic::GoalRegionKinematic_t>(si->getGoal());
    SpaceInformationKinematic::GoalStateKinematic_t  goal_s = dynamic_cast<SpaceInformationKinematic::GoalStateKinematic_t>(si->getGoal());
    unsigned int                                        dim = si->getStateDimension();
    
    time_utils::timestamp dt = time_utils::now();
    
    for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
    {
	SpaceInformationKinematic::MotionKinematic_t motion = new SpaceInformationKinematic::MotionKinematic(dim);
	si->copyState(motion->state, dynamic_cast<SpaceInformationKinematic::StateKinematic_t>(si->getStartState(i)));
	if (si->isValid(motion->state))
	    m_nn.add(motion);
	else
	{
	    fprintf(stderr, "Initial state is in collision!\n");
	    delete motion;
	}	
    }

    if (m_nn.size() == 0)
    {
	fprintf(stderr, "There are no valid initial states!\n");
	return false;	
    }    

    std::vector<double> range(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	range[i] = m_rho * si->getStateComponent(i).maxValue - si->getStateComponent(i).minValue;
    
    SpaceInformationKinematic::MotionKinematic_t solution = NULL;
    
    SpaceInformationKinematic::MotionKinematic_t rmotion = new SpaceInformationKinematic::MotionKinematic(dim);
    SpaceInformationKinematic::StateKinematic_t  rstate  = rmotion->state;

    SpaceInformationKinematic::StateKinematic_t  xstate  = new SpaceInformationKinematic::StateKinematic(dim);
    
    while (time_utils::elapsed(dt) < solveTime)
    {
	/* sample random state (with goal biasing) */
	if (goal_s && random_utils::uniform(&m_rngState, 0.0, 1.0) < m_goalBias)
	    si->copyState(rstate, goal_s->state);
	else
	    si->sample(rstate);

	/* find closest state in the tree */
	SpaceInformationKinematic::MotionKinematic_t nmotion = m_nn.nearest(rmotion);
	
	/* find state to add */
	for (unsigned int i = 0 ; i < dim ; ++i)
	{
	    double diff = rmotion->state->values[i] - nmotion->state->values[i];
	    xstate->values[i] = fabs(diff) < range[i] ? rmotion->state->values[i] : nmotion->state->values[i] + diff * m_rho;
	}
	
	/* create a motion */
	SpaceInformationKinematic::MotionKinematic_t motion = new SpaceInformationKinematic::MotionKinematic(dim);
	si->copyState(motion->state, xstate);
	motion->parent = nmotion;
	m_nn.add(motion);
	
	if (goal_r->distanceGoal(motion->state) < goal_r->threshold)
	{
	    solution = motion;
	    break;	    
	}
    }    
    
    delete xstate;
    delete rmotion;

    if (solution != NULL)
    {
	SpaceInformationKinematic::PathKinematic_t path = new SpaceInformationKinematic::PathKinematic(m_si);
	
	std::vector<SpaceInformationKinematic::StateKinematic_t> states;
	while (solution != NULL)
	{
	    states.push_back(solution->state);
	    solution = solution->parent;
	}
	for (int i = states.size() - 1 ; i >= 0 ; ++i)
	    path->states.push_back(states[i]);	

	goal_r->setSolutionPath(path);	
    }
    
    return goal_r->isAchieved();
}
