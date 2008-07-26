#include "ompl/extension/samplingbased/kinematic/extension/rrt/LazyRRT.h"

bool ompl::LazyRRT::solve(double solveTime)
{
    SpaceInformationKinematic_t                          si = dynamic_cast<SpaceInformationKinematic_t>(m_si); 
    SpaceInformationKinematic::GoalRegionKinematic_t goal_r = dynamic_cast<SpaceInformationKinematic::GoalRegionKinematic_t>(si->getGoal());
    SpaceInformationKinematic::GoalStateKinematic_t  goal_s = dynamic_cast<SpaceInformationKinematic::GoalStateKinematic_t>(si->getGoal());
    unsigned int                                        dim = si->getStateDimension();

    time_utils::Time endTime = time_utils::Time::now() + time_utils::Duration(solveTime);

    for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
    {
	Motion_t motion = new Motion(dim);
	si->copyState(motion->state, dynamic_cast<SpaceInformationKinematic::StateKinematic_t>(si->getStartState(i)));
	if (si->isValid(motion->state))
	{
	    motion->valid = true;
	    m_nn.add(motion);
	}	
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
    
    Motion_t                                    solution = NULL;
    Motion_t                                    rmotion  = new Motion(dim);
    SpaceInformationKinematic::StateKinematic_t rstate   = rmotion->state;
    SpaceInformationKinematic::StateKinematic_t xstate   = new SpaceInformationKinematic::StateKinematic(dim);
    
 RETRY:

    while (time_utils::Time::now() < endTime)
    {
	/* sample random state (with goal biasing) */
	if (goal_s && random_utils::uniform(&m_rngState, 0.0, 1.0) < m_goalBias)
	    si->copyState(rstate, goal_s->state);
	else
	    si->sample(rstate);

	/* find closest state in the tree */
	Motion_t nmotion = m_nn.nearest(rmotion);
	
	/* find state to add */
	for (unsigned int i = 0 ; i < dim ; ++i)
	{
	    double diff = rmotion->state->values[i] - nmotion->state->values[i];
	    xstate->values[i] = fabs(diff) < range[i] ? rmotion->state->values[i] : nmotion->state->values[i] + diff * m_rho;
	}
	
	/* create a motion */
	Motion_t motion = new Motion(dim);
	si->copyState(motion->state, xstate);
	motion->parent = nmotion;
	nmotion->children.push_back(motion);
	m_nn.add(motion);
	
	if (goal_r->distanceGoal(motion->state) < goal_r->threshold)
	{
	    solution = motion;
	    break;	    
	}
    }    
    
    if (solution != NULL)
    {
	/* construct the solution path */
	std::vector<Motion_t> mpath;
	while (solution != NULL)
	{
	    mpath.push_back(solution);
	    solution = solution->parent;
	}

	/* check the path */
	for (int i = mpath.size() - 1 ; i >= 0 ; ++i)
	    if (!mpath[i]->valid)
		if (si->checkMotion(mpath[i]->parent->state, mpath[i]->state))
		    mpath[i]->valid = true;
		else
		{
		    removeMotion(mpath[i]);
		    goto RETRY;
		}

	/*set the solution path */
	SpaceInformationKinematic::PathKinematic_t path = new SpaceInformationKinematic::PathKinematic(m_si);
   	for (int i = mpath.size() - 1 ; i >= 0 ; ++i)
	    path->states.push_back(mpath[i]->state);
	goal_r->setSolutionPath(path);	
    }

    delete xstate;
    delete rmotion;

    return goal_r->isAchieved();
}

void ompl::LazyRRT::removeMotion(Motion_t motion)
{
    
}

