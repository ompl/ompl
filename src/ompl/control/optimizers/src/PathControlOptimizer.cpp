#include <ompl/control/optimizers/PathControlOptimizer.h>


ompl::control::PathControlOptimizer::PathControlOptimizer(base::SpaceInformationPtr si, ompl::base::State* goalState , const base::OptimizationObjectivePtr& obj)
  : si_(si), obj_(obj), freeStates_(true)
{
	goalState_=si_->cloneState(goalState) ;
}
void ompl::control::PathControlOptimizer::simplify(PathControl* path)
{
  OMPL_DEBUG("simplify");
	
	reduceVertices(*path,200,50,0.5);

	addIntermediaryStates(*path);

	
  OMPL_DEBUG("done simplify");

}



void ompl::control::PathControlOptimizer::reduceVertices(PathControl &path, unsigned int maxSteps, unsigned int maxEmptySteps, double rangeRatio)
{
	const PathControl path_old = PathControl(path) ;
	
	if (path.getStateCount() < 3)
		return;
 
	if (maxSteps == 0)
		maxSteps = path.getStateCount();
 
	if (maxEmptySteps == 0)
		maxEmptySteps = path.getStateCount();
 
	unsigned int nochange = 0;


	std::vector<base::State *> &states = path.getStates();
	std::vector<Control *> &controls = path.getControls() ;
	std::vector<double > &controlDurations = path.getControlDurations() ;
	
	const base::SpaceInformationPtr &si = path.getSpaceInformation();
	ompl::control::SpaceInformation *siC = static_cast<ompl::control::SpaceInformation*>(si.get());
	siC->setMotionValidator(std::make_shared<ompl::base::DynamicalMotionValidator>(siC));
	siC->setMinMaxControlDuration(1,500);
	
	ompl::control::ModifiedDirectedControlSamplerPtr sampler;
	sampler = siC->allocModifiedDirectedControlSampler();
	sampler->setNumControlSamples(500);
  
	std::cout << " propagation size " << siC->getPropagationStepSize() << std::endl;
	std::cout << "initial size of states  " << states.size() << std::endl;
	std::cout << "initial size of controls  " << controls.size() << std::endl;
  

  for (unsigned int i = 0; i < maxSteps && nochange < maxEmptySteps; ++i, ++nochange)
  {

    int count = states.size();
    if (count<=15) 
    {
      rangeRatio= 0.5 ;
    }
    int maxN = count - 1;
    int range = 1 + (int)(floor(0.5 + (double)count * rangeRatio));

    int p1 = rng_.uniformInt(0, maxN);
    int p2 = rng_.uniformInt(std::max(p1 - range, 0), std::min(maxN, p1 + range));
    if (abs(p1 - p2) < 2)
    {
      if (p1 < maxN - 1)
        p2 = p1 + 2;
      else if (p1 > 1)
        p2 = p1 - 2;
      else
        continue;
    }

    if (p1 > p2)
      std::swap(p1, p2);
  
    if( p1 >= (int)states.size() || p2 >= (int)states.size()){
      OMPL_ERROR("p1 or p2 larger than states.");
      continue;
    }
   
		
    ompl::base::State *stateP2_tmp = siC->allocState(); 
    if(connectStateToGoal(0, path, states.at(0), siC, sampler ))
    {
		return ;
	}
    
    if (connectStates(p1,p2,path,states.at(p1),siC,sampler,stateP2_tmp))
    {
		ompl::base::State *state_tmp = siC->allocState();
		
		for (unsigned int j = p1+1 ; j < states.size()-1 ; ++j )
		{
			if (connectStates(j,states.size()-1,path,stateP2_tmp,siC,sampler,state_tmp))
			{
				return ;
			}
			if (connectStates(j,j+1,path,stateP2_tmp,siC,sampler,state_tmp))
			{
				siC->copyState(stateP2_tmp, state_tmp) ;
				continue ;
			}
			else
			{
				std::cout<< "could not find a connection to next or goal states at state" << j <<std::endl ;
				path = PathControl(path_old) ;
				return ;
			}
		} 
		
	}
    
  }
}

bool ompl::control::PathControlOptimizer::connectStates(unsigned int initial, unsigned int goal , ompl::control::PathControl &path, ompl::base::State* initial_State, control::SpaceInformation* siC, ModifiedDirectedControlSamplerPtr sampler, ompl::base::State* reached_State) 
{
	const PathControl path_old = PathControl(path) ;
	
	if (reached_State == nullptr)
		reached_State = siC->allocState() ;
			
	
	std::vector<base::State *> &states = path.getStates();
	std::vector<Control *> &controls = path.getControls() ;
	std::vector<double > &controlDurations = path.getControlDurations() ;

    ompl::control::Control *newControl =siC->allocControl() ;
	
	base::State *state1 = siC->allocState();
	base::State *state2 = siC->allocState();
	base::State *state2_tmp = siC->allocState();
	siC->copyState(state1, initial_State);
	siC->copyState(state2, states.at(goal)); // goal -1 ??
	
	double steps = sampler->sampleTo(newControl, state1, state2);
	double cD = siC->propagateWhileValid(state1, newControl, steps, state2_tmp);

	//Check that we reached the goal
	const double d12 = siC->getStateSpace()->distance(state1, state2);
	const double targetRegion = 0.2 * d12;

	double distToTarget = siC->getStateSpace()->distance(state2_tmp, state2);
	
	if ((distToTarget > targetRegion) || (cD != steps)) 
	{
		std::cout << "could not connect the states " << initial << "and " << goal << std::endl;
		path = PathControl(path_old) ;
		return false ;
	}
	else 
	{
		std::cout << "a connection between " << initial << " and " << goal << " has been found"<< std::endl;
		std::cout << "the distance is " << distToTarget/d12*100 << "% " << std::endl;
        for (int j = initial + 1; j<goal ; j++)
        {
          siC->freeState(states.at(j));
          siC->freeControl(controls.at(j));
        }	
        states.erase(states.begin() + initial + 1, states.begin() + goal );	
        controls.erase(controls.begin() + initial + 1, controls.begin() + goal);	
        controlDurations.erase(controlDurations.begin() + initial + 1, controlDurations.begin() + goal);	
        siC->copyState(reached_State, state2_tmp);
		siC->copyState(states.at(initial+1), state2_tmp);
		siC->copyControl(controls.at(initial) , newControl) ;
		controlDurations.at(initial)= steps ;
		
	}
	
	return true ;		
	
}

bool ompl::control::PathControlOptimizer::connectConsecutiveStates(unsigned int position, ompl::control::PathControl &path, ompl::base::State* state , 
control::SpaceInformation* siC, ModifiedDirectedControlSamplerPtr sampler)
{
	return connectStates(position,position+1, path,state,siC,sampler) ;
}

bool ompl::control::PathControlOptimizer::connectStateToGoal(unsigned int position, ompl::control::PathControl &path, ompl::base::State* state, 
control::SpaceInformation* siC, ModifiedDirectedControlSamplerPtr sampler ) 
{
	
	return connectStates(position, path.getStates().size()-1, path,state,siC,sampler) ;	
}


bool ompl::control::PathControlOptimizer::connectStates(unsigned int initial, unsigned int goal , ompl::control::PathControl &path, ompl::base::State* state, control::SpaceInformation* siC, ModifiedDirectedControlSamplerPtr sampler) 
{
	ompl::base::State* reached_State = siC->allocState() ;
	return connectStates(initial, goal, path, state, siC, sampler, reached_State) ; 
}


void ompl::control::PathControlOptimizer::addIntermediaryStates( PathControl &path) 
{
	const base::SpaceInformationPtr &si = path.getSpaceInformation();
	ompl::control::SpaceInformation *siC = static_cast<ompl::control::SpaceInformation*>(si.get());	
	
	std::vector<base::State *> &states = path.getStates();
	std::vector<Control *> &controls = path.getControls() ;
	std::vector<double > &controlDurations = path.getControlDurations() ;
	
	std::vector<base::State *> newStates ;
	std::vector<Control *> newControls ;
	std::vector<double > newControlDurations ;
	
	unsigned int count = states.size() ;
	
	std::vector<base::State *> results  ;
	unsigned int cD ;
	newStates.push_back(states.at(0));
	
	for (unsigned int i= 0 ; i<count-1 ; ++i)
	{
		cD = siC->propagateWhileValid(states.at(i), controls.at(i), controlDurations.at(i),results,true) ;
		for (auto &state: results)
		{
			newStates.push_back(siC->cloneState(state));
			newControls.push_back(siC->cloneControl(controls.at(i)));
			newControlDurations.push_back(1) ;
		}
		siC->freeStates(results) ;
		
	}
	
	states.swap(newStates);
	controls.swap(newControls) ;
	controlDurations.swap(newControlDurations) ;
}
