#include <ompl/control/optimizers/PathControlOptimizer.h>


ompl::control::PathControlOptimizer::PathControlOptimizer(base::SpaceInformationPtr si, const base::OptimizationObjectivePtr& obj)
  : si_(si), obj_(obj), freeStates_(true)
{
}
void ompl::control::PathControlOptimizer::simplify(PathControl* path)
{
  OMPL_DEBUG("simplify");
	reduceVertices(*path,20,20,0.33);
  OMPL_DEBUG("done simplify");
	//path->subdivide() ;
}


// void ompl::control::PathControlOptimizer::collapseCloseVertices(PathControl &path, unsigned int maxSteps, unsigned int maxEmptySteps)
// {
// 	if (path.getStateCount() < 3)
// 		return ;
 
//     if (maxSteps == 0)
//         maxSteps = path.getStateCount();
 
//     if (maxEmptySteps == 0)
//         maxEmptySteps = path.getStateCount();
 
//     const base::SpaceInformationPtr &si = path.getSpaceInformation();
//     std::vector<base::State *> &states = path.getStates();
//     ompl::control::SpaceInformation *siC = static_cast<ompl::control::SpaceInformation*>(si.get());
// 	siC->setMotionValidator(std::make_shared<ompl::base::DynamicalMotionValidator>(siC));
// 	siC->setup();
// 	siC->setMinMaxControlDuration(1,50);
 
//     // compute pair-wise distances in path (construct only half the matrix)
//     std::map<std::pair<const base::State *, const base::State *>, double> distances;
//     for (unsigned int i = 0; i < states.size(); ++i)
//         for (unsigned int j = i + 2; j < states.size(); ++j)
// 			distances[std::make_pair(states[i], states[j])] = si->distance(states[i], states[j]);
 
//     unsigned int nochange = 0;
//     for (unsigned int s = 0; s < maxSteps && nochange < maxEmptySteps; ++s, ++nochange)
//     {
//         // find closest pair of points
//         double minDist = std::numeric_limits<double>::infinity();
//         int p1 = -1;
//         int p2 = -1;
//         for (unsigned int i = 0; i < states.size(); ++i)
//             for (unsigned int j = i + 2; j < states.size(); ++j)
//             {
//                 double d = distances[std::make_pair(states[i], states[j])];
//                 if (d < minDist)
//                 {
//                     minDist = d;
//                     p1 = i;
//                     p2 = j;
//                 }
//             }
 
//         if (p1 >= 0 && p2 >= 0)
//         {
//             if (si->checkMotion(states[p1], states[p2]))
//             {
//                 if (freeStates_)
//                     for (int i = p1 + 1; i < p2; ++i)
//                         si->freeState(states[i]);
//                 states.erase(states.begin() + p1 + 1, states.begin() + p2);
//                 result = true;
//                 nochange = 0;
//             }
//             else
//                 distances[std::make_pair(states[p1], states[p2])] = std::numeric_limits<double>::infinity();
//         }
//         else
//             break;
//     }
//     return;
// }


void ompl::control::PathControlOptimizer::reduceVertices(PathControl &path, unsigned int maxSteps, unsigned int maxEmptySteps, double rangeRatio)
{
	if (path.getStateCount() < 3)
		return;
 
	if (maxSteps == 0)
		maxSteps = path.getStateCount();
 
	if (maxEmptySteps == 0)
		maxEmptySteps = path.getStateCount();
 
	unsigned int nochange = 0;

	const base::SpaceInformationPtr &si = path.getSpaceInformation();

	std::vector<base::State *> &states = path.getStates();
	std::vector<Control *> &controls = path.getControls() ;
	std::vector<double > &controlDurations = path.getControlDurations() ;

	ompl::control::SpaceInformation *siC = static_cast<ompl::control::SpaceInformation*>(si.get());
	siC->setMotionValidator(std::make_shared<ompl::base::DynamicalMotionValidator>(siC));
	siC->setMinMaxControlDuration(1,50);
	
	ompl::control::SimpleDirectedControlSamplerPtr sampler;
	sampler = siC->allocSimpleDirectedControlSampler();
	sampler->setNumControlSamples(1000);
  
	std::cout << " propagation size " << siC->getPropagationStepSize() << std::endl;
	std::cout << "initial size of states  " << states.size() << std::endl;
	std::cout << "initial size of controls  " << controls.size() << std::endl;
  
	// if (siC->checkMotion(states.front(), states.back()))
	// {
	// 	if (freeStates_)
	// 		for (std::size_t i = 2; i < states.size(); ++i)
	// 			siC->freeState(states.at(i - 1));
	// 		for (std::size_t i = 1; i < controls.size()-1; ++i)
	// 		{	
	// 			siC->freeControl(controls.at(i)) ;
	// 			controlDurations.erase(controlDurations.begin()+i) ;	
	// 		}
	// 	std::vector<base::State *> newStates(2);
    // std::vector<control::Control *> newControls;
    // std::vector<double> newControlDurations;
	// 	newControls[0] = siC->getCurrentControl();
	// 	newControlDurations[0] = siC->getControlDuration() ;
	// 	newStates[0] = states.front();
	// 	newStates[1] = states.back();	
	// 	states.swap(newStates);
	// 	controls.swap(newControls) ;
	// 	controlDurations.swap(controlDurations) ;    
	// }
	// else
	// {

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

    ompl::control::Control *newControl =siC->allocControl() ;
    const base::State *stateP1 = states.at(p1);
    const base::State *stateP2 = states.at(p2);
    ompl::base::State *stateP2_tmp = siC->allocState();
    siC->copyState(stateP2_tmp, stateP2);
    
    //(1) sampleTo might reach state different from P2, and this state might be
    //reached using propagate while valid.
    double steps = sampler->sampleTo(newControl, stateP1, stateP2_tmp);
    double cD = siC->propagateWhileValid(stateP1, newControl, steps, stateP2_tmp);

    //Check that we reached P2
    const double d12 = siC->getStateSpace()->distance(stateP1, stateP2);
    const double targetRegion = 0.1 * d12;

    double distToTarget = siC->getStateSpace()->distance(stateP2_tmp, stateP2);

    std::cout << "Trying shortcut: " << p1 << "  <-->  " << p2 
      << " (dist=" << distToTarget << "<=" 
      << targetRegion << ")" << std::endl;

    if(distToTarget < targetRegion)
      OMPL_DEBUG("Reached P2");
     

    if((distToTarget <= targetRegion && (cD == steps))&&(connectConsecutiveStates(p2, path,stateP2_tmp,siC,sampler)))
    {
      if (freeStates_)
      {
        for (int j = p1 + 1; j < p2; j++)
        {
          siC->freeState(states.at(j));
          siC->freeControl(controls.at(j));
        }
      }
      std::cout << "States: " << states.size() << std::endl;
      states.erase(states.begin() + p1 + 1, states.begin() + p2);	
      std::cout << "States: " << states.size() << std::endl;

      controls.erase(controls.begin() + p1 + 1, controls.begin() + p2);	
      controlDurations.erase(controlDurations.begin() + p1 + 1, controlDurations.begin() + p2);	

      siC->copyControl(controls.at(p1), newControl);
      controlDurations.at(p1) = steps;

      nochange = 0;
      
      if (!path.check())
      {
        OMPL_ERROR("Path is invalid.");
        // exit(0);
      }
      
    }
    // siC->freeControl(newControl);

    
  }
}

bool ompl::control::PathControlOptimizer::connectConsecutiveStates(unsigned int position, ompl::control::PathControl &path, ompl::base::State* state , 
control::SpaceInformation* siC, SimpleDirectedControlSamplerPtr sampler)
{
	const PathControl path_old = PathControl(path) ;
	
	std::vector<base::State *> &states = path.getStates();
	std::vector<Control *> &controls = path.getControls() ;
	std::vector<double > &controlDurations = path.getControlDurations() ;

    ompl::control::Control *newControl =siC->allocControl() ;

	
	base::State *state1 = siC->allocState();
	base::State *state2 = siC->allocState();
	base::State *state2_tmp = siC->allocState();
	

    
	state1 = siC->cloneState(state);
	


	for (unsigned int i=position ; i<states.size()-1; ++i)
	{
		if (connectStateToGoal(i,path,state1,siC,sampler))
		{
			std::cout << "successfully connected state " <<i << " with goal state"  <<std::endl ;
			return true ;
		}
		
		state2 = siC->cloneState(states.at(i+1));
		siC->copyState(state2_tmp, state2);
		
		
		//(1) sampleTo might reach state different from P2, and this state might be
		//reached using propagate while valid.
		double steps = sampler->sampleTo(newControl, state1, state2_tmp);
		double cD = siC->propagateWhileValid(state1, newControl, steps, state2_tmp);

		//Check that we reached P2
		const double d12 = siC->getStateSpace()->distance(state1, state2);
		const double targetRegion = 0.5 * d12;

		double distToTarget = siC->getStateSpace()->distance(state2_tmp, state2);
		std::cout << "Trying to connect states: " << i << "  <-->  " << i+1 
			<< " (dist=" << distToTarget << "<=" 
			<< targetRegion << ")" << std::endl;
		
		if ((distToTarget > targetRegion) || (cD != steps)) 
		{
			std::cout << "State " << i<< " could not be reached" << std::endl;
			path = PathControl(path_old) ;
			return false ;
		}
		else 
		{
			siC->copyControl(controls.at(i), newControl);
			siC->copyState(states.at(i+1), state2_tmp);
			controlDurations.at(i)= steps ;
			state1 = siC->cloneState(states.at(i+1));
		}
	}
	return true ;
}

bool ompl::control::PathControlOptimizer::connectStateToGoal(unsigned int position, ompl::control::PathControl &path, ompl::base::State* state, 
control::SpaceInformation* siC, SimpleDirectedControlSamplerPtr sampler ) 
{
	const PathControl path_old = PathControl(path) ;
	
	
	std::vector<base::State *> &states = path.getStates();
	std::vector<Control *> &controls = path.getControls() ;
	std::vector<double > &controlDurations = path.getControlDurations() ;

    ompl::control::Control *newControl =siC->allocControl() ;
	
	base::State *state1 = siC->allocState();
	base::State *state2 = siC->allocState();
	base::State *state2_tmp = siC->allocState();
	siC->copyState(state1, state);
	siC->copyState(state2, states.back());
	
	double steps = sampler->sampleTo(newControl, state1, state2);
	double cD = siC->propagateWhileValid(state1, newControl, steps, state2_tmp);

	//Check that we reached the goal
	const double d12 = siC->getStateSpace()->distance(state1, state2);
	const double targetRegion = 0.1 * d12;

	double distToTarget = siC->getStateSpace()->distance(state2_tmp, state2);
	
	if ((distToTarget > targetRegion) || (cD != steps)) 
	{
		std::cout << "goal could not be reached from state" << position << std::endl;
		path = PathControl(path_old) ;
		return false ;
	}
	else 
	{
        for (int j = position + 1; j<states.size() -1 ; j++)
        {
          siC->freeState(states.at(j));
          siC->freeControl(controls.at(j));
        }	
        states.erase(states.begin() + position + 1, states.end() -1);	
        controls.erase(controls.begin() + position + 1, controls.end() -1);	
        controlDurations.erase(controlDurations.begin() + position + 1, controlDurations.end() -1);	
		siC->copyControl(controls.back() , newControl) ;
		controlDurations.back()= steps ;
		
	}
	
	return true ;	
}
