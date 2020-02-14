#include <ompl/control/optimizers/PathControlOptimizer.h>


ompl::control::PathControlOptimizer::PathControlOptimizer(base::SpaceInformationPtr si, const base::OptimizationObjectivePtr& obj)
  : si_(si), obj_(obj), freeStates_(true)
{
}
void ompl::control::PathControlOptimizer::simplify(PathControl* path)
{
  OMPL_DEBUG("simplify");
	reduceVertices(*path,1,1);
  OMPL_DEBUG("done simplify");
	//path->subdivide() ;
}

/*
void ompl::control::PathControlOptimizer::reduceVertices(
PathControl &path, unsigned int maxSteps, unsigned int maxEmptySteps, double rangeRatio)
{
  //std::cout << "vertices reduced " << std::endl;
  if (path.getStateCount() < 3)
	return;
 
  if (maxSteps == 0)
    maxSteps = path.getStateCount();
 
  if (maxEmptySteps == 0)
	maxEmptySteps = path.getStateCount();
 
  bool result = false;
  unsigned int nochange = 0;
  const base::SpaceInformationPtr &si = path.getSpaceInformation();
  std::vector<base::State *> &states = path.getStates();
  ompl::control::SpaceInformation *siC1 = static_cast<ompl::control::SpaceInformation*>(si.get());
  siC1->setMotionValidator(std::make_shared<ompl::base::DynamicalMotionValidator>(siC1));
  siC1->setup();
  siC1->setMinMaxControlDuration(1,100);
  

 
  if (siC1->checkMotion(states.front(), states.back()))
  {
    std::cout << "vertices reduced " << std::endl; 
	if (freeStates_)
		for (std::size_t i = 2; i < states.size(); ++i)
			siC1->freeState(states[i - 1]);
    std::vector<base::State *> newStates(2);
    newStates[0] = states.front();
    newStates[1] = states.back();
	states.swap(newStates);
	result = true;
  }
  else
	for (unsigned int i = 0; i < maxSteps && nochange < maxEmptySteps; ++i, ++nochange)
    {

		int count = states.size();
		if (count<=10) 
		{
			rangeRatio= 0.99 ;
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
        std::cout << p1 << "<-->" << p2 << std::endl;
 
        if (p1 > p2)
			std::swap(p1, p2);
 
        if (siC1->checkMotion(states[p1], states[p2]))
        {
			if (freeStates_)
				for (int j = p1 + 1; j < p2; ++j)
					siC1->freeState(states[j]);
            states.erase(states.begin() + p1 + 1, states.begin() + p2);
			nochange = 0;
            result = true;
        }
    }
}
*/

void ompl::control::PathControlOptimizer::collapseCloseVertices(PathControl &path, unsigned int maxSteps, unsigned int maxEmptySteps)
{
	if (path.getStateCount() < 3)
		return ;
 
    if (maxSteps == 0)
        maxSteps = path.getStateCount();
 
    if (maxEmptySteps == 0)
        maxEmptySteps = path.getStateCount();
 
    const base::SpaceInformationPtr &si = path.getSpaceInformation();
    std::vector<base::State *> &states = path.getStates();
    ompl::control::SpaceInformation *siC = static_cast<ompl::control::SpaceInformation*>(si.get());
	siC->setMotionValidator(std::make_shared<ompl::base::DynamicalMotionValidator>(siC));
	siC->setup();
 
    // compute pair-wise distances in path (construct only half the matrix)
    std::map<std::pair<const base::State *, const base::State *>, double> distances;
    for (unsigned int i = 0; i < states.size(); ++i)
        for (unsigned int j = i + 2; j < states.size(); ++j)
			distances[std::make_pair(states[i], states[j])] = si->distance(states[i], states[j]);
 
    bool result = false;
    unsigned int nochange = 0;
    for (unsigned int s = 0; s < maxSteps && nochange < maxEmptySteps; ++s, ++nochange)
    {
        // find closest pair of points
        double minDist = std::numeric_limits<double>::infinity();
        int p1 = -1;
        int p2 = -1;
        for (unsigned int i = 0; i < states.size(); ++i)
            for (unsigned int j = i + 2; j < states.size(); ++j)
            {
                double d = distances[std::make_pair(states[i], states[j])];
                if (d < minDist)
                {
                    minDist = d;
                    p1 = i;
                    p2 = j;
                }
            }
 
        if (p1 >= 0 && p2 >= 0)
        {
            if (si->checkMotion(states[p1], states[p2]))
            {
                if (freeStates_)
                    for (int i = p1 + 1; i < p2; ++i)
                        si->freeState(states[i]);
                states.erase(states.begin() + p1 + 1, states.begin() + p2);
                result = true;
                nochange = 0;
            }
            else
                distances[std::make_pair(states[p1], states[p2])] = std::numeric_limits<double>::infinity();
        }
        else
            break;
    }
    return;
}

/*void ompl::control::PathControlOptimizer::subdivide(PathControl *path)
{
	if (path->states_.size() < 2)
		return;
    std::vector<base::State *> newStates(1, path->states_[0]);
    for (unsigned int i = 1; i < path->states_.size(); ++i)
    {
		base::State *temp = path->si_->allocState();
        si_->getStateSpace()->interpolate(newStates.back(), states_[i], 0.5, temp);
        newStates.push_back(temp);
        newStates.push_back(states_[i]);
    }
    path->states_.swap(newStates);
}

void ompl::control::PathControlOptimizer::smoothBSpline(PathControl &path, unsigned int maxSteps, double minChange)
{
	if (path.getStateCount() < 3)
		return;
 
    const base::SpaceInformationPtr &si = path.getSpaceInformation();
    std::vector<base::State *> &states = path.getStates();
 
    base::State *temp1 = si->allocState();
    base::State *temp2 = si->allocState();
 
    for (unsigned int s = 0; s < maxSteps; ++s)
    {
		subdivide();
 
        unsigned int i = 2, u = 0, n1 = states.size() - 1;
        while (i < n1)
        {
            if (si->isValid(states[i - 1]))
            {
                si->getStateSpace()->interpolate(states[i - 1], states[i], 0.5, temp1);
                si->getStateSpace()->interpolate(states[i], states[i + 1], 0.5, temp2);
                si->getStateSpace()->interpolate(temp1, temp2, 0.5, temp1);
                if (si->checkMotion(states[i - 1], temp1) && si->checkMotion(temp1, states[i + 1]))
                {
					if (si->distance(states[i], temp1) > minChange)
                    {
                        si->copyState(states[i], temp1);
                        ++u;
                    }
                }
            }
 
            i += 2;
        }
 
        if (u == 0)
            break;
    }
 
    si->freeState(temp1);
    si->freeState(temp2);
}
*/


void ompl::control::PathControlOptimizer::reduceVertices(PathControl &path, unsigned int maxSteps, unsigned int maxEmptySteps, double rangeRatio)
{
	std::cout << "vertices reduced " << std::endl;
	if (path.getStateCount() < 3)
		return;
 
	if (maxSteps == 0)
		maxSteps = path.getStateCount();
 
	if (maxEmptySteps == 0)
		maxEmptySteps = path.getStateCount();
 
	bool result = false;
	unsigned int nochange = 0;
	const base::SpaceInformationPtr &si = path.getSpaceInformation();
	std::vector<base::State *> &states = path.getStates();
	std::vector<Control *> &controls = path.getControls() ;
	std::vector<double > &controlDurations = path.getControlDurations() ;
	ompl::control::SpaceInformation *siC = static_cast<ompl::control::SpaceInformation*>(si.get());
	siC->setMotionValidator(std::make_shared<ompl::base::DynamicalMotionValidator>(siC));
	siC->setup();
	siC->setMinMaxControlDuration(1,50);
  
	std::vector<base::State *> newStates ;
	std::vector<control::Control *> newControls;
	std::vector<double> newControlDurations ;

  

	std::cout << " propagation size " << siC->getPropagationStepSize() << std::endl;
	std::cout << "initial size of states  " << states.size() << std::endl;
	std::cout << "initial size of controls  " << controls.size() << std::endl;
  

	if (siC->checkMotion(states.front(), states.back()))
	{
		if (freeStates_)
			for (std::size_t i = 2; i < states.size(); ++i)
				siC->freeState(states[i - 1]);
			for (std::size_t i = 1; i < controls.size()-1; ++i)
			{	
				siC->freeControl(controls[i]) ;
				controlDurations.erase(controlDurations.begin()+i) ;	
			}
		std::vector<base::State *> newStates(2);
		newControls[0] = siC->getCurrentControl();
		newControlDurations[0] = siC->getControlDuration() ;
		newStates[0] = states.front();
		newStates[1] = states.back();	
		states.swap(newStates);
		controls.swap(newControls) ;
		controlDurations.swap(controlDurations) ;    
		result = true;
	}
	else
	{
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
		
      if( p1 >= states.size() || p2 >= states.size()){
        OMPL_ERROR("p1 or p2 larger than states.");
        continue;
      }
      std::cout << "check motion" << std::endl;
			if (siC->checkMotion(states[p1], states[p2]))
			{
				std::cout << p1 <<"  <-->  " << p2 << std::endl;
				std::cout << "size of states  " << states.size() << std::endl;
				std::cout << "size of controls  " << controls.size() << std::endl;
				unsigned int s = controls.size();
				
        std::cout << "freestates" << std::endl;
				if (freeStates_)
				{
				
				
					for (int j = p1 + 1; j < p2; ++j)
						siC->freeState(states[j]);
				
					for (int i = p1 ; (i < p2) && (i < controls.size()) ; ++i)
						siC->freeControl(controls[i]);
				}
				
        std::cout << "states erase" << std::endl;
				states.erase(states.begin() + p1 + 1, states.begin() + p2);	
					
				if (p2<s)
				{
					controls.erase(controls.begin()+p1,controls.begin()+p2);
					controlDurations.erase(controlDurations.begin()+p1,controlDurations.begin()+p2) ;
				}
				else
				{
					controls.erase(controls.begin()+p1,controls.end());
					controlDurations.erase(controlDurations.begin()+p1,controlDurations.end()) ;
				}
				
        std::cout << "insert control" << std::endl;
        if( p1 < controls.size()){
          controls.insert(controls.begin()+p1, siC->getCurrentControl() ) ; 
          controlDurations.insert(controlDurations.begin()+p1, siC->getControlDuration() );
        }
        std::cout << "done insert control" << std::endl;
				
				//states.resize(states.size()-p2+p1+1);
				//controls.resize(states.size()-1);
				//controlDurations.resize(states.size()-1);
				
				//newControls.insert(newControls.end(), controls.begin(),controls.begin()+p1-1) ;
				//newControls.push_back(siC->getCurrentControl()) ;
				//newControls.insert(newControls.end(), controls.begin()+p2,controls.end()) ;
				
				//newControlDurations.insert(newControlDurations.end(), controlDurations.begin(),controlDurations.begin()+p1-1) ;
				//newControlDurations.push_back(siC->getControlDuration()) ;
				//newControlDurations.insert(newControlDurations.end(), controlDurations.begin()+p2,controlDurations.end()) ;		
				
				//controls.swap(newControls);
				//controlDurations.swap(newControlDurations) ;			
				



				nochange = 0;
				result = true;
			}
		}

	}
  

}

