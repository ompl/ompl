#include <ompl/control/optimizers/PathControlOptimizer.h>


ompl::control::PathControlOptimizer::PathControlOptimizer(base::SpaceInformationPtr si, const base::OptimizationObjectivePtr& obj)
  : si_(si), obj_(obj), freeStates_(true)
{
}
void ompl::control::PathControlOptimizer::simplify(PathControl* path)
{
  OMPL_DEBUG("simplify");
	reduceVertices(*path,10,10,0.33);
  OMPL_DEBUG("done simplify");
	//path->subdivide() ;
}


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
	siC->setMinMaxControlDuration(1,50);
 
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
	
	ompl::control::SimpleDirectedControlSamplerPtr sampler;
	sampler = siC->allocSimpleDirectedControlSampler();
	sampler->setNumControlSamples(100);
	
  
	std::vector<base::State *> newStates ;
	std::vector<control::Control *> newControls;
	std::vector<double> newControlDurations ;
	ompl::control::Control *newControl =siC->allocControl() ;
	double newControlDuration ;


	std::cout << " propagation size " << siC->getPropagationStepSize() << std::endl;
	std::cout << "initial size of states  " << states.size() << std::endl;
	std::cout << "initial size of controls  " << controls.size() << std::endl;
  

	if (siC->checkMotion(states.front(), states.back()))
	{
		if (freeStates_)
			for (std::size_t i = 2; i < states.size(); ++i)
				siC->freeState(states.at(i - 1));
			for (std::size_t i = 1; i < controls.size()-1; ++i)
			{	
				siC->freeControl(controls.at(i)) ;
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
			ompl::control::Control *newControl =siC->allocControl() ;
			ompl::base::State *s1_temp = siC->allocState() ;
			ompl::base::State *s2_temp = siC->allocState() ;
			
			siC->copyState(s1_temp,states.at(p1)) ;
			siC->copyState(s2_temp,states.at(p2));
			std::cout <<" cloned the states" << std::endl;
			std::vector<base::State *> res ;
			double cD ;
			
			
			newControlDuration=sampler->sampleTo(newControl, s1_temp , s2_temp) ;
			//std::cout << "number of steps before propagation " << newControlDuration << std::endl;
			cD =siC->propagateWhileValid(states.at(p1),newControl,newControlDuration,res, true) ;
			//std::cout << "number of steps after propagation " << cD << std::endl;
			
			if (cD!=newControlDuration)
			{
				std::cout <<" Propagation not valid " << std::endl;
				continue ;
			}
			siC->freeState(s1_temp) ;
			siC->freeState(s2_temp) ;
			
			
			std::cout << p1 <<"  <-->  " << p2 << std::endl;
			std::cout << "size of states  " << states.size() << std::endl;
			std::cout << "size of controls  " << controls.size() << std::endl;
			unsigned int s = controls.size();
			
			if (cD==newControlDuration)
			{
		std::cout << "freestates" << std::endl;
				if (freeStates_)
				{
				
				
					for (int j = p1 + 1; j < p2; ++j)
						siC->freeState(states.at(j));
				
					//for (int l = p1 ; (l < p2) && (l < controls.size()) ; ++l)
						//siC->freeControl(controls.at(l));
				}
				
		std::cout << "states erase" << std::endl;
				states.erase(states.begin() + p1 + 1, states.begin() + p2);	
				
				newControls.clear() ;
				newControlDurations.clear() ;
		std::cout << "cleared newcontrols" << std::endl;		
				
				for (int k = 0 ; k< controls.size(); ++k )
				{
					if (k<p1)
					{
		std::cout << "added control n° " << k << std::endl;				
						newControls.push_back(siC->cloneControl(controls.at(k)));
						newControlDurations.push_back(controlDurations.at(k));
						
					}
					if (k==p1)
					{
						//siC->copyControl(newControl, siC->getCurrentControl() );
						//newControls.push_back(siC->cloneControl(siC->getCurrentControl() ));
						//newControlDurations.push_back(siC->getControlDuration());
						newControls.push_back(siC->cloneControl(newControl));
						newControlDurations.push_back(newControlDuration);
		std::cout << "added control at p1 n° " << k << std::endl;					
						//siC->freeControl(controls.at(k));
						
		//std::cout << "freed control at n° " << k << std::endl;					
											
					}
					if (k>=p2)
					{
						newControls.push_back(siC->cloneControl(controls.at(k)));
						newControlDurations.push_back(controlDurations.at(k));	
		std::cout << "added control n° " << k << std::endl;										
					}			
															
				}
				siC->freeControl(newControl) ;
				controls.swap(newControls) ;
				controlDurations.swap(newControlDurations) ;

				nochange = 0;
				result = true; 
				
				if (!path.check())
					std::cout << "something is wrong " << std::endl;
				
			}
			
		}

	}
  

}

