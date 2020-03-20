#include <ompl/geometric/planners/quotientspace/datastructures/graphsampler/GraphSampler.h>

ompl::geometric::BundleSpaceGraphSampler::BundleSpaceGraphSampler(BundleSpaceGraph* bundleSpaceGraph):
  bundleSpaceGraph_(bundleSpaceGraph)
{

    double mu = bundleSpaceGraph_->getBundle()->getMaximumExtent();
    epsilonGraphThickening_ = mu * 1e-2;
    OMPL_DEBUG("Epsilon Graph Thickening constant set to %f", epsilonGraphThickening_);
}

void ompl::geometric::BundleSpaceGraphSampler::sample(
    base::State *xRandom)
{
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    //EXP DECAY PATH BIAS.
    //from 1.0 down to lower limit
    //COMMENT FOR FIXED PATH BIAS
    const double exponentialDecayLowerLimit = 0.1;

    pathBias_ = (1.0-exponentialDecayLowerLimit) * exp(-exponentialDecayLambda_ * exponentialDecayCtr_++) + exponentialDecayLowerLimit;
    // OMPL_DEVMSG1("Path Bias %d", pathBias_);
    // std::cout << "path bias"<< pathBias_ << std::endl;

    double p = rng_.uniform01();
    if(p<pathBias_)
    {
        if(bundleSpaceGraph_->isDynamic()){
          std::cout << "TODO: NYI" << std::endl;
          throw "NYI";
        }

        geometric::PathGeometric &spath = static_cast<geometric::PathGeometric &>(*bundleSpaceGraph_->solutionPath_);
        std::vector<base::State*> states = spath.getStates();

        if(states.size() < 2){
            std::cout << std::string(80, '-') << std::endl;
            std::cout << "SOLUTION PATH HAS NO STATES" << std::endl;
            std::cout << std::string(80, '-') << std::endl;
            OMPL_ERROR("Solution path has no states (did you set it?)");
            throw "ZeroStates";
        }
        // std::cout << "Sampling on solution path w " << states.size() << "states" << std::endl;

        if(states.size() < 2){
            sampleImplementation(xRandom);
        }else{

            double totalLength = spath.length();
            double distStopping = rng_.uniform01() * totalLength;

            base::State *s1 = nullptr;
            base::State *s2 = nullptr;

            int ctr = 0;
            double distLastSegment = 0;
            double distCountedSegments = 0;
            while(distCountedSegments < distStopping && (ctr < (int)states.size()-1))
            {
                s1 = states.at(ctr);
                s2 = states.at(ctr+1);
                distLastSegment = bundle->distance(s1, s2);
                distCountedSegments += distLastSegment;
                ctr++;
            }

            //          |---- d -----|
            //---O------O------------O
            //|--------- t ----------|
            //|--------- s ------|
            //          |d-(t-s) |
            double step = (distLastSegment - (distCountedSegments - distStopping))/(distLastSegment);
            bundle->getStateSpace()->interpolate(s1, s2, step, xRandom);
        }

    }else{
        sampleImplementation(xRandom);
    }


    // Perturbate sample in epsilon neighborhood
    //  Note: Alternatively, we can use sampleGaussian (but seems to give similar
    //  results)
    if(epsilonGraphThickening_ > 0) 
    {
        bundleSpaceGraph_->getBundleSamplerPtr()->sampleUniformNear(xRandom, xRandom, epsilonGraphThickening_);
    }
}
