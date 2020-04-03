#include <ompl/geometric/planners/quotientspace/datastructures/graphsampler/GraphSampler.h>

ompl::geometric::BundleSpaceGraphSampler::BundleSpaceGraphSampler(BundleSpaceGraph* bundleSpaceGraph):
  bundleSpaceGraph_(bundleSpaceGraph)
{
    double mu = bundleSpaceGraph_->getBundle()->getMaximumExtent();
    epsilonGraphThickening_ = mu * epsilonGraphThickeningFraction_;
    OMPL_DEBUG("Epsilon Graph Thickening constant set to %f", epsilonGraphThickening_);
}

void ompl::geometric::BundleSpaceGraphSampler::sample(
    base::State *xRandom)
{
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    //EXP DECAY PATH BIAS.
    //from 1.0 down to lower limit pathbiasfixed_
    const double pathBias = 
      (1.0 - pathBiasFixed_) * exp(-exponentialDecayLambda_ * counterPathSampling_++) 
      + pathBiasFixed_;

    double p = rng_.uniform01();
    if(p < pathBias && !bundleSpaceGraph_->isDynamic())
    {
        geometric::PathGeometric &spath = 
          static_cast<geometric::PathGeometric &>(*bundleSpaceGraph_->solutionPath_);
        std::vector<base::State*> states = spath.getStates();

        if(states.size() < 2){
            //empty solution returned, cannot use path bias sampling
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
        //Decay on graph thickening (reflects or believe in the usefullness of
        //the graph for biasing our sampling)
        double graphBias = 
          (- epsilonGraphThickening_) * exp(-exponentialDecayLambda_ * counterGraphSampling_++) 
          + epsilonGraphThickening_;
        bundleSpaceGraph_->getBundleSamplerPtr()->sampleUniformNear(xRandom, xRandom, graphBias);

        //TODO: what to do when we encounter an integer wrap around?
        // -- Currently we do nothing, so periodically we will see a spike in
        // graphbias/pathbias, which might be OK (i.e. if we get stuck, maybe go
        // back and test some more on the path or the graph
    }
}
