#include <ompl/geometric/planners/multilevel/datastructures/graphsampler/GraphSampler.h>

ompl::geometric::BundleSpaceGraphSampler::BundleSpaceGraphSampler(BundleSpaceGraph* bundleSpaceGraph):
  bundleSpaceGraph_(bundleSpaceGraph)
{
    double mu = bundleSpaceGraph_->getBundle()->getMaximumExtent();
    epsilonGraphThickening_ = mu * epsilonGraphThickeningFraction_;
    OMPL_DEBUG("Epsilon Graph Thickening constant set to %f", epsilonGraphThickening_);

    pathBiasDecay_.setLambda(exponentialDecayLambda_);
    pathBiasDecay_.setLowerBound(pathBiasFixed_);

    pathThickeningGrowth_.setLambda(exponentialDecayLambda_);
    pathThickeningGrowth_.setLowerBound(epsilonGraphThickening_);
    pathThickeningGrowth_.setUpperBound(0.0);

    graphThickeningGrowth_.setLambda(exponentialDecayLambda_);
    graphThickeningGrowth_.setLowerBound(epsilonGraphThickening_);
    graphThickeningGrowth_.setUpperBound(0.0);
}

void ompl::geometric::BundleSpaceGraphSampler::reset()
{
    double mu = bundleSpaceGraph_->getBundle()->getMaximumExtent();
    epsilonGraphThickening_ = mu * epsilonGraphThickeningFraction_;

    pathBiasDecay_.reset();
    pathThickeningGrowth_.reset();
    graphThickeningGrowth_.reset();
}

void ompl::geometric::BundleSpaceGraphSampler::setPathBiasStartSegment(double s)
{
  if(s > this->pathBiasStartSegment_)
  {
      this->pathBiasStartSegment_ = s;
      geometric::PathGeometric &spath = 
        static_cast<geometric::PathGeometric &>(*bundleSpaceGraph_->solutionPath_);
      OMPL_DEBUG("Set path bias: %f/%f", s, spath.length());
  }
}

double ompl::geometric::BundleSpaceGraphSampler::getPathBiasStartSegment()
{
  return this->pathBiasStartSegment_;
}

void ompl::geometric::BundleSpaceGraphSampler::sample(
    base::State *xRandom)
{
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    //EXP DECAY PATH BIAS.
    //from 1.0 down to lower limit pathbiasfixed_
    // const double pathBias = 
    //   (1.0 - pathBiasFixed_) * exp(-exponentialDecayLambda_ * counterPathSampling_++) 
    //   + pathBiasFixed_;

    double p = rng_.uniform01();
    if(p < pathBiasDecay_() && !bundleSpaceGraph_->isDynamic())
    {
        geometric::PathGeometric &spath = 
          static_cast<geometric::PathGeometric &>(*bundleSpaceGraph_->solutionPath_);
        std::vector<base::State*> states = spath.getStates();

        if(states.size() < 2){
            //empty solution returned, cannot use path bias sampling
            sampleImplementation(xRandom);
        }else{
            //First one works well for SE3->R3, second one works best for
            //SE3^k->SE3^{k-1}
            // double endLength = std::min( pathBiasStartSegment_ + 0.1*spath.length(),
            //     spath.length());


            //TODO: outsource to bundlegraph?
            double endLength = spath.length();
            double distStopping = 
              pathBiasStartSegment_ + rng_.uniform01() * (endLength - pathBiasStartSegment_);

            // std::cout << "pathSampling:" << distStopping << "/" << totalLength 
            //   << " biasSegment:" << pathBiasStartSegment_ << "." << std::endl;

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

            if(epsilonGraphThickening_ > 0) 
            {
                double eps = pathThickeningGrowth_();
                bundleSpaceGraph_->getBundleSamplerPtr()->sampleUniformNear(
                    xRandom, xRandom, eps);
            }

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
        double eps = graphThickeningGrowth_();
        bundleSpaceGraph_->getBundleSamplerPtr()->sampleUniformNear(xRandom, xRandom, 
            eps);
    }
}
