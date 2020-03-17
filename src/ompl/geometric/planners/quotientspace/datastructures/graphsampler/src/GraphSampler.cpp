#include <ompl/geometric/planners/quotientspace/datastructures/graphsampler/GraphSampler.h>

ompl::geometric::BundleSpaceGraphSampler::BundleSpaceGraphSampler(BundleSpaceGraph* bundleSpaceGraph):
  bundleSpaceGraph_(bundleSpaceGraph)
{
}

void ompl::geometric::BundleSpaceGraphSampler::sample(
    base::State *xRandom)
{
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    //EXP DECAY PATH BIAS.
    //from 1.0 down to lower limit
    //COMMENT FOR FIXED PATH BIAS
    double exponentialDecayLowerLimit = 0.1;
    pathBias_ = (1.0-exponentialDecayLowerLimit) * exp(-exponentialDecayLambda_ * pow(((double)exponentialDecayCtr_++/1000.0),2)) + exponentialDecayLowerLimit;
    OMPL_DEVMSG1("Path Bias %d", pathBias_);

    double p = rng_.uniform01();
    if(p<pathBias_)
    {
        if(bundleSpaceGraph_->isDynamic()){
          std::cout << "NYI" << std::endl;
          throw "NYI";
        }

        geometric::PathGeometric &spath = static_cast<geometric::PathGeometric &>(*bundleSpaceGraph_->solutionPath_);
        std::vector<base::State*> states = spath.getStates();

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

        //############################################################################
        // DEPRECATED
        // while(t < s && (ctr < (int)bundleSpaceGraph_->startGoalVertexPath_.size()-1))
        // {
        //     t += bundleSpaceGraph_->lengthsStartGoalVertexPath_.at(ctr);
        //     ctr++;
        // }
        // const Vertex v1 = bundleSpaceGraph_->startGoalVertexPath_.at(ctr-1);
        // const Vertex v2 = bundleSpaceGraph_->startGoalVertexPath_.at(ctr);
        // double d = bundleSpaceGraph_->lengthsStartGoalVertexPath_.at(ctr-1);
        ////          |---- d -----|
        ////---O------O------------O
        ////|--------- t ----------|
        ////|--------- s ------|
        ////          |d-(t-s) |
        //double step = (d - (t - s))/(d);
        //BundleSpaceGraph::Graph graph = bundleSpaceGraph_->getGraph();
        //bundleSpaceGraph_->getBundle()->getStateSpace()->interpolate(graph[v1]->state, graph[v2]->state, step, xRandom);
        //############################################################################


    }else{
        sampleImplementation(xRandom);
    }



    if(epsilonGraphThickening_ > 0) 
    {
        bundleSpaceGraph_->getBundleSamplerPtr()->sampleUniformNear(xRandom, xRandom, epsilonGraphThickening_);
    }



    //double p = rng_.uniform01();
    //if(lengthStartGoalVertexPath_ > 0 && p < pathBias_)
    //{
    //    //(1) Sample randomly on shortest path
    //    double p = rng_.uniform01() * lengthStartGoalVertexPath_;

    //    double t = 0;
    //    int ctr = 0;
    //    while(t < p && (ctr < (int)startGoalVertexPath_.size()-1))
    //    {
    //        t += lengthsStartGoalVertexPath_.at(ctr);
    //        ctr++;
    //    }
    //    const Vertex v1 = startGoalVertexPath_.at(ctr-1);
    //    const Vertex v2 = startGoalVertexPath_.at(ctr);
    //    double d = lengthsStartGoalVertexPath_.at(ctr-1);


    //    //          |---- d -----|
    //    //---O------O------------O
    //    //|--------- t ----------|
    //    //|--------- p ------|
    //    //          |d-(t-p) |
    //    double s = (d - (t - p))/(d);
    //    getBundle()->getStateSpace()->interpolate(graph_[v1]->state, graph_[v2]->state, s, xRandom);

    //}else{
    //    //(2) Sample randomly on graph
    //    BaseT::sampleFromDatastructure(xRandom);
    //}

    ////(3) Perturbate sample in epsilon neighborhood
}
