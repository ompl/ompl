#include <ompl/multilevel/planners/explorer/algorithms/PathSpaceSparseOptimization.h>
#include <ompl/multilevel/planners/explorer/datastructures/PathVisibilityChecker.h>
#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/PDF.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/Control.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <ompl/control/StatePropagator.h>

#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace ompl::multilevel;
using namespace ompl::base;

PathSpaceSparseOptimization::PathSpaceSparseOptimization(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : PathSpace(this), BaseT(si, parent_)
{
    setName("BundleSpaceExplorer" + std::to_string(id_));
    Planner::declareParam<double>("range", this, &PathSpaceSparseOptimization::setRange,
                                  &PathSpaceSparseOptimization::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &PathSpaceSparseOptimization::setGoalBias,
                                  &PathSpaceSparseOptimization::getGoalBias, "0.:.1:1.");

    specs_.approximateSolutions = true;
    approximateDistanceToGoal = std::numeric_limits<double>::infinity();

    if (isDynamic())
    {
        ompl::control::SpaceInformation *siC = dynamic_cast<ompl::control::SpaceInformation *>(si.get());
        siC->setup();
        siC->setMinMaxControlDuration(1, controlDuration);
        std::cout << "MaxControlDuration: " << controlDuration << std::endl;

        dCSampler = siC->allocDirectedControlSampler();
        //@TODO: need to write allocator for simpleDirectedControlSampler
        // dCSampler->setNumControlSamples(numberOfControlSamples);
        propStepSize = siC->getPropagationStepSize();
        prop = siC->getStatePropagator();
        c_random = siC->allocControl();
    }

    pathVisibilityChecker_ = new PathVisibilityChecker(getBundle());

    double maxExt = getBundle()->getMaximumExtent();
    pathBias_ = pathBiasFraction_ * maxExt;

    base::OptimizationObjectivePtr lengthObj(new base::PathLengthOptimizationObjective(getBundle()));
    base::OptimizationObjectivePtr clearObj(new base::MaximizeMinClearanceObjective(getBundle()));

    pathObj_ = std::make_shared<ompl::base::MultiOptimizationObjective>(getBundle());

    std::static_pointer_cast<base::MultiOptimizationObjective>(pathObj_)->addObjective(lengthObj, 0.5);
    // std::static_pointer_cast<base::MultiOptimizationObjective>(pathObj_)->addObjective(clearObj, 0.5);
}

PathSpaceSparseOptimization::~PathSpaceSparseOptimization()
{
    delete pathVisibilityChecker_;
}

void PathSpaceSparseOptimization::setGoalBias(double goalBias_)
{
    goalBias = goalBias_;
}
double PathSpaceSparseOptimization::getGoalBias() const
{
    return goalBias;
}
void PathSpaceSparseOptimization::setRange(double maxDistance_)
{
    maxDistance = maxDistance_;
}
double PathSpaceSparseOptimization::getRange() const
{
    return maxDistance;
}

void PathSpaceSparseOptimization::setup()
{
    BaseT::setup();
    ompl::tools::SelfConfig sc(getBundle(), getName());
    sc.configurePlannerRange(maxDistance);
}

void PathSpaceSparseOptimization::clear()
{
    BaseT::clear();
    pathStackHead_.clear();
    pathStack_.clear();
}

bool PathSpaceSparseOptimization::getSolution(base::PathPtr &solution)
{
    if (hasSolution_)
    {
        if (!isDynamic())
            return BaseT::getSolution(solution);
        else
        {
            const Configuration *q_nearest_to_goal = nearest(qGoal_);
            solution = getPath(qStart_->index, q_nearest_to_goal->index);
            return true;
        }
    }
    else
    {
        return false;
    }
}

void PathSpaceSparseOptimization::grow()
{
    if (firstRun_)
    {
        init();
        firstRun_ = false;
    }
    if (isDynamic())
    {
        if (hasSolution_ && getNumberOfVertices() <= 1)
        {
            hasSolution_ = false;
        }
    }
    if (hasSolution_ && pathStackHead_.size() > 0)
    {
        // No Goal Biasing if we already found a solution on this quotient space
        sampleBundle(xRandom_->state);
    }
    else
    {
        double s = rng_.uniform01();
        if (s < goalBias)
        {
            // sets xRandom_ as qGoal_
            getBundle()->copyState(xRandom_->state, qGoal_->state);
        }
        else
        {
            sampleBundle(xRandom_->state);
        }
    }
    if (isDynamic())
    {
        growControl();
    }
    else
    {
        growGeometric();
    }
}

bool PathSpaceSparseOptimization::hasSolution()
{
    return BaseT::hasSolution();
}

// void PathSpaceSparseOptimization::growGeometricExpand()
// {
//     PDF pdf;
//     foreach (Vertex v, boost::vertices(graph_))
//     {
//         const unsigned long int t = graph_[v]->total_connection_attempts;
//         pdf.add(graph_[v], (double)(t - graph_[v]->successful_connection_attempts) / (double)t);
//     }
//     if (pdf.empty())
//         return;

//     std::vector<base::State *> workStates(5);
//     getBundle()->allocStates(workStates);
//     Configuration *qv = pdf.sample(rng_.uniform01());
//     unsigned int s = getBundle()->randomBounceMotion(Bundle_sampler_, qv->state, workStates.size(), workStates,
//     false);

//     if (s > 0)
//     {
//         s--;
//         Configuration *qn = new Configuration(getBundle(), workStates[s]);
//         Vertex last = addConfiguration(qn);

//         for (unsigned int i = 0; i < s; ++i)
//         {
//             // add the vertex along the bouncing motion
//             Configuration *qs = new Configuration(getBundle(), workStates[i]);
//             Vertex m = addConfiguration(qs);

//             addEdge(qn->index, m);
//             qn = qs;
//         }

//         if (s > 0 || !sameComponent(qv->index, last))
//         {
//             addEdge(qn->index, last);
//         }
//     }
//     getBundle()->freeStates(workStates);
// }

void PathSpaceSparseOptimization::growGeometric()
{
    const Configuration *q_nearest = nearest(xRandom_);

    double d = getBundle()->distance(q_nearest->state, xRandom_->state);
    if (d > maxDistance)
    {
        getBundle()->getStateSpace()->interpolate(q_nearest->state, xRandom_->state, maxDistance / d, xRandom_->state);
    }

    if (getBundle()->checkMotion(q_nearest->state, xRandom_->state))
    {
        Configuration *q_next = new Configuration(getBundle(), xRandom_->state);

        Vertex v_next = addConfigurationConditional(q_next);

        addEdge(q_nearest->index, v_next);

        if (!hasSolution_)
        {
            bool satisfied = sameComponent(getStartIndex(), getGoalIndex());
            if (satisfied)
            {
                hasSolution_ = true;
            }
        }
    }
}

void PathSpaceSparseOptimization::growControl()
{
    const Configuration *q_nearest = nearest(xRandom_);

    s_random = xRandom_->state;

    int duration = dCSampler->sampleTo(c_random, q_nearest->state, s_random);

    if (duration > controlDuration)
    {
        // truncate motion
        prop->propagate(q_nearest->state, c_random, duration, s_random);
    }

    Configuration *q_next = new Configuration(getBundle(), s_random);
    Vertex v_next = addConfiguration(q_next);
    addEdge(q_nearest->index, v_next);

    if (!hasSolution_)
    {
        const Configuration *q_nearest_to_goal = nearest(qGoal_);
        goal_->isSatisfied(q_nearest_to_goal->state, &distanceToGoal);
        // if(hasSolution_){
        //   std::cout << "Found solution " << distanceToGoal << " away from goal." << std::endl;
        // }
        if (distanceToGoal < approximateDistanceToGoal)
        {
            approximateDistanceToGoal = distanceToGoal;
            // std::cout << "Found new solution " << distanceToGoal << " away from goal." << std::endl;
            Configuration *qStartSparse = getGraph()[getGoalIndex()];
            // getBundle()->printState(qStartSparse->state);
            // getBundle()->printState(q_nearest_to_goal->state);
            // std::cout << q_nearest_to_goal->index << std::endl;
            // std::cout << qStartSparse->index << std::endl;

            if (approximateDistanceToGoal < 2)
            {
                base::PathPtr path = getPath(qStartSparse->index, q_nearest_to_goal->index);
                if (path != nullptr)
                {
                    hasSolution_ = true;
                }
            }
        }
    }
}

void PathSpaceSparseOptimization::getPlannerData(base::PlannerData &data) const
{
    if (isDynamic())
    {
        if (!data.hasControls())
        {
            OMPL_ERROR("Dynamic Cspace, but PlannerData has no controls.");
        }
    }
    if (pathStackHead_.size() > 0)
    {
        OMPL_DEVMSG1("%s has %d solutions.", getName().c_str(), pathStackHead_.size());
        if (pathStackHead_.empty())
        {
            OMPL_ERROR("%s has 0 solutions.", getName().c_str());
            throw ompl::Exception("Zero solutions");
        }
        std::vector<int> idxPathI;
        for (uint i = 0; i < pathStackHead_.size(); i++)
        {
            const std::vector<base::State *> states = pathStackHead_.at(i);

            std::cout << pathStackHead_.at(i).size() << std::endl;

            // idxPathI.clear();
            // getPathIndices(states, idxPathI);
            // std::reverse(idxPathI.begin(), idxPathI.end());
            // idxPathI.push_back(i);
            // idxPathI.insert(idxPathI.begin(), idxPathI.rbegin(), idxPathI.rend());

            //############################################################################
            // DEBUG
            std::cout << "[";
            for (uint k = 0; k < idxPathI.size(); k++)
            {
                std::cout << idxPathI.at(k) << " ";
            }
            std::cout << "]" << std::endl;
            //############################################################################

            multilevel::PlannerDataVertexAnnotated *p1 = new multilevel::PlannerDataVertexAnnotated(states.at(0));
            p1->setLevel(getLevel());
            // p1->setPath(idxPathI);
            data.addStartVertex(*p1);

            double dk = 0;
            for (uint k = 0; k < states.size() - 1; k++)
            {
                dk += getBundle()->distance(states.at(k), states.at(k + 1));

                multilevel::PlannerDataVertexAnnotated *p2 =
                    new multilevel::PlannerDataVertexAnnotated(states.at(k + 1));
                p2->setLevel(getLevel());
                // p2->setPath(idxPathI);

                if (k == states.size() - 2)
                {
                    data.addGoalVertex(*p2);
                }
                else
                {
                    data.addVertex(*p2);
                }
                data.addEdge(*p1, *p2);

                p1 = p2;
            }
            std::cout << "Cost: " << dk << std::endl;

            // getBundle()->printState(states.back());
        }
        // getPlannerDataRoadmap(data, idxPathI);
    }
    else
    {
        OMPL_DEVMSG1("Sparse Roadmap has %d/%d vertices/edges.", 
            getNumberOfVertices(),
            getNumberOfEdges());
    }
}
//############################################################################
// EXPLORER
//############################################################################
void PathSpaceSparseOptimization::sampleFromDatastructure(base::State *xRandom_graph)
{
    if (!getTotalBundleSpace()->isDynamic() && pathStack_.size() > 0)
    {
        // if (selectedPath_ >= 0 && selectedPath_ < (int)pathStack_.size())
        // {
        const std::vector<base::State *> &states = localMinimaTree_->getSelectedMinimumAsStateVector(getLevel());

        uint N = states.size();

        //############################################################################
        // Vertex Sampling
        // int k = rng_.uniformInt(0, N-1);
        // base::State *state = states.at(k);
        // getBundle()->getStateSpace()->copyState(xRandom_graph, state);
        // Bundle_sampler->sampleUniformNear(xRandom_graph, xRandom_graph, 0.2);

        //############################################################################
        // Edge Sampling
        uint k = rng_.uniformInt(0, N - 1);
        double r = rng_.uniform01();
        base::State *s1 = states.at((k < N - 1) ? k : k - 1);
        base::State *s2 = states.at((k < N - 1) ? k + 1 : k);

        getBundle()->getStateSpace()->interpolate(s1, s2, r, xRandom_graph);

        getBundleSamplerPtr()->sampleUniformNear(xRandom_graph, xRandom_graph, pathBias_);
        // }
        // else
        // {
        //     std::cout << "Level:" << getLevel() << std::endl;
        //     OMPL_ERROR("Selected path is %d/%d (have you selected a path?)", selectedPath_, pathStack_.size());
        //     throw ompl::Exception("Unknown selected path");
        // }
    }
    else
    {
        BaseT::sampleFromDatastructure(xRandom_graph);
    }
}

unsigned int PathSpaceSparseOptimization::getNumberOfPaths() const
{
    return pathStackHead_.size();
}

// void PathSpaceSparseOptimization::removeLastPathFromStack()
// {
//     pathStackHead_.erase(pathStackHead_.end() - 1);
// }

void PathSpaceSparseOptimization::pushPathToStack(std::vector<base::State *> &path)
{
    if (isDynamic())
    {
        OMPL_ERROR("NYI");
        throw Exception("NYI");
        // control::PathControl cpath(getBundle());
        // for (uint k = 0; k < path.size(); k++)
        // {
        //     cpath.append(path.at(k));
        // }
        // // shortcutter.shortcutPath(gpath)
        // // control::PathControl* cpath = static_cast<control::PathControl*>(path_.get());
        // // base::State* goalState = si_->allocState();
        // // std::static_pointer_cast<const base::GoalSampleableRegion> (pdef_->getGoal())->sampleGoal(goalState) ;

        // control::PathControlOptimizer control_simplifier(getBundle(), qGoal_->state);
        // std::cout << "Dynamic Optimize" << std::endl;
        // control_simplifier.simplify(&cpath);

        // // for (uint k = 0; k < pathStack_.size(); k++)
        // // {
        // //     control::PathControl &pathk = pathStack_.at(k);
        // //     if (getPathVisibilityChecker()->IsPathVisible(gpath.getStates(), pathk.getStates()))
        // //     {
        // //         std::cout << "REJECTED (Equal to path " << k << ")" << std::endl;
        // //         numberOfFailedAddingPathCalls++;
        // //         return;
        // //     }
        // // }
        // // pathStack_.push_back(gpath);
    }
    else
    {
        geometric::PathGeometric gpath(getBundle());
        for (uint k = 0; k < path.size(); k++)
        {
            gpath.append(path.at(k));
        }
        geometric::PathSimplifier shortcutter(getBundle(), base::GoalPtr(), pathObj_);
        // make sure that we have enough vertices so that the right path class is
        // visualized (problems with S1)
        if (getBundle()->getStateSpace()->getType() == base::STATE_SPACE_SO2)
        {
            std::cout << "SO2 INTERPOLATE" << std::endl;
            gpath.interpolate();
        }
        else
        {
            shortcutter.smoothBSpline(gpath);
            shortcutter.simplifyMax(gpath);
        }
        if (!isProjectable(gpath.getStates()))
        {
            std::cout << "REJECTED (Not projectable)" << std::endl;
            numberOfFailedAddingPathCalls++;
            return;
        }

        if (!pathVisibilityChecker_->CheckValidity(gpath.getStates()))
        {
            std::cout << "REJECTED (Infeasible)" << std::endl;
            numberOfFailedAddingPathCalls++;
            return;
        }
        if (pathStack_.size() <= 0)
        {
            pathStack_.push_back(gpath);
        }
        else
        {
            for (uint k = 0; k < pathStack_.size(); k++)
            {
                geometric::PathGeometric &pathk = pathStack_.at(k);
                if (pathVisibilityChecker_->IsPathVisible(gpath.getStates(), pathk.getStates()))
                {
                    std::cout << "REJECTED (Equal to path " << k << ")" << std::endl;
                    numberOfFailedAddingPathCalls++;
                    return;
                }
            }
            pathStack_.push_back(gpath);
        }
        std::cout << "Added to stack (" << pathStack_.size() << " paths on stack)" << std::endl;
    }
}

void PathSpaceSparseOptimization::PrintPathStack()
{
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Path Stack" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    for (uint k = 0; k < pathStack_.size(); k++)
    {
        std::vector<base::State *> pathk = pathStack_.at(k).getStates();
        for (uint j = 0; j < pathk.size(); j++)
        {
            getBundle()->printState(pathk.at(j));
        }
    }
}

void PathSpaceSparseOptimization::removeEdgeIfReductionLoop(const Edge &e)
{
    const Vertex v1 = boost::source(e, getGraph());
    const Vertex v2 = boost::target(e, getGraph());

    //############################################################################
    // (2) Get common neighbors of v1,v2
    std::vector<Vertex> v1_neighbors;
    std::vector<Vertex> v2_neighbors;
    std::vector<Vertex> common_neighbors;

    OEIterator edge_iter, edge_iter_end, next;

    boost::tie(edge_iter, edge_iter_end) = boost::out_edges(v1, getGraph());
    for (next = edge_iter; edge_iter != edge_iter_end; edge_iter = next)
    {
        const Vertex v_target = boost::target(*edge_iter, getGraph());
        if (v_target != v2)
            v1_neighbors.push_back(v_target);
        ++next;
    }
    boost::tie(edge_iter, edge_iter_end) = boost::out_edges(v2, getGraph());
    for (next = edge_iter; edge_iter != edge_iter_end; edge_iter = next)
    {
        const Vertex v_target = boost::target(*edge_iter, getGraph());
        if (v_target != v1)
            v2_neighbors.push_back(v_target);
        ++next;
    }

    for (uint k = 0; k < v1_neighbors.size(); k++)
    {
        for (uint j = 0; j < v2_neighbors.size(); j++)
        {
            const Vertex v1k = v1_neighbors.at(k);
            const Vertex v2k = v2_neighbors.at(j);
            if (v1k == v2k)
            {
                common_neighbors.push_back(v1k);
            }
        }
    }

    // rm duplicates
    std::sort(common_neighbors.begin(), common_neighbors.end());
    auto last = std::unique(common_neighbors.begin(), common_neighbors.end());
    common_neighbors.erase(last, common_neighbors.end());

    //############################################################################
    //  (3) Check if face (v1, v2, v3) is feasible
    for (uint k = 0; k < common_neighbors.size(); k++)
    {
        const Vertex v3 = common_neighbors.at(k);
        std::vector<Vertex> vpath1;
        vpath1.push_back(v1);
        vpath1.push_back(v3);
        vpath1.push_back(v2);
        std::vector<Vertex> vpath2;
        vpath2.push_back(v1);
        vpath2.push_back(v2);

        if (pathVisibilityChecker_->IsPathVisible(vpath1, vpath2, getGraphNonConst()))
        {
            // RemoveEdge
            std::cout << "Removing Edge " << v1 << "<->" << v2 << std::endl;
            boost::remove_edge(v1, v2, getGraphNonConst());
        }
    }
}
void PathSpaceSparseOptimization::removeReducibleLoops()
{
    // Edge e = boost::random_edge(getGraph(), rng_boost);
    uint Mend = getNumberOfEdges();
    for (uint k = 0; k < Mend; k++)
    {
        Edge e = boost::random_edge(getGraph(), rng_boost);
        removeEdgeIfReductionLoop(e);
    }
}

void PathSpaceSparseOptimization::freePath(std::vector<base::State *> path, const base::SpaceInformationPtr &si) const
{
    for (uint k = 0; k < path.size(); k++)
    {
        si->freeState(path.at(k));
    }
    path.clear();
}

std::vector<ompl::base::State *> PathSpaceSparseOptimization::getProjectedPath(
    const std::vector<base::State *> pathBundle, const base::SpaceInformationPtr &) const
{
    std::vector<base::State *> pathBase;
    for (uint k = 0; k < pathBundle.size(); k++)
    {
        const base::State *qk = pathBundle.at(k);
        base::State *qkProjected = getBase()->allocState();
        projectBase(qk, qkProjected);
        pathBase.push_back(qkProjected);
    }
    return pathBase;
}

bool PathSpaceSparseOptimization::isProjectable(const std::vector<base::State *> &pathBundle) const
{
    return (getProjectionIndex(pathBundle) >= 0);
}

int PathSpaceSparseOptimization::getProjectionIndex(const std::vector<base::State *> &pathBundle) const
{
    if (!hasBaseSpace())
    {
        return 0;
    }
    std::vector<base::State *> pathBase = getProjectedPath(pathBundle, getBase());
    // for(uint k = 0; k < pathBundle.size(); k++){
    //   base::State *qk = pathBundle.at(k);
    //   base::State *qkProjected = getBase()->allocState();
    //   projectBase(qk, qkProjected);
    //   pathBase.push_back(qkProjected);
    // }

    PathSpaceSparseOptimization *parent = 
      static_cast<PathSpaceSparseOptimization *>(getBaseBundleSpace());
    unsigned int K = parent->getNumberOfPaths();

    for (uint k = 0; k < K; k++)
    {
        std::vector<base::State *> pathBasek = parent->getKthPath(k);
        bool visible = parent->pathVisibilityChecker_->IsPathVisible(pathBase, pathBasek);
        if (visible)
        {
            freePath(pathBase, getBase());
            return k;
        }
    }
    freePath(pathBase, getBase());
    return -1;
}

// void PathSpaceSparseOptimization::getPathIndices(const std::vector<base::State *> &states, std::vector<int> &idxPath)
// const
// {
//     if (!hasBaseSpace())
//     {
//         return;
//     }
//     else
//     {
//         PathSpaceSparseOptimization *parent = static_cast<PathSpaceSparseOptimization *>(parent_);
//         // TODO: we need to check here to which local minima we project. This is
//         // necessary, since sometimes we find a path which actually projects on a
//         // different Bundle-space path (and not the selected one).
//         // APPROACH 1: Assign them all to selected path
//         // PathSpaceSparseOptimization *Bundle = static_cast<PathSpaceSparseOptimization*>(parent_);
//         // unsigned int K = getBundle()->getNumberOfPaths();
//         // assert(K>0);
//         // unsigned int Ks = getBundle()->getselectedpath();
//         // assert(Ks>=0);
//         // idxPath.push_back(Ks);
//         // getBundle()->getPathIndices(states, idxPath);
//         if (isDynamic())
//         {
//             int Ks = parent->getSelectedPath();
//             std::cout << "DYNAMIC Projection Index " << Ks << "| " << getName() << std::endl;
//             idxPath.push_back(Ks);
//         }
//         else
//         {
//             int K = getProjectionIndex(states);
//             std::cout << "Projection Index " << K << "| " << getName() << std::endl;
//             if (K < 0)
//             {
//                 K = 0;
//                 OMPL_WARN("Projection not found. Possibly unprojectable path.");
//             }
//             idxPath.push_back(K);
//             // getBundle()->getPathIndices(states, idxPath);
//         }
//         std::vector<base::State *> pathBase = getProjectedPath(states, getBase());
//         parent->getPathIndices(pathBase, idxPath);
//     }
// }

const std::vector<ompl::base::State *> PathSpaceSparseOptimization::getKthPath(uint k) const
{
    return pathStackHead_.at(k);
}

// A recursive function to print all paths from 'u' to 'd'.
// visited[] keeps track of vertices in current path.
// path[] stores actual vertices and path_index is current
// index in path[]
void PathSpaceSparseOptimization::printAllPathsUtil(Vertex u, Vertex d, bool visited[], int path[], int &path_index)
{
    // terminate if we have enough paths in stack
    if (pathStack_.size() > Nhead)
        return;
    if (numberOfFailedAddingPathCalls > 10)
        return;

    // Mark the current node and store it in path[]
    visited[u] = true;
    path[path_index] = u;
    path_index++;

    // If current vertex is same as destination, then print
    // current path[]
    if (u == d)
    {
        std::vector<base::State *> pp;
        for (int i = 0; i < path_index; i++)
        {
            pp.push_back(getGraph()[path[i]]->state);
        }
        pushPathToStack(pp);
    }
    else  // If current vertex is not destination
    {
        // Recur for all the vertices adjacent to current vertex
        OEIterator ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::out_edges(u, getGraph()); ei != ei_end; ++ei)
        {
            Vertex source = boost::source(*ei, getGraph());
            Vertex target = boost::target(*ei, getGraph());
            Vertex vnext = (source == u ? target : source);
            if (!visited[vnext])
            {
                printAllPathsUtil(vnext, d, visited, path, path_index);
                if (pathStack_.size() > Nhead)
                    break;
            }
        }
    }

    // Remove current vertex from path[] and mark it as unvisited
    path_index--;
    visited[u] = false;
}
void PathSpaceSparseOptimization::enumerateAllPaths()
{
    if (!hasSolution_)
        return;

    if (isDynamic())
    {
        base::PathPtr path;

        const Configuration *q_nearest_to_goal = nearest(qGoal_);
        Configuration *qStartSparse = getGraph()[getStartIndex()];
        path = getPath(qStartSparse->index, q_nearest_to_goal->index);
        if (path == nullptr)
        {
            OMPL_WARN("No solution found, but hasSolution_ is set.");
            return;
        }
        geometric::PathGeometric &gpath = static_cast<geometric::PathGeometric &>(*path);
        // pathStack_.push_back(gpath);

        pushPathToStack(gpath.getStates());
    }
    else
    {
        // Check if we already enumerated all paths. If yes, then the number of
        // vertices has not changed.
        if (!hasSparseGraphChanged())
        {
            return;
        }
        std::cout << "Enumerating paths on " << getName() << std::endl;

        // Remove Edges
        //(1) REDUCIBLE: Removal of reducible loops [Schmitzberger 02]
        removeReducibleLoops();
        //############################################################################

        // pe.ComputePaths();

        unsigned int numberVertices = boost::num_vertices(getGraph());
        if (numberVertices <= 0)
            return;
        bool *visited = new bool[numberVertices];
        std::cout << "Sparse Graph has " << boost::num_vertices(getGraph()) << " vertices and "
                  << boost::num_edges(getGraph()) << " edges." << std::endl;

        int *path = new int[numberVertices];
        int path_index = 0;  // Initialize path[] as empty

        for (unsigned int i = 0; i < numberVertices; i++)
            visited[i] = false;

        numberOfFailedAddingPathCalls = 0;

        printAllPathsUtil(getStartIndex(), getGoalIndex(), visited, path, path_index);
        //############################################################################
    }
    uint Npathsize = pathStack_.size();
    uint Npaths = std::min(Nhead, Npathsize);
    pathStackHead_.clear();
    for (uint k = 0; k < Npaths; k++)
    {
        // geometric::PathGeometric& pathK = (*(pathStack_.rbegin()+k));
        geometric::PathGeometric &pathK = pathStack_.at(k);  //*(pathStack_.rbegin()+k));
        pathStackHead_.push_back(pathK.getStates());
    }
    OMPL_INFORM("Found %d path classes.", pathStackHead_.size());
    OMPL_INFORM("%s", std::string(80, '-').c_str());
}
// std::vector<int> PathSpaceSparseOptimization::GetSelectedPathIndex() const
// {
//     std::vector<int> CurPath;
//     PathSpaceSparseOptimization *pparent = static_cast<PathSpaceSparseOptimization *>(parent_);
//     while (pparent != nullptr)
//     {
//         CurPath.push_back(pparent->getSelectedPath());
//         pparent = static_cast<PathSpaceSparseOptimization *>(pparent->parent_);
//     }
//     if (selectedPath_ < 0)
//         CurPath.push_back(0);
//     else
//         CurPath.push_back(selectedPath_);

//     return CurPath;
// }

//############################################################################
