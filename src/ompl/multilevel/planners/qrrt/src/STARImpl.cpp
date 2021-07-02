#include <ompl/multilevel/planners/qrrt/STARImpl.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/multilevel/datastructures/propagators/Geometric.h>
#include <ompl/multilevel/datastructures/metrics/Geodesic.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/special/TorusStateSpace.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH
using namespace ompl::multilevel;

STARImpl::STARImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("STARImpl" + std::to_string(id_));
    setImportance("exponential");
    setGraphSampler("randomedge");
    getGraphSampler()->disableSegmentBias();
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

STARImpl::~STARImpl()
{
}

void STARImpl::setup()
{
    BaseT::setup();

    maxDistance_ = 0.1;

    if(!treeStart_)
    {
      TreeData tree;
      tree.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
      tree->setDistanceFunction(
          [this](const Configuration *a, const Configuration *b) 
          { return distance(a, b); });
      treeStart_ = std::make_shared<SparseTree>(tree, getBundle());
      treeStart_->setup();
    }
    if(!treeGoal_)
    {
      TreeData tree;
      tree.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
      tree->setDistanceFunction(
          [this](const Configuration *a, const Configuration *b) 
          { return distance(a, b); });

      treeGoal_ = std::make_shared<SparseTree>(tree, getBundle());
      treeGoal_->setup();
    }
    //setFindSectionStrategy(FindSectionType::SIDE_STEP):
    setFindSectionStrategy(FindSectionType::PATTERN_DANCE);
}
void STARImpl::clear()
{
    BaseT::clear();
    treeStart_->clear();
    treeGoal_->clear();
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

void STARImpl::init()
{
    if (const base::State *state = pis_.nextStart())
    {
        qStart_ = new Configuration(getBundle(), state);
        qStart_->isStart = true;
        addToTree(treeStart_, qStart_);
    }

    if (qStart_ == nullptr)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        throw ompl::Exception("Invalid initial states.");
    }

    if (const base::State *state = pis_.nextGoal())
    {
        qGoal_ = new Configuration(getBundle(), state);
        qGoal_->isGoal = true;
        addToTree(treeGoal_, qGoal_);
        goalConfigurations_.push_back(qGoal_);
    }

    if (qGoal_ == nullptr && getGoalPtr()->canSample())
    {
        OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
        throw ompl::Exception("Invalid goal states.");
    }
}

void STARImpl::addToTree(SparseTreePtr& tree, Configuration *x)
{
    Vertex m = boost::add_vertex(x, graph_);
    disjointSets_.make_set(m);
    x->index = m;
    tree->add(x);
}

void STARImpl::grow()
{
    // TODO
    // [ ] Add conditional adding
    // [ ] Add termination criterion
    // [ ] Provide guarantees in terms of free space covered
    // [ ] Add explicit shell sampling to SO3
    // [x] Impl shell sampling
    // [x] Color start/goal trees differently
    //
    //  -> Longterm Goal is to solve 06D mug infeasible/feasible

    if (firstRun_)
    {
        init();
        firstRun_ = false;

        findSection();
    }

    //###########################################################
    //(0) Tree Selection. 
    SparseTreePtr &tree = activeInitialTree_ ? treeStart_ : treeGoal_;
    activeInitialTree_ = !activeInitialTree_;
    SparseTreePtr &otherTree = activeInitialTree_ ? treeStart_ : treeGoal_;

    //DEBUG with only start tree
    // SparseTreePtr &tree = treeStart_;
    // SparseTreePtr &otherTree = treeGoal_;

    if(tree->isConverged()) return;

    //###########################################################
    //(1) State Selection
    //  [ ] Selected state based on 
    //    (i) number of unsuccessful attempts.
    //    (ii) frontier node status
    //    (iii) not yet expanded
    //    (iv) number of neighbors. More neighbors means more of the shell is
    //    blocked. 
    //
    // std::vector<Configuration*> treeElements;
    // tree->list(treeElements);

    // int selectedTreeElement = rng_.uniformInt(0, tree->size()-1);
    // Configuration *xSelected = treeElements.at(selectedTreeElement);

    Configuration *xSelected = tree->pop();

    if(!xSelected) return;

    //###########################################################
    //(2) Extend Selection
    // auto sampler = std::static_pointer_cast<base::RealVectorStateSampler>(getBundleSamplerPtr());

    double sparseDelta = tree->getRadius(xSelected);

    //if infeasible, make delta smaller

    auto sampler = getBundleSamplerPtr();
    sampler->sampleShell(xRandom_->state, xSelected->state, 
        sparseDelta, sparseDelta + 0.01*sparseDelta);

    double ds = distance(xSelected, xRandom_);
    if(ds < sparseDelta)
    {
        tree->push(xSelected, EXTENSION_FAILURE_SHELL_SAMPLING);
        return;
    }

    bool valid = getBundle()->getStateValidityChecker()->isValid(xRandom_->state);
    if(!valid)
    {
        // boost::add_vertex(new Configuration(getBundle(), xRandom_->state), graph_);
        tree->push(xSelected, EXTENSION_FAILURE_INVALID_STATE);
        return;
    }

    //###########################################################
    //(3) Remove Covered Samples.
    //  Need to keep some to guarantee near-optimality.

    //It could happen that we have a feasible sample, which is not connectable
    Configuration *xNearest = tree->nearest(xRandom_);
    double d = distance(xNearest, xRandom_);
    if (d < sparseDelta && 
        getBundle()->checkMotion(xNearest->state, xRandom_->state))
    {
        if(xNearest == xSelected)
        {
            OMPL_ERROR("Shell sampling returned a state inside inner radius.");
            throw "ERROR";
        }
        bool selfConnection = getBundle()->checkMotion(xSelected->state, xRandom_->state);
        if(selfConnection)
        {
          // boost::add_vertex(new Configuration(getBundle(), xRandom_->state), graph_);
        }else
        {
          Configuration *xNext = new Configuration(getBundle(), xRandom_->state);
          addToTree(tree, xNext);
          addBundleEdge(xNearest, xNext);
          // tree->push(xSelected, EXTENSION_SUCCESS);
          // std::cout << "Connector " << xSelected->index << " to " << xNearest->index << std::endl;
        }
        tree->push(xSelected, EXTENSION_FAILURE_INSIDE_COVER);
        return;
    }

    //###########################################################
    //(4) Connect Selected to Random
    // Improvements: Do local planning to reach state. We could even use a
    // potential field method here. 

    // if (!propagator_->steer(xSelected, xRandom_, xRandom_))
    if(!getBundle()->checkMotion(xSelected->state, xRandom_->state))
    {
        // boost::add_vertex(new Configuration(getBundle(), xRandom_->state), graph_);
        //If we find a valid sample which is not connectable, then this usually
        //indicates a complicated state space geometry which could be useful to
        //investigate further (i.e. allocate more resources to this)
        tree->push(xSelected, EXTENSION_FAILURE_NO_CONNECTION);
        return;
    }

    //###########################################################
    //(5) Valid Connected Element is added to Tree

    Configuration *xNext = new Configuration(getBundle(), xRandom_->state);
    addToTree(tree, xNext);
    addBundleEdge(xSelected, xNext);
    tree->push(xSelected, EXTENSION_SUCCESS);

    //###########################################################
    //(6) If extension was successful, check if we reached goal
    if (xNext && !hasSolution_)
    {
        /* update distance between trees */
        Configuration *xOtherTree = otherTree->nearest(xNext);
        const double newDist = distance(xNext, xOtherTree);
        if (newDist < distanceBetweenTrees_)
        {
            distanceBetweenTrees_ = newDist;
            OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
        }

        bool satisfied = propagator_->steer(xNext, xOtherTree, xRandom_);

        if (satisfied)
        {
            addBundleEdge(xNext, xOtherTree);
            hasSolution_ = true;
        }
    }
}

bool STARImpl::isInfeasible()
{
  return !hasSolution_ && (treeStart_->isConverged() || treeGoal_->isConverged());
}

bool STARImpl::hasConverged()
{
  return false;
}

double STARImpl::getImportance() const
{
  return BaseT::getImportance();
}

void STARImpl::getPlannerData(ompl::base::PlannerData &data) const
{
    BaseT::getPlannerData(data);
    OMPL_DEBUG(" Start Tree has %d (%d cnvrgd) vertices, Goal Tree has %d (%d cnvrgd) vertices.", 
        treeStart_->size(), treeStart_->sizeConvergedNodes(), 
        treeGoal_->size(), treeGoal_->sizeConvergedNodes());
}
