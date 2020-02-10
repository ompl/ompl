#include <ompl/geometric/planners/quotientspace/algorithms/SQMPImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"

#define foreach BOOST_FOREACH

ompl::geometric::SQMPImpl::SQMPImpl(const base::SpaceInformationPtr &si, QuotientSpace *parent_) : BaseT(si, parent_)
{
    setName("SQMPImpl" + std::to_string(id_));
    Planner::declareParam<double>("range", this, &SQMPImpl::setRange, &SQMPImpl::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &SQMPImpl::setGoalBias, &SQMPImpl::getGoalBias, "0.:.1:1.");
    qRandom_ = new Configuration(Q1);
}

ompl::geometric::SQMPImpl::~SQMPImpl()
{
    deleteConfiguration(qRandom_);
}

void ompl::geometric::SQMPImpl::setGoalBias(double goalBias)
{
    goalBias_ = goalBias;
}

double ompl::geometric::SQMPImpl::getGoalBias() const
{
    return goalBias_;
}

void ompl::geometric::SQMPImpl::setRange(double maxDistance)
{
    maxDistance_ = maxDistance;
}

double ompl::geometric::SQMPImpl::getRange() const
{
    return maxDistance_;
}

void ompl::geometric::SQMPImpl::setup()
{
    BaseT::setup();
    ompl::tools::SelfConfig sc(Q1, getName());
    sc.configurePlannerRange(maxDistance_);
    std::cout << "max dis --- --- --------------------------- -- " << maxDistance_ << ",, ,, , spd " << sparseDelta_ << std::endl;
}

void ompl::geometric::SQMPImpl::clear()
{
    BaseT::clear();
}

bool ompl::geometric::SQMPImpl::getSolution(base::PathPtr &solution)
{
    if (hasSolution_)
    {
        bool baset_sol = BaseT::getSolution(solution);
        if (baset_sol)
        {
            shortestPathVertices_ = shortestVertexPath_;
        }
        return baset_sol;
    }
    else
    {
        return false;
    }
}

void ompl::geometric::SQMPImpl::grow()
{
    // start from qrrt
    if (firstRun_)
    {
        Init();
        firstRun_ = false;
    }

    sample(qRandom_->state);
    addMileStone(qRandom_);
    
}
void ompl::geometric::SQMPImpl::expand()
{
    if (firstRun_)
    {
        Init();
        firstRun_ = false;
    }
    PDF pdf;
    foreach (Vertex v, boost::vertices(graph_))
    {
        const unsigned long int t = graph_[v]->total_connection_attempts;
        pdf.add(graph_[v], (double)(t - graph_[v]->successful_connection_attempts) / (double)t);
    }

    if (pdf.empty())
        return;
    // random number 1 to 10
    //int j = std::max(10 * rng_.uniform01() , (double)(boost::num_vertices(graph_) - 1));
    // pick one from top 10 ?
    // check nodes which has high probability

    //std::vector<base::State *> randomWorkStates(5);
    
    Configuration *q = pdf.sample(rng_.uniform01());
    
    sample(q->state);
    addMileStone(q);
    
    /*int s = si_->randomBounceMotion(Q1_sampler_, q->state, randomWorkStates.size(), randomWorkStates, false);
    for (int i = 0; i < s; i++)
    {
        Configuration *tmp = new Configuration(randomWorkStates[i]);
        addMileStone(tmp);
    }*/
    
}

void ompl::geometric::SQMPImpl::addMileStone(Configuration *q_random)
{
    Configuration *q_next = addConfigurationDense(qRandom_);

    findGraphNeighbors(q_next, graphNeighborhood, visibleNeighborhood);

    if (!checkAddCoverage(q_next, visibleNeighborhood))
        if (!checkAddConnectivity(q_next, visibleNeighborhood))
            if (!checkAddInterface(q_next, graphNeighborhood, visibleNeighborhood))
            {
                if (!checkAddPath(q_next))
                    ++consecutiveFailures_;
            }
    if (isDenseFoundSolution_)
    {
        bool same_component = sameComponentSparse(v_start_sparse, v_goal_sparse);
        if(!same_component) {
            return;
        }
        hasSolution_ = true;
    }
}

ompl::geometric::QuotientSpaceGraph::Configuration * ompl::geometric::SQMPImpl::addConfigurationDense(Configuration *q_random)
{
    /*const Configuration *q_nearest = ompl::geometric::QuotientSpaceGraph::nearest(q_random);
    double d = Q1->distance(q_nearest->state, q_random->state);
    if (d > maxDistance_)
    {
        Q1->getStateSpace()->interpolate(q_nearest->state, q_random->state, maxDistance_ / d, q_random->state);
    }*/
    // end interpolation

    Configuration *q_next = new Configuration(Q1, q_random->state);
    Vertex v_next = ompl::geometric::QuotientSpaceGraph::addConfiguration(q_next);
    totalNumberOfSamples_++;
    totalNumberOfFeasibleSamples_++;

    // Calculate K , k(n) = kPRMConstant * log(n), where kPRMConstant > kStarPRMConstant = e(1 + 1/d)
    unsigned int k = static_cast<unsigned int>(ceil(kPRMStarConstant_ * log((double) boost::num_vertices(graph_))));

    // find nearest neighbors to be conected to new sample
    std::vector<Configuration *> r_nearest_neighbors;
    ompl::geometric::QuotientSpaceGraph::nearestDatastructure_->nearestK(q_next, k, r_nearest_neighbors);
    
    //std::cout << "KStart neighb\t" << k << "\t-\t" << r_nearest_neighbors.size() << std::endl; 

    for (unsigned int i = 0; i < r_nearest_neighbors.size(); i++)
    {
        q_next->total_connection_attempts++;
        Configuration *q_neighbor = r_nearest_neighbors.at(i);
        q_neighbor->total_connection_attempts++;

        if (Q1->checkMotion(q_neighbor->state, q_random->state))
        {
            ompl::geometric::QuotientSpaceGraph::addEdge(q_neighbor->index, v_next);
            q_next->successful_connection_attempts++;
            q_neighbor->successful_connection_attempts++;
            //remove hasSolution_ for prm use instead stop growing condition here
            if (q_neighbor->isGoal && !isDenseFoundSolution_)
            {
                bool same_component = sameComponent(vStart_, vGoal_);
                if (!same_component)
                {
                    isDenseFoundSolution_ = false;
                }
                else
                {
                    isDenseFoundSolution_ = true;
                }
            }
            /*if (!isDenseFoundSolution_)
            {
                double dist = 0.0;
                bool satisfied = goal_->isSatisfied(q_next->state, &dist);
                if (satisfied)
                {
                    //vGoal_ = ompl::geometric::QuotientSpaceGraph::addConfiguration(qGoal_);
                    addEdge(q_neighbor->index, vGoal_);
                    isDenseFoundSolution_ = true;
                    // sa-> check both start & goal should be in same compnent because for prm can have many graph
                    //component and no need incase of rrt 
                    bool same_component = sameComponent(vStart_, vGoal_);
                    if(!same_component)
                    {
                        isDenseFoundSolution_ = false;
                    }
                }
            }*/
        }
    }

    // Update its representative
    std::vector<Configuration *> graphNeighborhood;
    nearestSparse_->nearestR(q_next, sparseDelta_, graphNeighborhood); // Sparse Neighbors

    for (Configuration *qn : graphNeighborhood)
        if (si_->checkMotion(q_next->state, qn->state))
        {
            q_next->representativeIndex = qn->index;
            break;
        }

    if ( q_next->representativeIndex < 0 )
        return q_next;


    std::vector<Vertex> interfaceNeighborhood;
    std::set<Vertex> interfaceRepresentatives;

    getInterfaceNeighborRepresentatives(q_next, interfaceRepresentatives);
    getInterfaceNeighborhood(q_next, interfaceNeighborhood);
    addToRepresentatives(v_next, q_next->representativeIndex, interfaceRepresentatives);
    
    foreach (Vertex qp, interfaceNeighborhood)
    {
        Vertex qp_rep = graph_[qp]->representativeIndex;
        if ( qp_rep == -1 )
            return q_next;
        removeFromRepresentatives(graph_[qp]);
        getInterfaceNeighborRepresentatives(graph_[qp], interfaceRepresentatives);
        addToRepresentatives(qp, qp_rep, interfaceRepresentatives);
    }
    return q_next;
}

double ompl::geometric::SQMPImpl::getImportance() const
{
    // Should depend on
    // (1) level : The higher the level, the more importance
    // (2) total samples: the more we already sampled, the less important it
    // becomes
    // (3) has solution: if it already has a solution, we should explore less
    // (only when nothing happens on other levels)
    // (4) vertices: the more vertices we have, the less important (let other
    // levels also explore)
    //
    // exponentially more samples on level i. Should depend on ALL levels.
    // const double base = 2;
    // const double normalizer = powf(base, level);
    // double N = (double)GetNumberOfVertices()/normalizer;
    double N = (double)getNumberOfVertices();
    return 1.0 / (N + 1);
}

// Make it faster by removing the validity check
bool ompl::geometric::SQMPImpl::sample(base::State *q_random)
{
    if (parent_ == nullptr)
    {
        Q1_sampler_->sampleUniform(q_random);
    }
    else
    {
        if (X1_dimension_ > 0)
        {
            X1_sampler_->sampleUniform(s_X1_tmp_);
            parent_->sampleQuotient(s_Q0_tmp_);
            mergeStates(s_Q0_tmp_, s_X1_tmp_, q_random);
        }
        else
        {
            parent_->sampleQuotient(q_random);
        }
    }
    return true;
}

bool ompl::geometric::SQMPImpl::sampleQuotient(base::State *q_random_graph)
{
    // RANDOM VERTEX SAMPLING
    const Vertex v = boost::random_vertex(graph_, rng_boost);
    Q1->getStateSpace()->copyState(q_random_graph, graph_[v]->state);
    return true;
}

bool ompl::geometric::SQMPImpl::getPlannerTerminationCondition()
{
    return hasSolution_ || consecutiveFailures_ > maxFailures_;
}
