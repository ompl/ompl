#include "ompl/control/planners/ltl/ProductGraph.h"
#include "ompl/base/State.h"
#include "ompl/control/planners/ltl/Automaton.h"
#include "ompl/control/planners/ltl/PropositionalDecomposition.h"
#include "ompl/control/planners/ltl/World.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"
#include <algorithm>
#include <boost/function.hpp>
#include <boost/functional/hash.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <map>
#include <ostream>
#include <queue>
#include <stack>
#include <vector>

ompl::control::ProductGraph::State::State(void) :
    decompRegion(-1),
    cosafeState(-1),
    safeState(-1)
{
}

bool ompl::control::ProductGraph::State::operator==(const State& s) const
{
    return decompRegion==s.decompRegion
        && cosafeState==s.cosafeState
        && safeState==s.safeState;
}

bool ompl::control::ProductGraph::State::isValid(void) const
{
    return cosafeState != -1 && safeState != -1;
}

namespace ompl
{
    namespace control
    {
        std::size_t hash_value(const ProductGraph::State& s)
        {
            std::size_t hash = 0;
            boost::hash_combine(hash, s.decompRegion);
            boost::hash_combine(hash, s.cosafeState);
            boost::hash_combine(hash, s.safeState);
            return hash;
        }

        std::ostream& operator<<(std::ostream& out, const ProductGraph::State& s)
        {
            out << "(" << s.decompRegion << "," << s.cosafeState << ",";
            out << s.safeState << ")";
            return out;
        }
    }
}

unsigned int ompl::control::ProductGraph::State::getDecompRegion(void) const
{
    return decompRegion;
}

int ompl::control::ProductGraph::State::getCosafeState(void) const
{
    return cosafeState;
}

int ompl::control::ProductGraph::State::getSafeState(void) const
{
    return safeState;
}

ompl::control::ProductGraph::ProductGraph(const PropositionalDecompositionPtr& decomp,
    const AutomatonPtr& cosafetyAut, const AutomatonPtr& safetyAut) :
    decomp_(decomp),
    cosafety_(cosafetyAut),
    safety_(safetyAut)
{
}

ompl::control::ProductGraph::ProductGraph(const PropositionalDecompositionPtr& decomp,
    const AutomatonPtr& cosafetyAut) :
    decomp_(decomp),
    cosafety_(cosafetyAut),
    safety_(Automaton::AcceptingAutomaton(decomp->getNumProps()))
{
}

const ompl::control::PropositionalDecompositionPtr& ompl::control::ProductGraph::getDecomp() const
{
    return decomp_;
}

const ompl::control::AutomatonPtr& ompl::control::ProductGraph::getCosafetyAutom() const
{
    return cosafety_;
}

const ompl::control::AutomatonPtr& ompl::control::ProductGraph::getSafetyAutom() const
{
    return safety_;
}

std::vector<ompl::control::ProductGraph::State*>
ompl::control::ProductGraph::computeLead(
    ProductGraph::State* start,
    const boost::function<double(ProductGraph::State*, ProductGraph::State*)>& edgeWeight)
{
	std::vector<GraphType::vertex_descriptor> parents(boost::num_vertices(graph_));
	std::vector<double> distances(boost::num_vertices(graph_));
    EdgeIter ei, eend;
    //first build up the edge weights
    for (boost::tie(ei,eend) = boost::edges(graph_); ei != eend; ++ei)
    {
        GraphType::vertex_descriptor src = boost::source(*ei, graph_);
        GraphType::vertex_descriptor target = boost::target(*ei, graph_);
        graph_[*ei].cost = edgeWeight(graph_[src], graph_[target]);
    }
    unsigned int startIndex = stateToIndex_[start];
	boost::dijkstra_shortest_paths(graph_, boost::vertex(startIndex,graph_),
		boost::weight_map(get(&Edge::cost, graph_)).distance_map(
			boost::make_iterator_property_map(distances.begin(), get(boost::vertex_index, graph_)
		)).predecessor_map(
			boost::make_iterator_property_map(parents.begin(), get(boost::vertex_index, graph_))
		)
	);
	//pick state from solutionStates_ such that distance[state] is minimized
	State* bestSoln = *solutionStates_.begin();
	double cost = distances[boost::vertex(stateToIndex_[bestSoln], graph_)];
	for (std::vector<State*>::const_iterator s = solutionStates_.begin()+1; s != solutionStates_.end(); ++s)
	{
		if (distances[boost::vertex(stateToIndex_[*s], graph_)] < cost)
		{
			cost = distances[boost::vertex(stateToIndex_[*s], graph_)];
			bestSoln = *s;
		}
        
        
	}
	//build lead from bestSoln parents
	std::stack<State*> leadStack;
	while (!(bestSoln == start))
	{
		leadStack.push(bestSoln);
		bestSoln = graph_[parents[boost::vertex(stateToIndex_[bestSoln], graph_)]];
	}
	leadStack.push(bestSoln);

	std::vector<State*> lead;
	while (!leadStack.empty())
	{
		lead.push_back(leadStack.top());
		leadStack.pop();
        // Truncate the lead as early when it hits the desired automaton states
        // TODO: more elegant way to do this?
        if (lead.back()->cosafeState == solutionStates_.front()->cosafeState
            && lead.back()->safeState == solutionStates_.front()->safeState)
            break;
	}
	return lead;
}

void ompl::control::ProductGraph::clear()
{
    solutionStates_.clear();
    stateToIndex_.clear();
    startState_ = NULL;
    graph_.clear();
    boost::unordered_map<State,State*>::iterator i;
    for (i = stateToPtr_.begin(); i != stateToPtr_.end(); ++i)
        delete i->second;
    stateToPtr_.clear();
}

void ompl::control::ProductGraph::buildGraph(State* start, const boost::function<void(State*)>& initialize)
{
    graph_.clear();
    solutionStates_.clear();
    std::queue<State*> q;
    boost::unordered_set<State*> processed;
    std::vector<unsigned int> regNeighbors;
    VertexIndexMap index = get(boost::vertex_index, graph_);
    /* Holds states that are closest to accepting in cosafety automaton.
       If the automaton can be satisfied, then this vector
       will end up being equivalent to solutionStates_.
       Otherwise, it will hold the states that have the smallest distance
       from an accepting state in the cosafety automaton, measured in number of
       transitions.
       If the safety automaton cannot be satisfied, then this vector will be empty,
       as no solution is possible in such cases. */
    std::vector<State*> closeStates;

    GraphType::vertex_descriptor next = boost::add_vertex(graph_);
    startState_ = start;
    graph_[boost::vertex(next,graph_)] = startState_;
    stateToIndex_[startState_] = index[next];
    q.push(startState_);
    processed.insert(startState_);

    OMPL_INFORM("Building graph from start state (%u,%u,%u) with index %d",
		startState_->decompRegion, startState_->cosafeState,
		startState_->safeState, stateToIndex_[startState_]);

    while (!q.empty())
    {
        State* current = q.front();
        //Initialize each state using the supplied state initializer function
        initialize(current);
        q.pop();
        /* We only consider a state to be a solution or a close-solution if it is
           accepted by the safety automaton. We are less strict with the cosafety automaton. */
        if (safety_->isAccepting(current->safeState))
        {
            if (cosafety_->isAccepting(current->cosafeState))
            {
                /* Since we have found an actual accepting state,
                   we no longer need to keep track of close states. */
                closeStates.clear();
                solutionStates_.push_back(current);
            }
            else if (solutionStates_.empty())
            {
                if (closeStates.empty())
                    closeStates.push_back(current);
                else
                {
                    unsigned int closeness = cosafety_->distFromAccepting(current->cosafeState);
                    unsigned int best = cosafety_->distFromAccepting(closeStates.front()->cosafeState);
                    //TODO we are only allowing one state in closeStates...
                    if (closeness < best)
                    {
                        closeStates.clear();
                        closeStates.push_back(current);
                    }
                }
            }
        }

        GraphType::vertex_descriptor v = boost::vertex(stateToIndex_[current], graph_);

        //enqueue each neighbor of current
        decomp_->getNeighbors(current->decompRegion, regNeighbors);
        for (std::vector<unsigned int>::const_iterator r = regNeighbors.begin(); r != regNeighbors.end(); ++r)
        {
            State* nextState = getState(current, *r);
            if (!nextState->isValid())
                continue;
			//if this state is newly discovered,
			//then we can dynamically allocate a copy of it
            //and add the new pointer to the graph.
            //either way, we need the pointer
			if (processed.find(nextState) == processed.end())
			{
				const GraphType::vertex_descriptor next = boost::add_vertex(graph_);
                stateToIndex_[nextState] = index[next];
				graph_[boost::vertex(next,graph_)] = nextState;
				q.push(nextState);
                processed.insert(nextState);
			}

            //whether or not the neighbor is newly discovered,
            //we still need to add the edge to the graph
            GraphType::edge_descriptor edge;
            bool ignore;
            boost::tie(edge,ignore) = boost::add_edge(v, boost::vertex(stateToIndex_[nextState],graph_), graph_);
            //graph_[edge].src = index[v];
            //graph_[edge].dest = stateToIndex_[nextState];
        }
		regNeighbors.clear();
    }
    if (solutionStates_.empty())
    {
        OMPL_INFORM("No solution states found in abstraction.");
        if (closeStates.empty())
        {
            OMPL_ERROR("Since no states were found that satisfy the safety automaton, no close states were found either.");
            return;
        }
        OMPL_INFORM("Instead, using states that come closest to satisfying the cosafety automaton.");
        unsigned int best = cosafety_->distFromAccepting(closeStates.front()->cosafeState);
        OMPL_INFORM("These states are %u transitions away from an accepting state.", best);
        solutionStates_.assign(closeStates.begin(), closeStates.end());
    }

    OMPL_INFORM("Number of decomposition regions: %u", decomp_->getNumRegions());
    OMPL_INFORM("Number of cosafety automaton states: %u", cosafety_->numStates());
    OMPL_INFORM("Number of safety automaton states: %u", safety_->numStates());
    OMPL_INFORM("Number of high-level states in abstraction graph: %u", boost::num_vertices(graph_));
}

bool ompl::control::ProductGraph::isSolution(const State* s) const
{
    return std::find(solutionStates_.begin(), solutionStates_.end(), s)
        != solutionStates_.end();
}

ompl::control::ProductGraph::State* ompl::control::ProductGraph::getStartState(void) const
{
    return startState_;
}

double ompl::control::ProductGraph::getRegionVolume(const State* s)
{
    return decomp_->getRegionVolume(s->decompRegion);
}

unsigned int ompl::control::ProductGraph::getCosafeAutDistance(const State* s) const
{
    return cosafety_->distFromAccepting(s->cosafeState);
}

unsigned int ompl::control::ProductGraph::getSafeAutDistance(const State* s) const
{
    return safety_->distFromAccepting(s->safeState);
}

ompl::control::ProductGraph::State* ompl::control::ProductGraph::getState(const base::State* cs) const
{
    State s;
    s.decompRegion = decomp_->locateRegion(cs);
    s.cosafeState = cosafety_->getStartState();
    s.safeState = safety_->getStartState();
	State*& ret = stateToPtr_[s];
	if (ret == NULL)
		ret = new State(s);
    return ret;
}

ompl::control::ProductGraph::State* ompl::control::ProductGraph::getState(const State* parent, unsigned int nextRegion) const
{
    State s;
    s.decompRegion = nextRegion;
    const World nextWorld = decomp_->worldAtRegion(nextRegion);
    s.cosafeState = cosafety_->step(parent->cosafeState, nextWorld);
    s.safeState = safety_->step(parent->safeState, nextWorld);
	State*& ret = stateToPtr_[s];
	if (ret == NULL)
		ret = new State(s);
    return ret;
}

ompl::control::ProductGraph::State* ompl::control::ProductGraph::getState(const State* parent, const base::State* cs) const
{
    return getState(parent, decomp_->locateRegion(cs));
}
