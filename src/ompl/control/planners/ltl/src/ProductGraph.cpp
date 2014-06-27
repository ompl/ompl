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

int ompl::control::ProductGraph::State::getDecompRegion(void) const
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

ompl::control::ProductGraph::~ProductGraph()
{
    clear();
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
    int startIndex = stateToIndex_[start];
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
    std::vector<int> regNeighbors;
    VertexIndexMap index = get(boost::vertex_index, graph_);

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

        if (safety_->isAccepting(current->safeState) && cosafety_->isAccepting(current->cosafeState))
        {
            solutionStates_.push_back(current);
        }

        GraphType::vertex_descriptor v = boost::vertex(stateToIndex_[current], graph_);

        //enqueue each neighbor of current
        decomp_->getNeighbors(current->decompRegion, regNeighbors);
        for (std::vector<int>::const_iterator r = regNeighbors.begin(); r != regNeighbors.end(); ++r)
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
        OMPL_ERROR("No solution path found in product graph.");
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

int ompl::control::ProductGraph::getCosafeAutDistance(const State* s) const
{
    return cosafety_->distFromAccepting(s->cosafeState);
}

int ompl::control::ProductGraph::getSafeAutDistance(const State* s) const
{
    return safety_->distFromAccepting(s->safeState);
}

ompl::control::ProductGraph::State* ompl::control::ProductGraph::getState(const base::State* cs) const
{
    return getState(cs, cosafety_->getStartState(), safety_->getStartState());
}

ompl::control::ProductGraph::State* ompl::control::ProductGraph::getState(const base::State* cs, int cosafe, int safe) const
{
    State s;
    s.decompRegion = decomp_->locateRegion(cs);
    s.cosafeState = cosafe;
    s.safeState = safe;
	State*& ret = stateToPtr_[s];
	if (ret == NULL)
		ret = new State(s);
    return ret;
}

ompl::control::ProductGraph::State* ompl::control::ProductGraph::getState(const State* parent, int nextRegion) const
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
