#include "ompl/control/planners/ltl/Automaton.h"
#include "ompl/control/planners/ltl/World.h"
#include <boost/range/irange.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/dynamic_bitset.hpp>
#include <ostream>
#include <limits>
#include <queue>
#include <vector>

int ompl::control::Automaton::TransitionMap::eval(const World& w) const
{
    typedef boost::unordered_map<World, unsigned int>::const_iterator DestIter;
    DestIter d = entries.find(w);
    if (d != entries.end())
        return d->second;
    for (d = entries.begin(); d != entries.end(); ++d)
    {
        if (w.satisfies(d->first))
        {
            //Since w satisfies another world that leads to d->second,
            //we can add an edge directly from w to d->second.
            entries[w] = d->second;
            return d->second;
        }
    }
    return -1;
}

ompl::control::Automaton::Automaton(unsigned int numProps, unsigned int numStates) :
    numProps_(numProps),
    numStates_(numStates),
    startState_(-1),
    accepting_(numStates_, false),
    transitions_(numStates_),
    distances_(numStates_, std::numeric_limits<unsigned int>::max())
{
}

unsigned int ompl::control::Automaton::addState(bool accepting)
{
    ++numStates_;
    accepting_.resize(numStates_);
    accepting_[numStates_-1] = accepting;
    transitions_.resize(numStates_);
    return numStates_-1;
}

void ompl::control::Automaton::setAccepting(unsigned int s, bool a)
{
    accepting_[s] = a;
}

bool ompl::control::Automaton::isAccepting(unsigned int s) const
{
    return accepting_[s];
}

void ompl::control::Automaton::setStartState(unsigned int s)
{
    startState_ = s;
}

int ompl::control::Automaton::getStartState(void) const
{
    return startState_;
}

void ompl::control::Automaton::addTransition(
    unsigned int src, 
    const World& w,
    unsigned int dest)
{
    TransitionMap& map = transitions_[src];
    map.entries[w] = dest;
}

bool ompl::control::Automaton::run(const std::vector<World>& trace) const
{
    int current = startState_;
    for (std::vector<World>::const_iterator w = trace.begin(); w != trace.end(); ++w)
    {
        current = step(current, *w);
        if (current == -1)
            return false;
    }
    return true;
}

int ompl::control::Automaton::step(int state, const World& w) const
{
    if (state == -1)
        return -1;
    return transitions_[state].eval(w);
}

ompl::control::Automaton::TransitionMap& ompl::control::Automaton::getTransitions(unsigned int src)
{
    return transitions_[src];
}

unsigned int ompl::control::Automaton::numStates(void) const
{
    return numStates_;
}

unsigned int ompl::control::Automaton::numTransitions(void) const
{
    unsigned int ntrans = 0;
    typedef std::vector<TransitionMap>::const_iterator TransIter;
    for (TransIter i = transitions_.begin(); i != transitions_.end(); ++i)
        ntrans += i->entries.size();
    return ntrans;
}

unsigned int ompl::control::Automaton::numProps(void) const
{
    return numProps_;
}

void ompl::control::Automaton::print(std::ostream& out) const
{
    out << "digraph automaton {" << std::endl;
    out << "rankdir=LR" << std::endl;
    for (unsigned int i = 0; i < numStates_; ++i)
    {
        out << i << " [label=\"" << i << "\",shape=";
        out << (accepting_[i] ? "doublecircle" : "circle") << "]" << std::endl;

        const TransitionMap& map = transitions_[i];
        boost::unordered_map<World, unsigned int>::const_iterator e;
        for (e = map.entries.begin(); e != map.entries.end(); ++e)
        {
            const World& w = e->first;
            unsigned int dest = e->second;
            const std::string formula = w.formula();
            out << i << " -> " << dest << " [label=\"" << formula << "\"]" << std::endl;
        }
    }
    out << "}" << std::endl;
}

unsigned int ompl::control::Automaton::distFromAccepting(unsigned int s, unsigned int maxDist) const
{
    if (distances_[s] < std::numeric_limits<unsigned int>::max())
        return distances_[s];
    if (accepting_[s])
        return 0;
    std::queue<unsigned int> q;
    boost::unordered_set<unsigned int> processed;
    boost::unordered_map<unsigned int, unsigned int> distance;

    q.push(s);
    distance[s] = 0;
    processed.insert(s);

    while (!q.empty())
    {
        unsigned int current = q.front();
        q.pop();
        if (accepting_[current])
        {
            distances_[s] = distance[current];
            return distance[current];
        }
        const TransitionMap& map = transitions_[current];
        boost::unordered_map<World, unsigned int>::const_iterator e;
        for (e = map.entries.begin(); e != map.entries.end(); ++e)
        {
            unsigned int neighbor = e->second;
            if (processed.count(neighbor) > 0)
                continue;
            q.push(neighbor);
            processed.insert(neighbor);
            distance[neighbor] = distance[current]+1;
        }
    }
    return std::numeric_limits<unsigned int>::max();
}

ompl::control::AutomatonPtr ompl::control::Automaton::AcceptingAutomaton(unsigned int numProps)
{
    AutomatonPtr phi(new Automaton(numProps, 1));
    World trivial(numProps);
    phi->addTransition(0, trivial, 0);
    phi->setStartState(0);
    phi->setAccepting(0, true);
    return phi;
}

ompl::control::AutomatonPtr ompl::control::Automaton::CoverageAutomaton(unsigned int numProps, const std::vector<unsigned int>& covProps)
{
    AutomatonPtr phi(new Automaton(numProps, 1<<covProps.size()));
    for (unsigned int src = 0; src < phi->numStates(); ++src)
    {
        const boost::dynamic_bitset<> state(covProps.size(), src);
        World loop(numProps);
        //each value of p is an index of a proposition in covProps
        for (unsigned int p = 0; p < covProps.size(); ++p)
        {
            //if proposition covProps[p] has already been covered at state src, skip it
            if (state[p])
                continue;
            //for each proposition covProps[p] that has not yet been
            //covered at state src, construct a transition from src to (src|p)
            //on formula (covProps[p]==true)
            boost::dynamic_bitset<> target(state);
            target[p] = true;
            World nextProp(numProps);
            nextProp[covProps[p]] = true;
            phi->addTransition(src, nextProp, target.to_ulong());
            //also build a loop from src to src on formula with conjunct (covProps[p]==false)
            loop[covProps[p]] = false;
        }
        //now we add a loop from src to src on conjunction of (covProps[p]==false)
        //for every p such that the pth bit of src is 1
        phi->addTransition(src, loop, src);
    }
    phi->setAccepting(phi->numStates()-1, true);
    phi->setStartState(0);
    return phi;
}

ompl::control::AutomatonPtr ompl::control::Automaton::SequenceAutomaton(unsigned int numProps, const std::vector<unsigned int>& seqProps)
{
    AutomatonPtr seq(new Automaton(numProps, seqProps.size()+1));
    for (unsigned int state = 0; state < seqProps.size(); ++state)
    {
        // loop when next proposition in sequence is not satisfied
        World loop(numProps);
        loop[seqProps[state]] = false;
        seq->addTransition(state, loop, state);

        // progress forward when next proposition in sequence is satisfied
        World progress(numProps);
        progress[seqProps[state]] = true;
        seq->addTransition(state, progress, state+1);
    }
    //loop on all input when in accepting state
    seq->addTransition(seqProps.size(), World(numProps), seqProps.size());
    seq->setAccepting(seqProps.size(), true);
    seq->setStartState(0);
    return seq;
}

ompl::control::AutomatonPtr ompl::control::Automaton::DisjunctionAutomaton(unsigned int numProps, const std::vector<unsigned int>& disjProps)
{
    AutomatonPtr disj(new Automaton(numProps, 2));
    World loop(numProps);
    for (std::vector<unsigned int>::const_iterator p = disjProps.begin(); p != disjProps.end(); ++p)
    {
        World satisfy(numProps);
        satisfy[*p] = true;
        loop[*p] = false;
        disj->addTransition(0, satisfy, 1);
    }
    disj->addTransition(0, loop, 0);
    disj->addTransition(1, World(numProps), 1);
    disj->setAccepting(1, true);
    disj->setStartState(0);
    return disj;
}

ompl::control::AutomatonPtr ompl::control::Automaton::AvoidanceAutomaton(unsigned int numProps, const std::vector<unsigned int>& avoidProps)
{
    /* An avoidance automaton is simply a disjunction automaton with its acceptance condition flipped. */
    AutomatonPtr avoid = DisjunctionAutomaton(numProps, avoidProps);
    avoid->setAccepting(0, true);
    avoid->setAccepting(1, false);
    return avoid;
}

ompl::control::AutomatonPtr ompl::control::Automaton::CoverageAutomaton(unsigned int numProps)
{
    const boost::integer_range<unsigned int> props = boost::irange(0u,numProps);
    return CoverageAutomaton(numProps, std::vector<unsigned int>(props.begin(), props.end()));
}

ompl::control::AutomatonPtr ompl::control::Automaton::SequenceAutomaton(unsigned int numProps)
{
    const boost::integer_range<unsigned int> props = boost::irange(0u,numProps);
    return SequenceAutomaton(numProps, std::vector<unsigned int>(props.begin(), props.end()));
}

ompl::control::AutomatonPtr ompl::control::Automaton::DisjunctionAutomaton(unsigned int numProps)
{
    const boost::integer_range<unsigned int> props = boost::irange(0u,numProps);
    return DisjunctionAutomaton(numProps, std::vector<unsigned int>(props.begin(), props.end()));
}
