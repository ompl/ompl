#include "ompl/control/planners/ltl/LTLPlanner.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/planners/ltl/ProductGraph.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/util/Console.h"
#include <algorithm>
#include <boost/unordered_map.hpp>
#include <limits>
#include <map>
#include <vector>

ompl::control::LTLPlanner::LTLPlanner(const SpaceInformationPtr& si, const ProductGraphPtr& a, double exploreTime) :
    ompl::base::Planner(si, "LTLPlanner"),
    siC_(si.get()),
    abstraction_(a),
    exploreTime_(exploreTime)
{
    specs_.approximateSolutions = true;
}

ompl::control::LTLPlanner::~LTLPlanner(void)
{
}

void ompl::control::LTLPlanner::setup()
{
    base::Planner::setup();
}

void ompl::control::LTLPlanner::clear()
{
    base::Planner::clear();
    starts_.clear();
    availDist_.clear();
    abstractInfo_.clear();
    clearMotions();
}

ompl::base::PlannerStatus ompl::control::LTLPlanner::solve(const base::PlannerTerminationCondition& ptc)
{
    return solve(ptc, NULL);
}

ompl::base::PlannerStatus ompl::control::LTLPlanner::solve(const ompl::base::PlannerTerminationCondition& ptc, ProductGraph::State* highLevelStart)
{
    checkValidity();
    //TODO for now, we are only taking the first start state
    //TODO add OMPL_WARN message if >1 start state given, that only the first one will be used
    const base::State* s = pis_.nextStart();
    Motion *motion = new Motion(siC_);
    si_->copyState(motion->state, s);
    siC_->nullControl(motion->control);
    motions_.push_back(motion);

    ProductGraph::State* a = highLevelStart;
    if (a == NULL)
        a = abstraction_->getState(s);
    starts_.push_back(a);

    ompl::time::point start = ompl::time::now();
    abstraction_->buildGraph(starts_[0], boost::bind(&LTLPlanner::initAbstractInfo, this, _1));

    abstractInfo_[a].addMotion(motion);
    updateWeight(a);
    availDist_.add(a, abstractInfo_[a].weight);
    motion->abstractState = a;
    //TODO consider moving estimates and weights into here instead of in abstraction state

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocControlSampler();

    bool solved = false;
    Motion* soln;

    while (ptc()==false && !solved)
    {
        const std::vector<ProductGraph::State*> lead = abstraction_->computeLead(starts_[0], boost::bind(&LTLPlanner::abstractEdgeWeight, this, _1, _2));
        buildAvail(lead);
        solved = explore(lead, soln, exploreTime_);
    }

    if (solved)
    {
        //build solution path
        std::vector<Motion*> path;
        while (soln != NULL)
        {
            path.push_back(soln);
            soln = soln->parent;
        }
        PathControl* pc = new PathControl(si_);
        for (int i = path.size()-1; i >= 0; --i)
        {
            if (path[i]->parent != NULL)
                pc->append(path[i]->state, path[i]->control, path[i]->steps * siC_->getPropagationStepSize());
            else
                pc->append(path[i]->state);
        }
        pdef_->addSolutionPath(base::PathPtr(pc));
    }

    OMPL_INFORM("Created %u states", motions_.size());
    return base::PlannerStatus(solved, false);
}

void ompl::control::LTLPlanner::getTree(std::vector<base::State*>& tree) const
{
    tree.resize(motions_.size());
    for (unsigned int i = 0; i < motions_.size(); ++i)
        tree[i] = motions_[i]->state;
}

std::vector<ompl::control::ProductGraph::State*> ompl::control::LTLPlanner::getHighLevelPath(const std::vector<base::State*>& path, ProductGraph::State* start) const
{
    std::vector<ProductGraph::State*> hlPath(path.size());
    hlPath[0] = (start != NULL ? start : abstraction_->getState(path[0]));
    for (unsigned int i = 1; i < path.size(); ++i)
    {
        hlPath[i] = abstraction_->getState(hlPath[i-1], path[i]);
        if (!hlPath[i]->isValid())
            OMPL_WARN("High-level path fails automata");
    }
    return hlPath;
}

ompl::control::LTLPlanner::Motion::Motion(void) : state(NULL), control(NULL), parent(NULL), steps(0)
{
}

ompl::control::LTLPlanner::Motion::Motion(const SpaceInformation* si) :
    state(si->allocState()),
    control(si->allocControl()),
    parent(NULL), 
    steps(0)
{
}

ompl::control::LTLPlanner::Motion::~Motion(void)
{
}

ompl::control::LTLPlanner::ProductGraphStateInfo::ProductGraphStateInfo(void) :
    numSel(0),
    pdfElem(NULL)
{
}

void ompl::control::LTLPlanner::ProductGraphStateInfo::addMotion(Motion* m)
{
    motionElems[m] = motions.add(m, 1.);
}

double ompl::control::LTLPlanner::updateWeight(ProductGraph::State* as)
{
    ProductGraphStateInfo& info = abstractInfo_[as];
    /* TODO weight should include freeVolume, for cases in which decomposition
       does not respect obstacles. */
    info.weight = ((info.motions.size()+1)*info.volume) / (info.autWeight*(info.numSel+1)*(info.numSel+1));
    return info.weight;
}

void ompl::control::LTLPlanner::initAbstractInfo(ProductGraph::State* as)
{
    ProductGraphStateInfo& info = abstractInfo_[as];
    info.numSel = 0;
    info.pdfElem = NULL;
	info.volume = abstraction_->getRegionVolume(as);
	unsigned int autDist = std::max(abstraction_->getCosafeAutDistance(as),
		abstraction_->getSafeAutDistance(as));
    //TODO try something larger than epsilon
    if (autDist == 0)
        info.autWeight = std::numeric_limits<double>::epsilon();
    else
        info.autWeight = autDist;
    info.weight = info.volume/info.autWeight;
}

void ompl::control::LTLPlanner::buildAvail(const std::vector<ProductGraph::State*>& lead)
{
    for (unsigned int i = 0; i < availDist_.size(); ++i)
        abstractInfo_[availDist_[i]].pdfElem = NULL;
    availDist_.clear();
    unsigned int numTreePts = 1;
    for (int i = lead.size()-1; i >= 0; --i)
    {
        ProductGraph::State* as = lead[i];
        ProductGraphStateInfo& info = abstractInfo_[as];
        if (!info.motions.empty())
        {
            info.pdfElem = availDist_.add(as, info.weight);
            numTreePts += info.motions.size();
            if (rng_.uniform01() < 0.5)
                break;
        }
    }
}

bool ompl::control::LTLPlanner::explore(const std::vector<ProductGraph::State*>& lead, Motion*& soln, double duration)
{
    bool solved = false;
    base::PlannerTerminationCondition ptc = base::timedPlannerTerminationCondition(duration);
    while (!ptc() && !solved)
    {
        ProductGraph::State* as = availDist_.sample(rng_.uniform01());
        ++abstractInfo_[as].numSel;
        updateWeight(as);

        PDF<Motion*>& motions = abstractInfo_[as].motions;
        Motion* v = motions.sample(rng_.uniform01());
        PDF<Motion*>::Element* velem = abstractInfo_[as].motionElems[v];
        double vweight = motions.getWeight(velem);
        if (vweight > 1e-20)
            motions.update(velem, vweight/(vweight+1.));

        Control* rctrl = siC_->allocControl();
        controlSampler_->sampleNext(rctrl, v->control, v->state);
        unsigned int cd = controlSampler_->sampleStepCount(siC_->getMinControlDuration(), siC_->getMaxControlDuration());

        base::State* newState = si_->allocState();
        cd = siC_->propagateWhileValid(v->state, rctrl, cd, newState);
        if (cd < siC_->getMinControlDuration())
        {
            si_->freeState(newState);
            siC_->freeControl(rctrl);
			continue;
        }
        Motion* m = new Motion();
        m->state = newState;
        m->control = siC_->allocControl();
        siC_->copyControl(m->control, rctrl);
        m->steps = cd;
        m->parent = v;
		m->abstractState = abstraction_->getState(m->parent->abstractState, m->state);
        // If we have created a state that will violate either automaton,
        // disregard the propagation.
        if (!m->abstractState->isValid())
        {
            siC_->freeControl(m->control);
            delete m;
            si_->freeState(newState);
            siC_->freeControl(rctrl);
			continue;
        }
        motions_.push_back(m);
        
        abstractInfo_[m->abstractState].addMotion(m);
        updateWeight(m->abstractState);
        // update weight if hl state already exists in avail
        if (abstractInfo_[m->abstractState].pdfElem != NULL)
            availDist_.update(abstractInfo_[m->abstractState].pdfElem, abstractInfo_[m->abstractState].weight);
        else
        {
            // otherwise, only add hl state to avail if it already exists in lead
			if (std::find(lead.begin(), lead.end(), m->abstractState) != lead.end())
            {
                PDF<ProductGraph::State*>::Element* elem = availDist_.add(m->abstractState, abstractInfo_[m->abstractState].weight);
                abstractInfo_[m->abstractState].pdfElem = elem;
            }
        }

        solved = abstraction_->isSolution(m->abstractState);
        if (solved)
        {
            soln = m;
            break;
        }
        siC_->freeControl(rctrl);
    }
    return solved;
}

double ompl::control::LTLPlanner::abstractEdgeWeight(ProductGraph::State* a, ProductGraph::State* b) const
{
    const ProductGraphStateInfo& infoA = abstractInfo_.find(a)->second;
    const ProductGraphStateInfo& infoB = abstractInfo_.find(b)->second;
    return 1./(infoA.weight * infoB.weight);
}

void ompl::control::LTLPlanner::clearMotions(void)
{
    availDist_.clear();
    for (std::vector<Motion*>::iterator i = motions_.begin(); i != motions_.end(); ++i)
    {
        Motion* m = *i;
        if (m->state != NULL)
            si_->freeState(m->state);
        if (m->control != NULL)
            siC_->freeControl(m->control);
        delete m;
    }
    motions_.clear();
    //TODO we've accepted that we'll only use one start state - do we even need starts_?
    starts_.clear();
    pis_.clear();
    pis_.update();
}
