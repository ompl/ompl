#include "ompl/control/planners/ltl/LTLPlanner.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/planners/ltl/ProductGraph.h"
#include "ompl/control/planners/ltl/LTLProblemDefinition.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/util/Console.h"
#include <algorithm>
#include <boost/unordered_map.hpp>
#include <limits>
#include <map>
#include <vector>

#include <stdio.h>

ompl::control::LTLPlanner::LTLPlanner(const LTLSpaceInformationPtr& ltlsi, const ProductGraphPtr& a, double exploreTime) :
    ompl::base::Planner(ltlsi, "LTLPlanner"),
    ltlsi_(ltlsi.get()),
    abstraction_(a),
    prodStart_(NULL),
    exploreTime_(exploreTime)
{
    specs_.approximateSolutions = true;
}

ompl::control::LTLPlanner::~LTLPlanner(void)
{
    clearMotions();
}

void ompl::control::LTLPlanner::setup()
{
    base::Planner::setup();
}

void ompl::control::LTLPlanner::clear()
{
    base::Planner::clear();
    availDist_.clear();
    abstractInfo_.clear();
    clearMotions();
}

ompl::base::PlannerStatus ompl::control::LTLPlanner::solve(const ompl::base::PlannerTerminationCondition& ptc)
{
    // TODO make solve work when called more than once!
    checkValidity();
    const base::State* start = pis_.nextStart();
    prodStart_ = ltlsi_->getProdGraphState(start);

    if (pis_.haveMoreStartStates())
        OMPL_WARN("Multiple start states given. Using only the first start state.");

    Motion* startMotion = new Motion(ltlsi_);
    si_->copyState(startMotion->state, start);
    ltlsi_->nullControl(startMotion->control);
    startMotion->abstractState = prodStart_;

    motions_.push_back(startMotion);
    abstractInfo_[prodStart_].addMotion(startMotion);
    updateWeight(prodStart_);
    availDist_.add(prodStart_, abstractInfo_[prodStart_].weight);

    abstraction_->buildGraph(prodStart_, boost::bind(&LTLPlanner::initAbstractInfo, this, _1));

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = ltlsi_->allocControlSampler();

    bool solved = false;
    Motion* soln;

    while (ptc()==false && !solved)
    {
        const std::vector<ProductGraph::State*> lead = abstraction_->computeLead(prodStart_, boost::bind(&LTLPlanner::abstractEdgeWeight, this, _1, _2));
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
            if (path[i]->parent != NULL) {
                pc->append(path[i]->state, path[i]->control, path[i]->steps * ltlsi_->getPropagationStepSize());
            }
            else {
                pc->append(path[i]->state);
            }
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
    hlPath[0] = (start != NULL ? start : ltlsi_->getProdGraphState(path[0]));
    for (unsigned int i = 1; i < path.size(); ++i)
    {
        hlPath[i] = ltlsi_->getProdGraphState(path[i]);
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
    base::GoalPtr goal = pdef_->getGoal();
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

        Control* rctrl = ltlsi_->allocControl();
        controlSampler_->sampleNext(rctrl, v->control, v->state);
        unsigned int cd = controlSampler_->sampleStepCount(ltlsi_->getMinControlDuration(), ltlsi_->getMaxControlDuration());

        base::State* newState = si_->allocState();
        cd = ltlsi_->propagateWhileValid(v->state, rctrl, cd, newState);
        if (cd < ltlsi_->getMinControlDuration())
        {
            si_->freeState(newState);
            ltlsi_->freeControl(rctrl);
			continue;
        }
        Motion* m = new Motion();
        m->state = newState;
        m->control = rctrl;
        m->steps = cd;
        m->parent = v;
        // Since the state was determined to be valid by SpaceInformation, we don't need to check automaton states
        m->abstractState = ltlsi_->getProdGraphState(m->state);
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

        solved = goal->isSatisfied(m->state);
        if (solved)
        {
            soln = m;
            break;
        }
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
            ltlsi_->freeControl(m->control);
        delete m;
    }
    motions_.clear();
    pis_.clear();
    pis_.update();
}
