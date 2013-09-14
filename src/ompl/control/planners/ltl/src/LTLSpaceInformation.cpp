#include "ompl/control/planners/ltl/LTLSpaceInformation.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/StatePropagator.h"
#include "ompl/control/planners/ltl/ProductGraph.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace
{
    static ob::StateSpacePtr extendStateSpace(const ob::StateSpacePtr& lowSpace,
                                              const oc::ProductGraphPtr& prod);
}

oc::LTLSpaceInformation::LTLSpaceInformation(const oc::SpaceInformationPtr& si,
                                             const oc::ProductGraphPtr& prod)
    : oc::SpaceInformation(extendStateSpace(si->getStateSpace(), prod),
                           si->getControlSpace()), prod_(prod), lowSpace_(si)
{
    //TODO bug - control space points to old SI instead of new SI!
    // In practice, this is fine, since control space never actually uses
    // the SI that was passed to it in its constructor.
    extendPropagator(si);
    extendValidityChecker(si);
}

oc::LTLSpaceInformation::~LTLSpaceInformation(void)
{
}

void oc::LTLSpaceInformation::extendPropagator(const oc::SpaceInformationPtr& oldsi)
{
    class LTLStatePropagator : public oc::StatePropagator
    {
    public:
        LTLStatePropagator(oc::LTLSpaceInformation* ltlsi,
                           const oc::ProductGraphPtr& prod,
                           const oc::StatePropagatorPtr& lowProp)
            : oc::StatePropagator(ltlsi), prod_(prod), lowProp_(lowProp), ltlsi_(ltlsi)
        {
        }
        virtual ~LTLStatePropagator() { }

        // TODO what to do with other methods defined in statePropagator?
        virtual void propagate(const ob::State* state, const oc::Control* control,
                               const double duration, ob::State* result) const
        {
            const ob::State* lowLevelPrev = ltlsi_->getLowLevelState(state);
            ob::State* lowLevelResult = ltlsi_->getLowLevelState(result);
            lowProp_->propagate(lowLevelPrev, control, duration, lowLevelResult);
            const oc::ProductGraph::State* prevHigh = ltlsi_->getProdGraphState(state);
            const oc::ProductGraph::State* nextHigh = prod_->getState(prevHigh, lowLevelResult);
            result->as<ob::CompoundState>()->as
                <ob::DiscreteStateSpace::StateType>(REGION)->value = nextHigh->getDecompRegion();
            result->as<ob::CompoundState>()->as
                <ob::DiscreteStateSpace::StateType>(COSAFE)->value = nextHigh->getCosafeState();
            result->as<ob::CompoundState>()->as
                <ob::DiscreteStateSpace::StateType>(SAFE)->value = nextHigh->getSafeState();
        }
    private:
        const oc::ProductGraphPtr prod_;
        const oc::StatePropagatorPtr lowProp_;
        const oc::LTLSpaceInformationPtr ltlsi_;
    };

    // Some compilers have trouble with LTLStatePropagator being hidden in this function,
    // and so we explicitly cast it to its base type.
    setStatePropagator(oc::StatePropagatorPtr(static_cast<oc::StatePropagator*>(
        new LTLStatePropagator(this, prod_, oldsi->getStatePropagator()))));
}

void oc::LTLSpaceInformation::extendValidityChecker(const oc::SpaceInformationPtr& oldsi)
{
    class LTLStateValidityChecker : public ob::StateValidityChecker
    {
    public:
        LTLStateValidityChecker(oc::LTLSpaceInformation* ltlsi,
                                const oc::ProductGraphPtr& prod,
                                const ob::StateValidityCheckerPtr& lowChecker)
            : ob::StateValidityChecker(ltlsi), prod_(prod), lowChecker_(lowChecker), ltlsi_(ltlsi)
        {
        }
        virtual ~LTLStateValidityChecker() { }
        virtual bool isValid(const base::State* s) const
        {
            return ltlsi_->getProdGraphState(s)->isValid()
                && lowChecker_->isValid(ltlsi_->getLowLevelState(s));
        }
    private:
        const oc::ProductGraphPtr prod_;
        const ob::StateValidityCheckerPtr lowChecker_;
        const oc::LTLSpaceInformationPtr ltlsi_;
    };

    // Some compilers have trouble with LTLStateValidityChecker being hidden in this function,
    // and so we explicitly cast it to its base type.
    setStateValidityChecker(ob::StateValidityCheckerPtr(static_cast<ob::StateValidityChecker*>(
        new LTLStateValidityChecker(this, prod_, oldsi->getStateValidityChecker()))));
}

namespace
{
    ob::StateSpacePtr extendStateSpace(const ob::StateSpacePtr& lowSpace,
                                       const oc::ProductGraphPtr& prod)
    {
        const oc::AutomatonPtr cosafe (prod->getCosafetyAutom());
        const oc::AutomatonPtr safe (prod->getSafetyAutom());
        ob::StateSpacePtr regionSpace(new ob::DiscreteStateSpace(0, prod->getDecomp()->getNumRegions()-1));
        ob::StateSpacePtr cosafeSpace (new ob::DiscreteStateSpace(0, cosafe->numStates()-1));
        ob::StateSpacePtr safeSpace (new ob::DiscreteStateSpace(0, safe->numStates()-1));
        //TODO this may not work correctly if lowSpace is compound and lock() has not been called
        // if dyn_cast<Compound>(lowSpace), then ensure lock() has been called
        return lowSpace + regionSpace + cosafeSpace + safeSpace;
    }
}
