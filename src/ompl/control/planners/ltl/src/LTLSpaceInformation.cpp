#include <utility>

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
    // Helper method to take a robot state space and product graph and return
    // the hybrid state space representing their product.
    static ob::StateSpacePtr extendStateSpace(const ob::StateSpacePtr &lowSpace, const oc::ProductGraphPtr &prod);
}

oc::LTLSpaceInformation::LTLSpaceInformation(const oc::SpaceInformationPtr &si, const oc::ProductGraphPtr &prod)
  : oc::SpaceInformation(extendStateSpace(si->getStateSpace(), prod), si->getControlSpace()), prod_(prod), lowSpace_(si)
{
    //\todo: Technically there's a bug here, as we've assigning LTLSpaceInformation's
    //      control space to be si->getControlSpace(), which internally holds a pointer
    //      to si->getStateSpace() instead of this->getStateSpace(). In practice, this
    //      is fine for now, since control space never actually uses its internal state
    //      space pointer.
    extendPropagator(si);
    extendValidityChecker(si);
}

void oc::LTLSpaceInformation::setup()
{
    // Set up the low space, then match our parameters to it.
    if (!lowSpace_->isSetup())
        lowSpace_->setup();
    // We never actually use the below parameters in LTLSpaceInformation while planning.
    // All integrating is done in lowSpace. However, we will need these parameters when
    // printing the path - PathControl::print() will convert path steps using these
    // parameters.
    setMinMaxControlDuration(lowSpace_->getMinControlDuration(), lowSpace_->getMaxControlDuration());
    setPropagationStepSize(lowSpace_->getPropagationStepSize());
    setup_ = true;
}

void oc::LTLSpaceInformation::getFullState(const ob::State *low, ob::State *full)
{
    const ProductGraph::State *high = prod_->getState(low);
    ob::CompoundState &cs = *full->as<ob::CompoundState>();
    stateSpace_->as<ob::CompoundStateSpace>()->getSubspace(LOW_LEVEL)->copyState(cs[LOW_LEVEL], low);
    using DiscreteState = ob::DiscreteStateSpace::StateType;
    cs[REGION]->as<DiscreteState>()->value = high->getDecompRegion();
    cs[COSAFE]->as<DiscreteState>()->value = high->getCosafeState();
    cs[SAFE]->as<DiscreteState>()->value = high->getSafeState();
}

ob::State *oc::LTLSpaceInformation::getLowLevelState(ob::State *s)
{
    return const_cast<ob::State *>(getLowLevelState(const_cast<const ob::State *>(s)));
}

const ob::State *oc::LTLSpaceInformation::getLowLevelState(const ob::State *s)
{
    return s->as<ob::CompoundState>()->operator[](LOW_LEVEL);
}

oc::ProductGraph::State *oc::LTLSpaceInformation::getProdGraphState(const ob::State *s) const
{
    const ob::CompoundState &cs = *s->as<ob::CompoundState>();
    using DiscreteState = ob::DiscreteStateSpace::StateType;
    return prod_->getState(cs[REGION]->as<DiscreteState>()->value, cs[COSAFE]->as<DiscreteState>()->value,
                           cs[SAFE]->as<DiscreteState>()->value);
}

void oc::LTLSpaceInformation::extendPropagator(const oc::SpaceInformationPtr &oldsi)
{
    class LTLStatePropagator : public oc::StatePropagator
    {
    public:
        LTLStatePropagator(oc::LTLSpaceInformation *ltlsi, oc::ProductGraphPtr prod, oc::StatePropagatorPtr lowProp)
          : oc::StatePropagator(ltlsi), prod_(std::move(prod)), lowProp_(std::move(lowProp)), ltlsi_(ltlsi)
        {
        }
        ~LTLStatePropagator() override = default;

        void propagate(const ob::State *state, const oc::Control *control, const double duration,
                       ob::State *result) const override
        {
            const ob::State *lowLevelPrev = ltlsi_->getLowLevelState(state);
            ob::State *lowLevelResult = ltlsi_->getLowLevelState(result);
            lowProp_->propagate(lowLevelPrev, control, duration, lowLevelResult);
            const oc::ProductGraph::State *prevHigh = ltlsi_->getProdGraphState(state);
            const oc::ProductGraph::State *nextHigh = prod_->getState(prevHigh, lowLevelResult);
            result->as<ob::CompoundState>()->as<ob::DiscreteStateSpace::StateType>(REGION)->value =
                nextHigh->getDecompRegion();
            result->as<ob::CompoundState>()->as<ob::DiscreteStateSpace::StateType>(COSAFE)->value =
                nextHigh->getCosafeState();
            result->as<ob::CompoundState>()->as<ob::DiscreteStateSpace::StateType>(SAFE)->value =
                nextHigh->getSafeState();
        }

        bool canPropagateBackward() const override
        {
            return lowProp_->canPropagateBackward();
        }

    private:
        const oc::ProductGraphPtr prod_;
        const oc::StatePropagatorPtr lowProp_;
        oc::LTLSpaceInformation *ltlsi_;
    };

    // Some compilers have trouble with LTLStatePropagator being hidden in this function,
    // and so we explicitly cast it to its base type.
    setStatePropagator(std::make_shared<LTLStatePropagator>(this, prod_, oldsi->getStatePropagator()));
}

void oc::LTLSpaceInformation::extendValidityChecker(const oc::SpaceInformationPtr &oldsi)
{
    class LTLStateValidityChecker : public ob::StateValidityChecker
    {
    public:
        LTLStateValidityChecker(oc::LTLSpaceInformation *ltlsi, oc::ProductGraphPtr prod,
                                ob::StateValidityCheckerPtr lowChecker)
          : ob::StateValidityChecker(ltlsi), prod_(std::move(prod)), lowChecker_(std::move(lowChecker)), ltlsi_(ltlsi)
        {
        }
        ~LTLStateValidityChecker() override = default;
        bool isValid(const ob::State *s) const override
        {
            return ltlsi_->getProdGraphState(s)->isValid() && lowChecker_->isValid(ltlsi_->getLowLevelState(s));
        }

    private:
        const oc::ProductGraphPtr prod_;
        const ob::StateValidityCheckerPtr lowChecker_;
        oc::LTLSpaceInformation *ltlsi_;
    };

    // Some compilers have trouble with LTLStateValidityChecker being hidden in this function,
    // and so we explicitly cast it to its base type.
    setStateValidityChecker(std::make_shared<LTLStateValidityChecker>(this, prod_, oldsi->getStateValidityChecker()));
}

namespace
{
    ob::StateSpacePtr extendStateSpace(const ob::StateSpacePtr &lowSpace, const oc::ProductGraphPtr &prod)
    {
        const oc::AutomatonPtr cosafe(prod->getCosafetyAutom());
        const oc::AutomatonPtr safe(prod->getSafetyAutom());
        auto regionSpace(std::make_shared<ob::DiscreteStateSpace>(0, prod->getDecomp()->getNumRegions() - 1));
        auto cosafeSpace(std::make_shared<ob::DiscreteStateSpace>(0, cosafe->numStates() - 1));
        auto safeSpace(std::make_shared<ob::DiscreteStateSpace>(0, safe->numStates() - 1));

        auto compound(std::make_shared<ob::CompoundStateSpace>());
        compound->addSubspace(lowSpace, 1.);
        compound->addSubspace(regionSpace, 0.);
        compound->addSubspace(cosafeSpace, 0.);
        compound->addSubspace(safeSpace, 0.);
        compound->lock();

        return compound;
    }
}
