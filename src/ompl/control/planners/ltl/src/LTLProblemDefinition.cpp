#include "ompl/control/PathControl.h"
#include "ompl/control/planners/ltl/LTLProblemDefinition.h"
#include "ompl/control/planners/ltl/LTLSpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

ob::LTLProblemDefinition::LTLProblemDefinition(const oc::LTLSpaceInformationPtr& ltlsi)
    : ob::ProblemDefinition(ltlsi), ltlsi_(ltlsi)
{
    createGoal();
}

ob::LTLProblemDefinition::~LTLProblemDefinition(void)
{
}

void ob::LTLProblemDefinition::addLowerStartState(const base::State* s)
{
    base::ScopedState<base::CompoundStateSpace> fullStart(si_);
    ltlsi_->getFullState(s, fullStart.get());
    addStartState(fullStart);
}

void ob::LTLProblemDefinition::createGoal(void)
{
    class LTLGoal : public Goal
    {
    public:
        LTLGoal(const oc::LTLSpaceInformationPtr& ltlsi)
            : Goal(ltlsi), ltlsi_(ltlsi), prod_(ltlsi->getProductGraph())
        {
        }
        virtual ~LTLGoal(void) { }
        virtual bool isSatisfied(const base::State* s) const
        {
            return prod_->isSolution(ltlsi_->getProdGraphState(s));
        }
    protected:
        const oc::LTLSpaceInformationPtr ltlsi_;
        const oc::ProductGraphPtr prod_;
    };

    // Some compilers have trouble with LTLGoal being hidden in this function,
    // and so we explicitly cast it to its base type.
    setGoal(ob::GoalPtr(static_cast<ob::Goal*>(new LTLGoal(ltlsi_))));
}

ob::PathPtr ob::LTLProblemDefinition::getLowerSolutionPath(void) const
{
    oc::PathControl* fullPath = static_cast<oc::PathControl*>(getSolutionPath().get());
    ob::PathPtr lowPathPtr(new oc::PathControl(ltlsi_->getLowSpace()));
    oc::PathControl* lowPath = static_cast<oc::PathControl*>(lowPathPtr.get());
    lowPath->getControls().assign(fullPath->getControls().begin(), fullPath->getControls().end());
    lowPath->getControlDurations().assign(
        fullPath->getControlDurations().begin(), fullPath->getControlDurations().end());
    for (std::size_t i = 0; i < fullPath->getStateCount(); ++i)
    {
        lowPath->append(ltlsi_->getLowSpace()->allocState());
        ltlsi_->getLowSpace()->copyState(lowPath->getState(i),
            ltlsi_->getLowLevelState(fullPath->getState(i)));
    }
    return lowPathPtr;
}
