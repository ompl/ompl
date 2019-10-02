# Implementing a New Motion Planner {#newPlanner}

## Strict Requirements

Implementing a new motion planner is very simple using OMPL.  There are just two strict requirements:

- Publicly derive the new class from ompl::base::Planner
- Provide an implementation of the method solve() from ompl::base::Planner.

By satisfying these requirements, the planner can be fully integrated within the existing OMPL framework.

## Optional Features

Aside from the strict requirements, there are other methods which can be implemented and practices which should be followed for ease of integration. These are not required, but are strongly recommended for simplicity and consistency:

- Update the values of the ompl::base::PlannerSpecs member of ompl::base::Planner in the constructor of your planner to indicate its capabilities to the user.
- Overload ompl::base::Planner::setup (if needed) to execute any one-time setup methods. Note that setup() is only guaranteed to be called once, and is not likely to be invoked before each call to solve().
- When a solution path is found in the solve() method, save it to the instance of ompl::base::ProblemDefinition using its addSolutionPath member.  ompl::base::ProblemDefinition is a member of ompl::base::Planner.
- Return an informative value from ompl::base::PlannerStatus in the implementation of solve().
- solve() should respect the ompl::base::PlannerTerminationCondition argument passed to it. When the given condition evaluates true, solve() should return as quickly as possible.
- Repeated calls to solve() should not restart the planning process from scratch, but rather pick up the search where it left off previously.
- Provide an implementation of ompl::base::Planner::clear().  This method should free any memory allocated by the planner and restore the planner to a state where ompl::base::Planner::solve() can be called again (without passing on information from previous calls to solve() ). If necessary, clear() can set ompl::base::Planner::setup_ to `false`, to communicate to solve() that setup() needs to be called again.
- Provide an implementation of ompl::bas::Planner::getPlannerData() which translates the internal planner data structure to the ompl::base::PlannerData graph implementation. This method is particularly useful for debugging purposes since it allows the user to inspect the data structure.

## New Planner Template

The following is a template which can be used to craft a new ompl::base::Planner object:

~~~{.cpp}
#include <ompl/base/Planner.h>

// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>

namespace ompl
{
    class myNewPlanner : public base::Planner
    {
    public:

        myNewPlanner(const base::SpaceInformationPtr &si) : base::Planner(si, "the planner's name")
        {
            // the specifications of this planner (ompl::base::PlannerSpecs)
            specs_.approximateSolutions = ...;
            specs_.recognizedGoal = ...;
            ...
        }

        virtual ~myNewPlanner(void)
        {
            // free any allocated memory
        }

        virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc)
        {
            // make sure the planner is configured correctly; ompl::base::Planner::checkValidity
            // ensures that there is at least one input state and a ompl::base::Goal object specified
            checkValidity();

            // get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
            base::Goal *goal = pdef_->getGoal().get();

            // get input states with PlannerInputStates helper, pis_
            while (const base::State *st = pis_.nextStart())
            {
                // st will contain a start state.  Typically this state will
                // be cloned here and inserted into the Planner's data structure.
            }

            // if needed, sample states from the goal region (and wait until a state is sampled)
            const base::State *st = pis_.nextGoal(ptc);
            // or sample a new goal state only if available:
            const base::State *st = pis_.nextGoal();

            // periodically check if ptc() returns true.
            // if it does, terminate planning.
            while (ptc() == false)
            {
                // Start planning here.

                // call routines from SpaceInformation (si_) as needed. i.e.,
                // si_->allocStateSampler() for sampling,
                // si_->checkMotion(state1, state2) for state validity, etc...

                // use the Goal pointer to evaluate whether a sampled state satisfies the goal requirements

                // use log macros for informative messaging, i.e., logInfo("Planner found a solution!");
            }

            // When a solution path is computed, save it here
            pdef_->addSolutionPath(...);

            // Return a value from the PlannerStatus enumeration.
            // See ompl::base::PlannerStatus for the possible return values
            return base::PlannerStatus::EXACT_SOLUTION;
        }

        virtual void clear(void)
        {
            Planner::clear();
            // clear the data structures here
        }

        // optional, if additional setup/configuration is needed, the setup() method can be implemented
        virtual void setup(void)
        {
            Planner::setup();

            // perhaps attempt some auto-configuration
            SelfConfig sc(si_, getName());
            sc.configure...
        }

        virtual void getPlannerData(base::PlannerData &data) const
        {
            // fill data with the states and edges that were created
            // in the exploration data structure
            // perhaps also fill control::PlannerData
        }

    };
}
~~~
