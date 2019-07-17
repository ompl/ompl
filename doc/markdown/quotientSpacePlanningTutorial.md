# QuotientSpace Planning Tutorial {#quotientSpacePlanningTutorial}

We show in this tutorial how the setup of a QuotientSpace planning task differs
from a classical ConfigurationSpace planning task. Finally, we show how the
PlannerData can be extracted from the planner.

### Classical Configuration Space Planning 
A classical planning problem in OMPL is defined by the following classes
- `ompl::base::SpaceInformationPtr`, the configuration space
- `ompl::base::ProblemDefinitionPtr`, the start and goal configuration
- `ompl::base::PlannerPtr`, the planner used

Having defined those classes, a solve routine (e.g. using RRT) looks roughly like that

~~~{.cpp}

ompl::base::SpaceInformationPtr si;
ompl::base::ProblemDefinitionPtr pdef; 

//...
// Set start and goal states to pdef
//...

ompl::base::PlannerPtr planner = std::make_shared<ompl::geometric::RRT>(si);
planner->setProblemDefinition(pdef);
planner->setup();
planner->solve();

~~~

### QuotientSpace Planning 

In a QuotientSpace planning setting, we change this routine to add a finite
number of QuotientSpaces, which can be exploited by an algorithm (if it
supports it).

We therefore need in addition
- `std::vector<ompl::base::SpaceInformationPtr>`, the QuotientSpaces and the
  configuration space as last element
- `std::vector<ompl::base::ProblemDefinitionPtr>`, the start and goal configuration for each QuotientSpace
- `ompl::geometric::MultiQuotient`, the manager class which decides when to explore a
  QuotientSpace and when to switch to another QuotientSpace
- `ompl::geometric::QuotientSpace`, the actual implementation of a QuotientSpace.
  Every planner needs to inherit this class. You are not allowed to call solve() on this
class, but you rather need to use ompl::geometric::MultiQuotient to manage the
QuotientSpaces.
- `ompl::geometric::QRRT`, a specific QuotientSpace planner which inherits from
  ompl::geometric::QuotientSpace

Having defined those classes, we can solve a new planning problem as

~~~{.cpp}
std::vector<ompl::base::SpaceInformationPtr> si_vec; 
std::vector<ompl::base::ProblemDefinitionPtr> pdef_vec; 

//...
// Set QuotientSpaces in si_vec, set start and goal states to pdef_vec
//...

typedef og::MultiQuotient<og::QRRT> MultiQuotient;
auto planner = std::make_shared<MultiQuotient>(si_vec);
planner->setProblemDefinition(pdef_vec);

planner->setup();
planner->solve();

~~~

NOTE: The runtime of a QuotientSpace planner depends crucially on the sequence
of QuotientSpaces defined. For some spaces, planning time can be very fast,
while for others it is easily outperformed by classical planner such as
ompl::geometric::RRT. Which spaces work best is still a research question.

### AnnotatedPlannerDataVertex

Having defined a QuotientSpace planner, we can utilize it similar to a usual
ompl::base::PlannerPtr, i.e. it can be added to any benchmark, and we can get
the planner data.

However, the classical PlannerData does not know about QuotientSpaces. To add
QuotientSpace information, we have written a new class ompl::base::PlannerDataVertexAnnotated, which
inherits from PlannerDataVertex. PlannerDataVertexAnnotated adds new
functionalities, including

- `getLevel()`, returns the QuotientSpace level the current vertex is on
- `getMaxLevel()`, returns the number of QuotientSpace levels
- `getQuotientState()`, returns a QuotientSpace state on the current
  vertex level
- `getState()`, returns a state in the original configuration space (this way
    the behavior is the same as for the classical PlannerData class).

~~~{.cpp}
#include <ompl/geometric/planners/quotientspace/PlannerDataVertexAnnotated.h>

PlannerDataVertexAnnotated *v = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(0));
if(v!=nullptr){
    unsigned int level = v->getLevel();
    unsigned int maxLevel = v->getMaxLevel();
    const ob::State *s_QuotientSpace = v->getQuotientState();
    const ob::State *s_ConfigurationSpace = v->getState();
}
~~~
### Further Information

For more information, please refer to either the general introduction to [QuotientSpaces](quotientSpacePlanning.html) or the [demos](group__demos.html).
