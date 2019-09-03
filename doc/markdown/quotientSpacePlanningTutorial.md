# QuotientSpace Planning Tutorial {#quotientSpacePlanningTutorial}

We show in this tutorial how the setup of a QuotientSpace planning task differs
from a classical ConfigurationSpace planning task. Finally, we show how the
PlannerData can be extracted from the QuotientSpace planner.

### Classical Configuration Space Planning 
A classical planning problem in OMPL is defined by 
- `ompl::base::SpaceInformationPtr`, the configuration space
- `ompl::base::ProblemDefinitionPtr`, the start and goal configuration
- `ompl::base::PlannerPtr`, the planner used

We then construct and solve a planning problem as

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
- `std::vector<ompl::base::SpaceInformationPtr>`, the QuotientSpaces. Sorted in
  ascending order depending on number of dimensions. Last element has to be the
  Configuration Space.
- `ompl::geometric::QRRT`, a specific QuotientSpace planner which inherits from
  ompl::geometric::QuotientSpace

A planning problem can then be solved as

~~~{.cpp}
std::vector<ompl::base::SpaceInformationPtr> siVec; 
ompl::base::ProblemDefinitionPtr pdef; 

//...
// Build QuotientSpaces in siVec (see below), set start and goal states to pdef
//...

auto planner = std::make_shared<og::QRRT>(siVec);

planner->setProblemDefinition(pdef);
planner->setup();
planner->solve();
~~~

### Building QuotientSpaces

Let us build a small sequence of two QuotientSpaces for a Rigid Body in 2D with
configuration space 
\f$SE(2)\f$. This
example has been implemented in demos/QuotientSpacePlanningRigidBody2D.cpp

First, we need to construct the SpaceInformationPtr for each QuotientSpace. In
this case, we opt for two layers 
\f$\{\mathbb{R}^2, SE(2)\}\f$. The space is bounded to be in the unit square
\f$[0,1]^2\f$

~~~{.cpp}

auto SE2(std::make_shared<ob::SE2StateSpace>());
ob::RealVectorBounds bounds(2);
bounds.setLow(0);
bounds.setHigh(1);
SE2->setBounds(bounds);
ob::SpaceInformationPtr si_SE2(std::make_shared<ob::SpaceInformation>(SE2));

auto R2(std::make_shared<ob::RealVectorStateSpace>(2));
R2->setBounds(0, 1);
ob::SpaceInformationPtr si_R2(std::make_shared<ob::SpaceInformation>(R2));
~~~

Second, we need to set the validitychecker for each SpaceInformationPtr. This is
done by defining a Validity function which evaluates to true if a State is
feasible and false if it is infeasible. 

For the SE(2) constraint, we declare all configurations feasible where the
rotation is smaller than \f$\frac{\pi}{2}\f$ and where the position is not in a
circle of radius \f$0.2\f$ located at \f$(0.5,0.5)\f$.

~~~{.cpp}
bool boxConstraint(const double values[])
{
    const double &x = values[0]-0.5;
    const double &y = values[1]-0.5;
    double pos_cnstr = sqrt(x*x + y*y);
    return (pos_cnstr > 0.2);
}
bool isStateValid_SE2(const ob::State *state) 
{
    const auto *SE2state = state->as<ob::SE2StateSpace::StateType>();
    const auto *R2 = SE2state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *SO2 = SE2state->as<ob::SO2StateSpace::StateType>(1);
    return boxConstraint(R2->values) && (SO2->value < boost::math::constants::pi<double>() / 2.0);
}
~~~

The validity function for the QuotientSpace can be constructed for example by removing some of the constraints. In real experiments, we often remove links or parts of links from a robot, which implicitly removes the constraints.

~~~{.cpp}
bool isStateValid_R2(const ob::State *state) 
{ 
    const auto *R2 = state->as<ob::RealVectorStateSpace::StateType>();
    return boxConstraint(R2->values);
}
~~~

Finally, we need to set the Validity function to and push the
SpaceInformationPtr into the vector. Note that the SpaceInformationPtr have to
be sorted in ascending order depending on number of dimensions. 

~~~{.cpp}
si_SE2->setStateValidityChecker(isStateValid_SE2);
si_R2->setStateValidityChecker(isStateValid_R2);

siVec.push_back(si_R2);
siVec.push_back(si_SE2);
~~~

NOTE: The runtime of a QuotientSpace planner depends crucially on the sequence
of QuotientSpaces defined. For some spaces, planning time can be very fast,
while for others it is easily outperformed by classical planner such as
ompl::geometric::RRT (which is equivalent to running ompl::geometric::QRRT with
a single configuration space). Which spaces work best is still an open research
question. A good heuristic is to use more QuotientSpaces the more narrow
passages we have in an environment. More information can be found in the [QRRT
paper](https://arxiv.org/abs/1906.01350).

### AnnotatedPlannerDataVertex

Having defined a QuotientSpace planner, we can utilize it similar to a usual
ompl::base::PlannerPtr, i.e. it can be added to any benchmark, and we can get
the planner data.

However, the classical PlannerData structure does not know about QuotientSpaces. To add
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
