# Multilevel Planning Tutorial {#multiLevelPlanningTutorial}

We show in this tutorial how to setup a multilevel planning task for a 2D rigid body in the plane. 

### Classical Planning 
We assume that you know how to setup a classical planning problem in OMPL. To recap, we need to define  
- `ompl::base::SpaceInformationPtr`, the state space
- `ompl::base::ProblemDefinitionPtr`, the start and goal state
- `ompl::base::PlannerPtr`, the planner used

We then construct and solve a planning problem as

~~~{.cpp}

ompl::base::SpaceInformationPtr si;
ompl::base::ProblemDefinitionPtr pdef; 

//...
// Set start and goal states to pdef
//...

ompl::base::PlannerPtr planner = std::make_shared<geometric::RRT>(si);
planner->setProblemDefinition(pdef);
planner->setup();
planner->solve();

~~~

### Multilevel Planning 

In a Multilevel planning setting, we change this routine to add a finite
number of levels of abstraction, which can be exploited by an algorithm (if it
supports it).

While the `ompl::base::ProblemDefinitionPtr` remains the same, we change the `ompl::base::SpaceInformationPtr` to a vector and the `ompl::base::PlannerPtr` to a multilevel planner. This means we define the following structures:

- `std::vector<ompl::base::SpaceInformationPtr>`, a vector of
  SpaceInformationPtr, sorted in
  ascending order depending on number of dimensions. Last element has to be the
original state space.
- `ompl::multilevel::QRRT`, a specific multilevel planner, has to inherit from `ompl::multilevel::BundleSpaceSequence.h`

A planning problem can then be solved as

~~~{.cpp}
std::vector<ompl::base::SpaceInformationPtr> siVec; 
ompl::base::ProblemDefinitionPtr pdef; 

//...
// Build siVec (see below), set start and goal states to pdef
//...

auto planner = std::make_shared<ompl::multilevel::QRRT>(siVec);

planner->setProblemDefinition(pdef);
planner->setup();
planner->solve();
~~~

### Building multiple levels of abstraction

Let us build a small sequence of two levels for a rigid body in 2D with
configuration space 
\f$SE(2)\f$. You can find this example [here](MultiLevelPlanningRigidBody2D_8cpp_source.html)

First, we need to construct the `ompl::base::SpaceInformationPtr` for each level. In
this case we use spaces
\f$\{\mathbb{R}^2, SE(2)\}\f$, corresponding to an abstraction by projecting
onto the position of the rigid body. The space is bounded to be in the unit square
\f$[0,1]^2\f$

~~~{.cpp}

auto SE2(std::make_shared<ompl::base::SE2StateSpace>());
ompl::base::RealVectorBounds bounds(2);
bounds.setLow(0);
bounds.setHigh(1);
SE2->setBounds(bounds);
ompl::base::SpaceInformationPtr si_SE2(std::make_shared<ompl::base::SpaceInformation>(SE2));

auto R2(std::make_shared<ompl::base::RealVectorStateSpace>(2));
R2->setBounds(0, 1);
ompl::base::SpaceInformationPtr si_R2(std::make_shared<ompl::base::SpaceInformation>(R2));
~~~

Second, we need to set the `ompl::base::StateValidityChecker` for each
`ompl::base::SpaceInformationPtr`. This is done by defining a validity function
which evaluates to true if a state is feasible and false if it is infeasible. 

For the \f$SE(2)\f$ constraint, we declare all configurations feasible where the
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
bool isStateValid_SE2(const ompl::base::State *state) 
{
    const auto *SE2state = state->as<ompl::base::SE2StateSpace::StateType>();
    const auto *R2 = SE2state->as<ompl::base::RealVectorStateSpace::StateType>(0);
    const auto *SO2 = SE2state->as<ompl::base::SO2StateSpace::StateType>(1);
    return boxConstraint(R2->values) && (SO2->value < boost::math::constants::pi<double>() / 2.0);
}
~~~

The validity function for the simplified robot can be constructed by removing some of the constraints. In real experiments, we often remove links or parts of links from a robot, which implicitly removes the constraints. We choose the following constraint

~~~{.cpp}
bool isStateValid_R2(const ompl::base::State *state) 
{ 
    const auto *R2 = state->as<ompl::base::RealVectorStateSpace::StateType>();
    return boxConstraint(R2->values);
}
~~~

Finally, we set the validity function and push the
`ompl::base::SpaceInformationPtr` into siVec. Note that the
`ompl::base::SpaceInformationPtr` have to be sorted in ascending order depending
on number of dimensions. 

~~~{.cpp}
si_SE2->setStateValidityChecker(isStateValid_SE2);
si_R2->setStateValidityChecker(isStateValid_R2);

siVec.push_back(si_R2);
siVec.push_back(si_SE2);
~~~

NOTE: The runtime of a multilevel planner depends crucially on the sequence
of defined. For some spaces, planning time can be very fast,
while for others it is can still be outperformed by classical planner such as
ompl::geometric::RRT (which is equivalent to running ompl::multilevel::QRRT with
a single configuration space). Which spaces work best is still an open research
question. A good heuristic is to use more levels the more narrow
passages we have in an environment. More information can be found in the [QRRT
paper](https://arxiv.org/abs/1906.01350).

### AnnotatedPlannerDataVertex

Having defined a Multilevel planner, we can utilize it similar to a usual
ompl::base::PlannerPtr, i.e. it can be added to any benchmark, and we can get
the planner data.

However, the classical PlannerData structure does not know about multiple levels
of abstraction. To add
this information, we wrote the class `ompl::multilevel::PlannerDataVertexAnnotated`, which
inherits from PlannerDataVertex. PlannerDataVertexAnnotated adds new
functionalities, including

- `getLevel()`, returns the level the current vertex is on
- `getMaxLevel()`, returns the number of levels
- `getBaseState()`, returns a state on the current
  vertex level
- `getState()`, returns a state in the original configuration space (this way
    the behavior is the same as for the classical PlannerData class).


You can use this in your code in the following way:

~~~{.cpp}
    #include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>

    ompl::multilevel::PlannerDataVertexAnnotated *v = 
    dynamic_cast<ompl::multilevel::PlannerDataVertexAnnotated*>(&pd->getVertex(0));
if(v!=nullptr){
    unsigned int level = v->getLevel();
    unsigned int maxLevel = v->getMaxLevel();
    const ompl::base::State *s_BaseState = v->getBaseState();
    const ompl::base::State *s_ConfigurationState = v->getState();
}
~~~
### Further Information

For more information, please refer to either the general introduction to [Multilevel Planning](multiLevelPlanning.html) or the [demos](group__demos.html).
