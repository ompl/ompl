# Use of Projections in OMPL {#projections}

Most planning algorithms use a data structure (in addition to the tree/graph of motions) for guiding the exploration of the space. In many cases, this data structure is in the form of a discretization of the state space. For practical purposes, discretizing the state space itself is not feasible, due to its high dimensionality. An approach taken by many algorithms is to use projections from the state space to a low dimensional Euclidean space (R<sup>k</sup>, k usually around 2 or 3), and discretize that Euclidean space instead. This includes algorithms such as ompl::geometric::ProjEST, ompl::geometric::SBL, ompl::geometric::KPIECE1. This projection can also be thought of as a hash function that maps states to cells in a low dimensional Euclidean space.

To support this notion of projections, OMPL includes ompl::base::ProjectionEvaluator. This is an abstract class whose main operation is ompl::base::ProjectionEvaluator::project(), which takes a ompl::base::State* and produces an Eigen::VectorXd (this is a representation of a vector of real numbers).

ompl::base::ProjectionEvaluator also includes utilities for discretizing the projected space (the low dimensional, Euclidean one). In particular, grids can be implicitly imposed on the projection space by setting cell sizes (a real value for each dimension of the Euclidean space), using ompl::base::ProjectionEvaluator::setCellSizes(). The coordinates in the grid that correspond to a specific projection can be computed using ompl::base::ProjectionEvaluator::computeCoordinates(). If the sizes of cells for the grid have not been set, ompl::base::ProjectionEvaluator::setup() uses sampling to identify sizes such that each dimension of the projection space is split into roughly 20 parts. This is merely a default and the user is encouraged to set desired cell sizes.

Projections are specific to state spaces. This means that each ompl::base::StateSpace has its corresponding set of ompl::base::ProjectionEvaluator. In the implementation of state spaces, it is recommended that projections are set in the ompl::base::StateSpace::registerProjections() function. Although not required, it is good practice to register all projections to their corresponding state space, as this ensures calling ompl::base::ProjectionEvaluator::setup() when needed. Most state spaces have at least a default projection set by ompl::base::StateSpace::registerProjections(). The user is encouraged to define appropriate projections, if available.

Example code for adding a projection for an R<sup>n</sup> state space:

~~~{.cpp}
using namespace ompl;

class MyProjection : public base::ProjectionEvaluator
{
public:

MyProjection(const base::StateSpacePtr &space) : base::ProjectionEvaluator(space)
{
}

virtual unsigned int getDimension(void) const
{
    return 2;
}

virtual void defaultCellSizes(void)
{
    cellSizes_.resize(2);
    cellSizes_[0] = 0.1;
    cellSizes_[1] = 0.25;
}

virtual void project(const base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const
{
    const double *values = state->as<base::RealVectorStateSpace::StateType>()->values;
    projection(0) = (values[0] + values[1]) / 2.0;
    projection(1) = (values[2] + values[3]) / 2.0;
}
};
...
base::StateSpacePtr space;
base::PlannerPtr planner;

space->registerProjection("myProjection", base::ProjectionEvaluatorPtr(new MyProjection(space)));
...
planner->as<geometric::KPIECE1>()->setProjectionEvaluator("myProjection");
~~~

The last two lines can be combined into one if you register `MyProjection` as the default projection for your space:

~~~{.cpp}
space->registerDefaultProjection(base::ProjectionEvaluatorPtr(new MyProjection(space)));
~~~

For more reading on this topic, please see:

- Șucan, Ioan A., Kavraki Lydia E., On the Performance of Random Linear Projections for Sampling-Based Motion Planning, in _IEEE/RSJ International Conference on Intelligent Robots and Systems_, pp. 2434-2439, Oct 2009. DOI: <https://dx.doi.org/10.1109/IROS.2009.5354403>.
