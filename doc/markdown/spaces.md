# Available Spaces {#spaces}

## State Spaces

This set of state spaces is included in OMPL:

- R<sup>n</sup> (ompl::base::RealVectorStateSpace),
- SO(2) (rotation in the plane, ompl::base::SO2StateSpace),
- SO(3) (rotation in 3D, ompl::base::SO3StateSpace),
- SE(2) (rotation and translation in the plane, ompl::base::SE2StateSpace),
- SE(3) (rotation and translation in 3D, ompl::base::SE3StateSpace),
- Time  (representation of time, ompl::base::TimeStateSpace),
- Discrete  (representation of discrete states, ompl::base::DiscreteStateSpace),
- Dubins (representation of a Dubins car's state space, ompl::base::DubinsStateSpace),
- ReedsShepp (representation of a Reeds-Shepp car's state space, ompl::base::ReedsSheppStateSpace),
- Extensions of the Dubins model to 3D (useful for planning motions for UAVs and UUVs):
  - The model proposed by Owen et al., ompl::base::OwenStateSpace,
  - The model proposed by Váňa et al., ompl::base::VanaStateSpace,
  - A hybrid of these models proposed by Moll that combines the best elements of both, ompl::base::VanaOwenStatespace,
- A couple of "special" topological spaces: ompl::base::KleinBottleStateSpace, ompl::base::MobiusStateSpace, ompl::base::SphereStateSpace, and ompl::base::TorusStateSpace,
- Constrained state spaces (ompl::base::ConstrainedStateSpace) to represent implicitly defined spaces when [planning with constraints](constrainedPlanning.html). There are several derived classes corresponding to different methodologies for dealing with constraints:
  - ompl::base::ProjectedStateSpace: uses Newton's method to project states in the ambient configuration space onto the constraint manifold.
  - ompl::base::AtlasStateSpace: a state space that incrementally builds up an atlas approximation of the constraint manifold.
  - ompl::base::TangentBundleStateSpace: a state space that is derived from the atlas state space and performs some operations lazily.

In addition, the ompl::base::CompoundStateSpace allows users to create arbitrarily complex state spaces out of simpler state spaces.

## Control Spaces

This set of control spaces is included in OMPL:

- R<sup>n</sup> (ompl::control::RealVectorControlSpace).
- Discrete (ompl::control::DiscreteControlSpace).
