# Available State Samplers {#samplers}

There are two different kinds of samplers that sound similar, but have different roles: state space samplers (ompl::base::StateSampler) and _valid_ state samplers (ompl::base::ValidStateSampler). For each type of state space there needs to exist a corresponding derived ompl::base::StateSampler class that allows one to generate uniform samples from that state space, generate states near another state from that state space and generate states using a Gaussian distribution. The valid state samplers use the state space samplers as a low level primitive. Typically, they generate a number of state samples using the appropriate state space sampler until a valid state is found or a maximum number of iterations is exceeded. The validity of a state is determined through the ompl::base::SpaceInformation::isValid method. There are some pre-defined derived ompl::base::ValidStateSampler classes:

- ompl::base::UniformValidStateSampler: This is the default sampler. It simply calls the state space sampler to generate uniform samples until a valid state is found or a maximum number of iterations is exceeded.
- ompl::base::ObstacleBasedValidStateSampler: This sampler tries to first find one invalid sample and one valid sample. Next, it interpolates states incrementally from the valid to the invalid state. It returns the last state that is valid before reaching an invalid state. The idea is that samples near obstacles improve the chance of finding samples in narrow passages. Finding such samples is often the crucial problem in solving motion planning queries.
- ompl::base::GaussianValidStateSampler: This sampler tries to accomplish something similar to the previous one, but in a different way. It repeatedly generates pairs of of states. The first one is uniformly random, while the second one is sampled according to a Gaussian distribution centered around the first sample. If one sample is valid and the other one invalid, the valid one is returned. If both are valid or invalid, it generates a new pair. This process repeats until a maximum number of iterations is exceeded.
- ompl::base::MaximizeClearanceValidStateSampler: This sampler behaves a lot like ompl::base::UniformValidStateSampler but once it finds a valid state, it attempts to find additional valid states with higher clearance. The reported sample is the one with highest clearance.

Below we will describe how you can specify a planner to use one of these samplers and how to write your own valid state sampler. The code examples are taken from the [StateSampling.cpp](StateSampling_8cpp_source.html) demo program (note that there is also a [Python version](StateSampling_8py_source.html) of this demo).

## Using an Existing Sampler

We cannot set the type of sampler directly in the SimpleSetup or SpaceInformation classes, because each thread needs it own copy of a sampler. Instead, we need to define a ompl::base::ValidStateSamplerAllocator, a function that, given a pointer to an ompl::base::SpaceInformation, returns ompl::base::ValidStateSamplerPtr. This function can also configure the valid state sampler according to the specific space information before returning it. The following simple example shows how to use the ObstacleBasedValidStateSampler:

\dontinclude StateSampling.cpp
\skip ompl::base
\until ompl::geometric
\skip allocOBValidStateSampler
\until }
\skip plan
\until StateSpacePtr
\skipline SimpleSetup
\skip sampler
\until allocOBValidStateSampler
Other setup steps, such as specifying start and goal states, have been omitted for the sake of clarity.

## Creating a New Valid State Sampler

A wide variety of heuristics have been proposed to improve the sampling of states. The quality of a sample can be characterized, e.g., by its distance to the nearest obstacle or by the “visibility” from a state. There are also two common cases where problem-specific information can be exploited:

- If you use a collision checker that returns the distance to the closest obstacle or—even better—returns also the gradient of the distance function, then it would make sense to create a new valid state sampler that can exploit this information.
- If you can directly incorporate state validity constraints into the sampling (rather than use the rejection sampling scheme of the standard valid state samplers), then this will help boost performance. We will give an example of such sampler below.

In the code below we are planning for a 3D point moving around inside a cube centered at the origin. There is one rectangular obstacle. Since the valid region is easy to describe, we can sample directly from the free space.

\dontinclude StateSampling.cpp
\skip ompl::base
\until };
We define a valid state allocator similarly as describe above:
\skip allocMyValidStateSampler
\until }
Finally, using the new sampler is done analogously as before:
\skipline allocMyValidStateSampler
