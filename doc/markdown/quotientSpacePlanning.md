# Quotient Space Planning Framework {#quotientSpacePlanning}

[TOC]

We use the Quotient Space Planning Framework to formalize planning on different abstractions levels.
Abstraction levels are defined as [QuotientSpaces](https://en.wikipedia.org/wiki/Quotient_space_(topology)) which are lower-dimensional abstractions of the configuration space.
A simple example is a rigid body in the plane with the configuration space \f$SE(2)\f$.
We can abstract this configuration space by projecting onto \f$\mathbb{R}^2\f$, the QuotientSpace of positions of the rigid body.

## Admissible QuotientSpace Projections

QuotientSpace projections are many-to-one mappings, which map subsets of the
configuration space to a point of a QuotientSpace. The QuotientSpace, as the
configuration space, contains feasible and infeasible configurations. We like to
disallow projections which map feasible configurations onto infeasible
Quotientspace configurations --- because we could thereby abstract away valid
feasible paths.

A QuotientSpace projection thus projects an infeasible subset onto an infeasible
point, but never a feasible configuration onto an infeasible configuration. Such
projection are called admissible.

For the rigid body example, we get an admissible projection by nesting a smaller
robot in the larger one. Below we show a simple planning problem, where a rigid
body needs to move from an initial start configuration (green) to a designated
goal configuration (red). This problem can be abstracted by nesting a disk in
the rigid body (right Figure).

\htmlonly
<div class="row">
  <img src="images/quotient/rigidbody2d_1.png" class="col-xs-6 col-xs-offset-3">
  <img src="images/quotient/rigidbody2d_2.png" class="col-xs-6 col-xs-offset-3">
</div>
</div>
\endhtmlonly

The same can be done to any robot. To abstract a 3-dof manipulator (Left Figure), one could remove the last link to obtain a 2-dof manipulator (Right Figure) which corresponds to a QuotientSpace projection from a 3-dimensional space to a 2-dimensional space.

\htmlonly
<div class="row">
  <img src="images/quotient/planar_manipulator_3dofs.png" class="col-xs-6 col-xs-offset-3">
  <img src="images/quotient/planar_manipulator_2dofs.png" class="col-xs-6 col-xs-offset-3">
</div>
</div>
\endhtmlonly

In practice, an admissible projection requires that any constraint on the
QuotientSpace is also a constraint on the ConfigurationSpace or any
high-dimensional QuotientSpace. In that sense, QuotientSpace projections are
similar to constraint relaxations.

## Spurious Paths

Our main problem when planning with QuotientSpaces are spurious paths. A
spurious path is a path on a quotient space which cannot be lifted to the
configuration space (A QuotientSpace path can be lifted iff there exists a
feasible path on the ConfigurationSpace which when projected onto the
QuotientSpace will yield the QuotientSpace path). An example is shown below. On
the left we have a solution path for the nested disk going below the obstacle.
This solution path, however, cannot be lifted to the configuration space --- the
L-shaped robot cannot pass below the obstacle. A feasible path on the
ConfigurationSpace is shown on the right, which goes above the obstacle. This
path is a projection of the feasible path on the 3-dimensional
ConfigurationSpace down onto the 2-dimensional QuotientSpace.

\htmlonly
<div class="row">
  <img src="images/quotient/rigidbody2d_3.png" class="col-xs-6 col-xs-offset-3">
  <img src="images/quotient/rigidbody2d_4.png" class="col-xs-6 col-xs-offset-3">
</div>
</div>
\endhtmlonly

## Probabilistic Completeness

To plan with a sequence of QuotientSpaces, we have developed a new algorithm
called the QuotientSpace Rapidly-exploring random tree (QRRT) algorithm.
ompl::geometric::QRRT is probabilistically complete when used with admissible
projections, i.e. it can solve any planning problem which has a solution. In
particular, QRRT can deal with spurious paths. It does so by sampling random
vertices from a lower-dimensional QuotientSpace, and projecting them into the
configuration space.

## Why Use Quotient Space Planning

- Because it is more general than lower-dimensional projections like
  ompl::base::ProjectionEvaluator:
  - (1) We can handle a finite number of sequential projections instead of just one.
  - (2) We can guarantee probabilistic completeness when used with admissible projections
  - (3) We can project onto many manifold spaces instead of just euclidean
    space \f$\mathbb{R}^N\f$.
- Because it can be much faster compared to other planning algorithms.

### Hypercube Benchmark

To see how much faster QuotientSpace planning can be, we provide a hypercube
demo, which you can run yourself using the [Quotient Space HyperCube File.](QuotientSpacePlanningHyperCube_8cpp_source.html)

For demonstration, we change the number of dimensions of the hypercube from
6 to 8 to 10 and finally to 12 (using a narrow passage
of 0.1).

#### 6-dimensional HyperCube

Our results show that ompl::geometric::PRM
performs best with 0.103 seconds and ompl::geometric::QRRT on second place
with 0.111 seconds.

~~~{.txt}
Finished Benchmark (Runtime:10, RunCount:5)
Placement <Rank> <Time (in Seconds)> <Success (in Percentage)>
--------------------------------------------------------------------------------
Place <1> Time: <0.103107> %Success: <100> (PRM) <-- Winner
Place <2> Time: <0.111702> %Success: <100> (QuotientSpaceRRT[3lvl])
Place <3> Time: <5.33306> %Success: <60> (STRIDE)
Place <4> Time: <9.86457> %Success: <20> (RRT)
Place <5> Time: <10.0467> %Success: <0> (KPIECE1)
Place <6> Time: <10.069> %Success: <0> (EST)
--------------------------------------------------------------------------------
~~~

#### 8-dimensional HyperCube

Comparing only the two winning algorithms, we see that ompl::geometric::PRM does not scale well to 8 dimensions with zero solved runs, while ompl::geometric::QRRT performs well with 0.653 seconds for solving every single run.

~~~{.txt}
Finished Benchmark (Runtime:10, RunCount:5)
Placement <Rank> <Time (in Seconds)> <Success (in Percentage)>
--------------------------------------------------------------------------------
Place <1> Time: <0.65329> %Success: <100> (QuotientSpaceRRT[4lvl]) <-- Winner
Place <2> Time: <10.0419> %Success: <0> (PRM)
--------------------------------------------------------------------------------
~~~

#### 10-dimensional HyperCube

ompl::geometric::QRRT even performs well when we increase the dimensions further to 10.

~~~{.txt}
Finished Benchmark (Runtime:10, RunCount:5)
Placement <Rank> <Time (in Seconds)> <Success (in Percentage)>
--------------------------------------------------------------------------------
Place <1> Time: <1.84356> %Success: <100> (QuotientSpaceRRT[5lvl]) <-- Winner
--------------------------------------------------------------------------------
~~~

#### 12-dimensional HyperCube

The algorithm comes to a limit when we increase the dimensionality further to 12.

~~~{.txt}
Finished Benchmark (Runtime:10, RunCount:5)
Placement <Rank> <Time (in Seconds)> <Success (in Percentage)>
--------------------------------------------------------------------------------
Place <1> Time: <9.11489> %Success: <20> (QuotientSpaceRRT[6lvl]) <-- Winner
--------------------------------------------------------------------------------
~~~

## Want to learn more?

### Tutorials

Check out the [tutorial](quotientSpacePlanningTutorial.html).

### Demos

Check out the [demos](group__demos.html).

### Papers

- A Orthey and M Toussaint, _Rapidly-Exploring Quotient-Space Trees: Motion Planning using Sequential Simplifications_, 2019. Also available at <https://arxiv.org/abs/1906.01350>.

- A Orthey, A Escande and E Yoshida, _Quotient Space Motion Planning_, ICRA, 2018. Also available at <https://arxiv.org/abs/1807.09468>.
