# Quotient Space Planning Framework {#quotientSpacePlanning}

We use the Quotient Space Planning Framework to formalize planning on different abstractions levels.
Abstraction levels are defined as [QuotientSpaces](https://en.wikipedia.org/wiki/Quotient_space_(topology)) which are lower-dimensional abstractions of the configuration space. 
A simple example is a rigid body in the plane with the configuration space \f$SE(2)\f$. 
We can abstract this configuration space by projecting onto \f$R^2\f$, the QuotientSpace of positions of the rigid body. 



## Admissible QuotientSpace Projections

While there are infinitely many QuotientSpace projections, we are 
interested in admissible projections. Admissible projections map infeasible
subspaces of the configuration space onto infeasible points of a
QuotientSpace. For the rigid body example, an admissible projection is one
where the projection is obtained by nesting a smaller robot in the larger one.
Below we show a simple planning problem, where a rigid body needs to move from
an initial start configuration (green) to a designated goal configuration (red).
This problem can be abstracted by nesting a disk in the rigid body (right
Figure).

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

Our main problem when planning with quotient spaces are spurious
path. A spurious path is a path on a quotient space which cannot be realized
on the configuration space (it cannot be lifted). An example is shown below. On
the left we have a solution path for the nested disk going below the obstacle.
This solution path, however, cannot be lifted to the configuration space (there
does not exists a feasible path on the configuration space which when projected
on the QuotientSpace is equal to the solution path). The only feasible path on
the configuration space is the one shown on the right, which goes above the
obstacle.

\htmlonly
<div class="row">
  <img src="images/quotient/rigidbody2d_3.png" class="col-xs-6 col-xs-offset-3">
  <img src="images/quotient/rigidbody2d_4.png" class="col-xs-6 col-xs-offset-3">
</div>
</div>
\endhtmlonly

## Probabilistical Completeness

To plan with a sequence of QuotientSpaces, we have developed a new algorithm
called the QuotientSpace Rapidly-exploring random tree (QRRT).
ompl::geometric::QRRT is probabilistically complete when used with Admissible
Projections, i.e. it can solve any planning problem which has a solution. In
particular, QRRT can deal with spurious paths. It does so by sampling random
vertices from a lower-dimensional QuotientSpace, and projecting them into the
configuration space. Any finite sequence of QuotientSpaces is allowed.

## Why Use Quotient Space Planning

Because it can be much faster compared to other planning algorithms. You just
have to find the right QuotientSpace sequence. 

Here is an example, where we use the HyperCube to show how much faster
planning can be. [Quotient Space HyperCube](QuotientSpacePlanningHyperCube_8cpp_source.html)

In the first try, we have set the dimensions of the cube to \f$6\f$ and the size of
the narrow passage to \f$0.1\f$. Compared to other planners we got


## Want to learn more?

### Tutorials

Check out the [tutorial](quotientSpacePlanningTutorial.html).

### Demos

Check out the [demos](group__demos.html).

### Papers

- A Orthey and M Toussaint, _Rapidly-Exploring Quotient-Space Trees: Motion Planning using Sequential Simplifications_, 2019. Also available at https://arxiv.org/abs/1906.01350.

- A Orthey, A Escande and E Yoshida, _Quotient Space Motion Planning_, ICRA, 2018. Also available at https://arxiv.org/abs/1807.09468.
