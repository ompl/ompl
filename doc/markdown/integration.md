# Integration of OMPL in Other Systems

OMPL provides a high level of abstraction to make it easier to integrate it into larger robot software systems. By design, the core OMPL library does not include any code for representing geometry and kinematics. However, to solve motion planning problems, we _do_ need to pick a particular representation for robots and the environment. Below we have listed a number of projects that have done this in very different ways.

\htmlonly
<div class="btn-group">
  <a class="btn btn-default" href="#integration_moveit">MoveIt!</a>
  <a class="btn btn-default" href="#integration_openrave">OpenRAVE</a>
  <a class="btn btn-default" href="#integration_morse">MORSE</a>
  <a class="btn btn-default" href="#integration_sims">SIMS</a>
  <a class="btn btn-default" href="#integration_omplapp">OMPL.app</a>
</div>
\endhtmlonly

# MoveIt! {#integration_moveit}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    [MoveIt!](http://moveit.ros.org) provides motion planning functionality in [ROS](http://www.ros.org). Robots are described by [URDF files](http://wiki.ros.org/urdf), which describe the robot's geometry, kinematics, and additional robot information. MoveIt! can load such files, create appropriate state spaces for user-defined joint groups (e.g., “left arm,” “right leg,” “upper body,” “whole body,” etc.), and call OMPL planners to find feasible paths. There is support for inverse kinematics, which makes is possible to, e.g, include end-effector constraints. The paths produced by OMPL are translated by MoveIt! into dynamically feasible trajectories. The MoveIt! setup wizard will automatically discover self-collisions in a pre-processing phase. The environment can either be provided in the form of collection of geometric objects (triangles, spheres, cylinders, etc.), a point cloud (obtained from a RGBD sensor), or a combination of both. The adjacent video is a montage of MoveIt!'s capabilities in 2013.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <div class="embed-responsive embed-responsive-16by9">
      \htmlonly<iframe src="http://www.youtube.com/embed/dblCGZzeUqs"></iframe>\endhtmlonly
    </div>
  </div>
</div>

# OpenRAVE {#integration_openrave}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    [OpenRAVE](http://openrave.org) is a lightweight simulation and planning environment. It does not explicitly support OMPL, but it has a plugin architecture that makes it possible to add new planning algorithms. [Michael Koval](http://mkoval.org) has written a [plugin called or_ompl](https://github.com/personalrobotics/or_ompl) which allows you to use any of the OMPL planners with OpenRAVE. It also exposes OMPL's path simplification routines to OpenRAVE. The adjacent video outlines several key features of the plugin.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <div class="embed-responsive embed-responsive-16by9">
      \htmlonly<iframe src="http://www.youtube.com/embed/6qRRbvNzHG8"></iframe>\endhtmlonly
    </div>
  </div>
</div>


# MORSE {#integration_morse}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    [The Modular OpenRobots Simulation Engine (MORSE)](https://www.openrobots.org/wiki/morse) is a generic simulator for academic robotics. It is implemented as an extension for Blender, a 3D modeling program. Blender includes a game engine which uses the Bullet physics simulator under the hood. MORSE includes many simulated sensors, actuators, and robot models. Caleb Voss, as part of a Google Summer of Code project, developed [a plugin for Blender/MORSE](http://ompl.kavrakilab.org/morse.html) that adds planning functionality. The adjacent video shows an example of what can be produced with this plugin.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <div class="embed-responsive embed-responsive-16by9">
      \htmlonly<iframe src="http://player.vimeo.com/video/71580831?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=0&amp;loop=1"></iframe>\endhtmlonly
    </div>
  </div>
</div>


# SIMS {#integration_sims}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    The Kavraki lab has applied motion planning algorithms to the problem of characterizing protein flexibility and conformational changes. A better understanding of protein structure and flexibility is critical to understanding their function. Proteins can be modeled as long kinematic chains. Instead of collisions, state validity is determined by molecular energy. We have developed a conformational sampling framework called [the Structured Intuitive Move Selector (SIMS)](http://www.kavrakilab.org/bioinformatics/tracing_conformational_changes). It uses [Rosetta](http://www.rosettacommons.org) for modeling protein structures and computing their biophysical feasibility. The SIMS software is still very much under development, but will be released at some point in the near future.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <img src="http://www.kavrakilab.org/sites/default/files/groel_path.png" width="100%">
  </div>
</div>


# OMPL.app {#integration_omplapp}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    OMPL.app consist of two components: a library and a GUI. The library provides bindings to the  [FCL](http://gamma.cs.unc.edu/FCL) and [PQP](http://gamma.cs.unc.edu/SSV) collision checking libraries. By default FCL's discrete collision checking is used, but its continuous collision checking or PQP's discrete collision checking can also be specified. The library relies on the [Assimp](http://assimp.sf.net) library to import a large variety of mesh formats that can be used to represent the robot and its environment. The python-based GUI can be used for planning motions for rigid bodies and a few vehicle types (first-order and second-order cars, a blimp, and a quadrotor). The OMPL.app distribution also contains \c ompl_benchmark, a simple tool that demonstrates some of OMPL's [benchmarking](benchmark.html) capabilities.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <div class="embed-responsive embed-responsive-4by3">
      \htmlonly<iframe src="http://player.vimeo.com/video/58686592?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=0&amp;loop=1"></iframe>\endhtmlonly
    </div>
  </div>
</div>

