# Integration of OMPL in Other Systems {#integration}

[TOC]

OMPL provides a high level of abstraction to make it easier to integrate it into larger robot software systems. By design, the core OMPL library does not include any code for representing geometry and kinematics. However, to solve motion planning problems, we _do_ need to pick a particular representation for robots and the environment. Below we have listed a number of projects that have done this in very different ways.

## MoveIt {#integration_moveit}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    [MoveIt!](https://moveit.ros.org) provides motion planning functionality in [ROS](https://www.ros.org). Robots are described by [URDF files](https://wiki.ros.org/urdf), which describe the robot's geometry, kinematics, and additional robot information. MoveIt! can load such files, create appropriate state spaces for user-defined joint groups (e.g., “left arm,” “right leg,” “upper body,” “whole body,” etc.), and call OMPL planners to find feasible paths. There is support for inverse kinematics, which makes is possible to, e.g, include end-effector constraints. The paths produced by OMPL are translated by MoveIt! into dynamically feasible trajectories. The MoveIt! setup wizard will automatically discover self-collisions in a pre-processing phase. The environment can either be provided in the form of collection of geometric objects (triangles, spheres, cylinders, etc.), a point cloud (obtained from a RGBD sensor), or a combination of both. The adjacent video is a montage of MoveIt!'s capabilities in 2017.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <div class="embed-responsive embed-responsive-16by9">
      \htmlonly<iframe src="https://www.youtube.com/embed/0og1SaZYtRc"></iframe>\endhtmlonly
    </div>
  </div>
</div>

## OpenRAVE {#integration_openrave}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    [OpenRAVE](http://openrave.org) is a lightweight simulation and planning environment. It does not explicitly support OMPL, but it has a plugin architecture that makes it possible to add new planning algorithms. [Michael Koval](http://mkoval.org) has written a [plugin called or_ompl](https://github.com/personalrobotics/or_ompl) which allows you to use any of the OMPL planners with OpenRAVE. It also exposes OMPL's path simplification routines to OpenRAVE. The adjacent video outlines several key features of the plugin.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <div class="embed-responsive embed-responsive-16by9">
      \htmlonly<iframe src="https://www.youtube.com/embed/6qRRbvNzHG8"></iframe>\endhtmlonly
    </div>
  </div>
</div>

## CoppeliaSim {#integration_copelliasim}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    [CoppeliaSim](http://coppeliarobotics.com) is a modular, generic and general purpose robot simulation framework that offers various tools related to robotics (4 physics engines, collision detection, minimum distance calculation, proximity sensor simulation, vision sensor simulation, full FK/IK kinematic solver, etc.), with various kinds of interfaces (ROS, remote API, plug-ins, add-ons) and language support: C/C++, Python, Java, Matlab, Octave, Lua. It is built on a distributed control architecture, allowing virtually any number of scripts running in parallel and controlling various aspects of a simulation. The OMPL interface for V-REP was implemented via a plug-in wrapping the OMPL functionality, and offering that functionality via scripting functions. This allows to quickly test various scenarios, without the need to recompile/load test code over and over again. In combination with V-REP's kinematic functionality, complex movement sequences can easily be computed: e.g. V-REP can also quickly compute several valid robot configurations for a desired end-effector pose.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <div class="embed-responsive embed-responsive-16by9">
      \htmlonly<iframe src="https://www.youtube.com/embed/JAs2yciPjvM"></iframe>\endhtmlonly
    </div>
  </div>
</div>

## MORSE {#integration_morse}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    [The Modular OpenRobots Simulation Engine (MORSE)](http://morse-simulator.github.io) is a generic simulator for academic robotics. It is implemented as an extension for Blender, a 3D modeling program. Blender includes a game engine which uses the Bullet physics simulator under the hood. MORSE includes many simulated sensors, actuators, and robot models. Caleb Voss, as part of a Google Summer of Code project, developed [a plugin for Blender/MORSE](https://ompl.kavrakilab.org/morse.html) that adds planning functionality. The adjacent video shows an example of what can be produced with this plugin.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <div class="embed-responsive embed-responsive-16by9">
      \htmlonly<iframe src="https://player.vimeo.com/video/71580831?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=0&amp;loop=1"></iframe>\endhtmlonly
    </div>
  </div>
</div>

## The Kautham Project {#integration_kautham}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    [The Kautham Project](https://sir.upc.edu/projects/kautham/) is a software tool used at the Institute of Industrial and Control Engineering (IOC-UPC) for teaching and research in robot motion planning. The tool can plan and simulate systems ranging from simple two degrees of freedom free-flying robots to multi-robot scenarios with mobile manipulators equipped with anthropomorphic hands.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <img src="https://sir.upc.edu/projects/kautham/images/justin.JPG" width="100%">
  </div>
</div>

## VEROSIM {#integration_verosim}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    VEROSIM is a 3D simulation system which aims at implementing the core idea of “eRobotics,” i.e., to design, program, control and optimize complex automated systems in detailed 3D simulations of their prospective working environments (space, open landscapes, forests, cities, buildings, factories etc), before commissioning the real system. At the University of Southern Denmark (SDU), Christian Schlette works on the integration of OMPL into VEROSIM for conventional robot motion planning as well as examining the transferability of sampling-based planners to new (robotic) applications. The adjacent video shows an example by RWTH Aachen University, where the integration of VEROSIM and OMPL support non-roboticists with automated motion planning in cluttered workcells.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <div class="embed-responsive embed-responsive-16by9">
      \htmlonly<iframe src="https://www.youtube.com/embed/CENLGo2WDAw"></iframe>\endhtmlonly
    </div>
  </div>
</div>

## AIKIDO {#integration_aikido}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    [AIKIDO](https://github.com/personalrobotics/aikido) is a C++ library, complete with Python bindings, for solving robotic motion planning and decision making problems. This library is tightly integrated with [DART](http://dartsim.github.io/) for kinematic/dynamics calculations and [OMPL](https://ompl.kavrakilab.org/) for motion planning. AIKIDO optionally integrates with [ROS](https://ros.org/), through the suite of `aikido_ros` packages, for execution on real robots.
  </div>
</div>

## EXOTica {#integration_exotica}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    [The EXOTica library](https://github.com/ipab-slmc/exotica) is a generic Optimization Toolset for Robotics platforms, written in C++. Its motivation is to provide a more streamlined process for developing algorithms for such tasks as Inverse-Kinematics and Trajectory Optimisation.  Its design advocates modularity, extensibility, and integration with ROS. The library itself consists of two major specifications, both of which are abstract classes. The first is the Problem Solver which defines the way optimization should proceed: current implementation include iLQG, AICO, Jacobian pseudo-inverse IK, and a range of sampling based solvers from the OMPL library. The other is the Task Definition which describes the task itself by providing two necessary functions to compute the forward map from Configuration space (say joint angles in IK) to Task space (say end-effector positions in IK). The tasks themselves can describe a complete trajectory. Using the library then involves passing in an initial state and requesting a solution to the problem, which may consist of a single configuration or complete trajectory.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <div class="embed-responsive embed-responsive-16by9">
      \htmlonly<iframe src="https://www.youtube.com/embed/AZQY_QOX0Pw"></iframe>\endhtmlonly
    </div>
  </div>
</div>

## Robotics Library {#integration_rl}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    [The Robotics Library](https://www.roboticslibrary.org) is a self-contained C++ library for robot kinematics, motion planning and control. It covers mathematics, kinematics and dynamics, hardware abstraction, motion planning, collision detection, and visualization. OMPL is not included by default, but we have created [a pull request on GitHub](https://github.com/roboticslibrary/rl/pull/2) to add OMPL support. Let us know if this works for you!
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <div class="embed-responsive embed-responsive-16by9">
      \htmlonly<iframe src="https://www.youtube.com/embed/9JG3uY5M04A"></iframe>\endhtmlonly
    </div>
  </div>
</div>

## SIMS {#integration_sims}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    The Kavraki lab has applied motion planning algorithms to the problem of characterizing protein flexibility and conformational changes. A better understanding of protein structure and flexibility is critical to understanding their function. Proteins can be modeled as long kinematic chains. Instead of collisions, state validity is determined by molecular energy. We have developed a conformational sampling framework called [the Structured Intuitive Move Selector (SIMS)](https://doi.org/10.1371/journal.pone.0068826). It uses [Rosetta](https://www.rosettacommons.org) for modeling protein structures and computing their biophysical feasibility. The SIMS software is still very much under development, but will be released at some point in the near future.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <img src="https://journals.plos.org/plosone/article/figure/image?size=medium&id=10.1371/journal.pone.0068826.g008" width="100%">
  </div>
</div>

## OMPL.app {#integration_omplapp}

<div class="row">
  <div class="col-lg-7 col-md-6 col-sm-5">
    OMPL.app consist of two components: a library and a GUI. The library provides bindings to the  [FCL](https://github.com/flexible-collision-library/fcl) and [PQP](http://gamma.cs.unc.edu/SSV) collision checking libraries. By default FCL's discrete collision checking is used, but its continuous collision checking or PQP's discrete collision checking can also be specified. The library relies on the [Assimp](http://www.assimp.org) library to import a large variety of mesh formats that can be used to represent the robot and its environment. The python-based GUI can be used for planning motions for rigid bodies and a few vehicle types (first-order and second-order cars, a blimp, and a quadrotor). The OMPL.app distribution also contains `ompl_benchmark`, a simple tool that demonstrates some of OMPL's [benchmarking](benchmark.html) capabilities.
  </div>
  <div class="col-lg-5 col-md-6 col-sm-7">
    <div class="embed-responsive embed-responsive-4by3">
      \htmlonly<iframe src="https://player.vimeo.com/video/58686592?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=0&amp;loop=1"></iframe>\endhtmlonly
    </div>
  </div>
</div>
