# Gallery {#gallery}

## MoveIt

A 2017 highlight reel for [MoveIt](https://moveit.ros.org), the [ROS](https://www.ros.org) package that provides a high-level interface to OMPL and other motion planning packages.

\htmlonly
<div class="row justify-content-center">
  <div class="col-md-8 col-sm-10">
    <div class="embed-responsive embed-responsive-16by9">
      <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/0og1SaZYtRc"></iframe>
    </div>
  </div>
</div>
\endhtmlonly

## Planning with constraints

OMPL has support for [motion planning subject to hard constraints](constrainedPlanning.html), including, but not limited to, Cartesian planning. In [a 2019 paper in the International Journal of Robotics Research](http://www.kavrakilab.org/publications/kingston2019exploring-implicit-spaces-for-constrained.pdf) we describe how we have integrated prior motion planning approaches to planning with constraints in one framework that allows you to use any of the OMPL planners for constrained planning. The video below illustrates the main ideas. The examples in the videos are included as demo programs in the `ompl/demos/constraint` directory.

\htmlonly
<div class="row justify-content-center">
  <div class="col-md-8 col-sm-10">
    <div class="embed-responsive embed-responsive-4by3">
      <iframe class="embed-responsive-item" src="https://player.vimeo.com/video/261052837?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1"></iframe>
    </div>
  </div>
</div>
\endhtmlonly

## Morse

Morse is a robot simulator built on top of the 3D modeling program Blender. Caleb Voss has developed a plugin for Blender that allows you use OMPL to plan motions for Morse robots. There is [extensive documentation](morse.html) on how to use this plugin. Below are some example videos produced with this plugin.

\htmlonly
<div class="row">
  <div class="col-md-6 col-sm-6">
    <div class="embed-responsive embed-responsive-16by9">
      <iframe class="embed-responsive-item" src="https://player.vimeo.com/video/71580831?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1"></iframe>
    </div>
  </div>
  <div class="col-md-6 col-sm-6">
    <div class="embed-responsive embed-responsive-16by9">
      <iframe class="embed-responsive-item" src="https://player.vimeo.com/video/72710651?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1"></iframe>
    </div>
  </div>
</div>
\endhtmlonly

## Manipulation Planning

An example of using OMPL on the PR2 from Willow Garage. The robot is asked to move the manipulate objects on the table. This demo is using [ROS](http://www.ros.org).

\htmlonly
<div class="row justify-content-center">
  <div class="col-md-8 col-sm-10">
    <div class="embed-responsive embed-responsive-16by9">
      <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/eUpvbOxrbwY"></iframe>
    </div>
  </div>
</div>
\endhtmlonly

## ROS-Industrial Consortium

ROS-Industrial aims to bring ROS to the industrial world. Path and motion planning, as provided by OMPL, are critical component of that. Below is brief overview of ROS-Industrial.

\htmlonly
<div class="row justify-content-center">
  <div class="col-md-8 col-sm-10">
    <div class="embed-responsive embed-responsive-16by9">
      <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/h54YzGIZFt4"></iframe>
    </div>
  </div>
</div>
\endhtmlonly

## Real-time footstep planning for humanoid robots

Below is a video illustrating the results of using OMPL to plan footsteps for a humanoid. The work is described in detail in:

  Nicolas Perrin and Olivier Stasse and Florent Lamiraux and Young J. Kim and Dinesh Manocha, Real-time footstep planning for humanoid robots among 3D obstacles using a hybrid bounding box, in _Proc. IEEE Conf. on Robotics and Automation_, 2012.

The focus is not so much on OMPL, but rather a new hybrid bounding box that allows the robot to step over obstacles.

\htmlonly
<div class="row justify-content-center">
  <div class="col-md-8 col-sm-10">
    <div class="embed-responsive embed-responsive-4by3">
      <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/HNE4dMycosE"></iframe>
    </div>
  </div>
</div>
\endhtmlonly

## Planning for a Car-Like Vehicle Using ODE

An example of using OMPL to plan for a robotic system simulated with [ODE](http://ode.org). The goal is for the yellow car to reach the location of the green box without hitting the red box. The computation is performed using \ref cKPIECE1 "KPIECE". For each computed motion plan, a representation of the exploration data structure (a tree of motions) is also shown.

\htmlonly
<div class="row justify-content-center">
  <div class="col-md-8 col-sm-10">
    <div class="embed-responsive embed-responsive-4by3">
      <iframe class="embed-responsive-item" src="https://player.vimeo.com/video/108137623?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1"></iframe>
    </div>
  </div>
</div>
\endhtmlonly

## Planning Using OMPL.app {#gallery_omplapp}

Below are some rigid body motion planning problems and corresponding solutions found by OMPL.app. The planar examples use a kinematic car model.

\htmlonly
<div class="row">
  <div class="col-md-4 col-sm-4">
    <div class="embed-responsive embed-responsive-1by1">
      <iframe class="embed-responsive-item" src="https://player.vimeo.com/video/58686594?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1"></iframe>
    </div>
  </div>
  <div class="col-md-4 col-sm-4">
    <div class="embed-responsive embed-responsive-1by1">
      <iframe class="embed-responsive-item" src="https://player.vimeo.com/video/58709484?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="256" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe>
    </div>
  </div>
  <div class="col-md-4 col-sm-4">
    <div class="embed-responsive embed-responsive-1by1">
      <iframe class="embed-responsive-item" src="https://player.vimeo.com/video/58686591?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="280" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe>
    </div>
  </div>
</div>
<div class="row">
  <div class="col-md-4 col-sm-4">
    <div class="embed-responsive embed-responsive-1by1">
      <iframe class="embed-responsive-item" src="https://player.vimeo.com/video/58709589?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="188" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe>
    </div>
  </div>
  <div class="col-md-4 col-sm-4">
    <div class="embed-responsive embed-responsive-1by1">
      <iframe class="embed-responsive-item" src="https://player.vimeo.com/video/58686593?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="195" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe>
    </div>
  </div>
  <div class="col-md-4 col-sm-4">
    <div class="embed-responsive embed-responsive-1by1">
      <iframe class="embed-responsive-item" src="https://player.vimeo.com/video/58686590?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="198" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe>
    </div>
  </div>
</div>
\endhtmlonly

## Planning for Underactuated Systems in the Presence of Drift

In the [game of Koules](http://www.ucw.cz/~hubicka/koules/English/), there are a number of balls (called koules) flying in an orbit. The user controls a spaceship. At any time the user can do one of four things: coast, accelerate, turn left, or turn right. The ship and the koules are allowed to collide. The objective is for the ship to bounce all the koules out of the workspace. The user loses when the ship is bounced out of the workspace first. We have written a [demo program](Koules_8cpp_source.html) that solves this game. The physics have been made significantly harder than in the original game. The movie below shows a solution path found by the [Koules demo](Koules_8cpp_source.html) for 7 koules.

\htmlonly
<div class="row justify-content-center">
  <div class="col-md-8 col-sm-10">
    <div class="embed-responsive embed-responsive-1by1">
      <iframe class="embed-responsive-item" src="https://player.vimeo.com/video/68184980?byline=0&amp;portrait=0&amp;color=ffffff"></iframe>
    </div>
  </div>
</div>
\endhtmlonly

## Class Project from COMP 450 on Path Optimization {#gallery_comp450}

In Fall 2010 OMPL was used for the first time in Lydia Kavraki's Algorithmic Robotics class. Students completed several projects. For their last project they could choose from several options. Linda Hill and Yu Yun worked on path optimization. The different optimization criteria considered they considered were path length and sum of discrete path curvature sum. Minimizing the former in shorter paths, minimizing the second results in smoother paths. They used two optimization techniques specific to paths / curves: B-spline interpolation and path hybridization. Path smoothing using B-spline interpolation is shown below on the left. In path hybridization a set of (approximate) solutions to a motion planning problem is given as input, cross-over points are computed, and a new optimized path composed of path segments is found. An example of path hybridization to minimize path length is shown below on the right. In both cases the paths were in SE(3); the figures show simply the R<sup>3</sup> component.

\htmlonly
<div class="row">
<div class="col-md-6 col-sm-6">
  <img src="images/bspline_cubicle_smoother.png" width="100%"><br>
<b>Path smoothing with B-splines.</b> The input path is shown in red, the optimized output path is shown in blue.
</div>
<div class="col-md-6 col-sm-6">
  <img src="images/hybridization_cubicle_shorter.png" width="100%"><br>
<b>Path shortening using path hybridization.</b> The colored paths are input, the solid black path is the optimized output path.
</div>
</div>
\endhtmlonly
