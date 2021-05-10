# Help

## Table of Contents

- [About Planner Arena](#about)
- [How to cite Planner Arena](#cite)
- [The benchmarks included in the default database.](#sampleBenchmarks)
- The different visualizations of benchmark data:
  - [Plots of overall performance.](#overallPerformance)
  - [Progress of planners over time.](#progress)
  - [Comparison of different versions of the same planners.](#regression)
- [Detailed information about the database.](#databaseInfo)
- [Changing the database used.](#changeDatabase)
- [Installing Planner Arena locally](#installation)

## <a name="about"></a>About Planner Arena

**Planner Arena** is a site for benchmarking sampling-based planners. The site is set up to show the performance of implementations of various sampling-based planning algorithms in the [Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org). We have chosen a few benchmark problems that highlight some interesting aspects of motion planning.

**Planner Arena** is also a site you can use to analyze your own motion planning benchmark data. The easiest way to do so is to use either the `ompl_benchmark` program in OMPL.app or the [`Benchmark`](\ref ompl::tools::Benchmark) class in your own code. See the [relevant documentation](https://ompl.kavrakilab.org/benchmark.html) on the OMPL site. The log files that are produced by the OMPL benchmarking facilities get turned into a SQLite database using a script. The database schema is described on this page as well. This means that you could produce benchmark databases with some other software for entirely different planning algorithms (or different implementations of algorithms in OMPL) and use Planner Arena to visualize the data. Much of the Planner Arena user interface is dynamically constructed based on the contents of the benchmark database. In particular, if you store different types of performance measures in your tables, Planner Arena will still be able to plot the results.

## <a name="cite"></a>How to cite Planner Arena

If you use Planner Arena or the OMPL benchmarking facilities, then we kindly ask you to include the following citation in your publications:

<div class="panel panel-default">
  <div class="panel-body">
Mark Moll, Ioan A. Șucan, Lydia E. Kavraki, <a href="https://moll.ai/publications/moll2015benchmarking-motion-planning-algorithms.pdf">Benchmarking Motion Planning Algorithms: An Extensible Infrastructure for Analysis and Visualization</a>, <em>IEEE Robotics & Automation Magazine,</em> 22(3):96–102, September 2015. doi: <a href="https://dx.doi.org/10.1109/MRA.2015.2448276">10.1109/MRA.2015.2448276</a>.
  </div>
</div>

### BibTeX

    @article{moll2015benchmarking-motion-planning-algorithms,
        Author = {Mark Moll and Ioan A. {\c{S}}ucan and Lydia E. Kavraki},
        Doi = {10.1109/MRA.2015.2448276},
        Journal = {{IEEE} Robotics \& Automation Magazine},
        Month = {September},
        Number = {3},
        Pages = {96--102},
        Title = {Benchmarking Motion Planning Algorithms: An Extensible Infrastructure for Analysis and Visualization},
        Volume = {22},
        Year = {2015}
    }


## <a name="sampleBenchmarks"></a>Sample benchmark descriptions

The default database used by the Planner Arena server contains results for a number  of sample benchmarks described below. Most of them were produced by running the `ompl_benchmark` tool on the following configuration files included with the OMPL.app distribution:
- `cubicles.cfg`: A fairly straightforward 3D rigid body planning problem. There are a few path homotopy classes. The path is a little convoluted, since it has go through the whole environment. A large part of the “basement” is not connected to the rest of the environment. A sample solution is shown in the first movie below.
- `cubicles_opt.cfg`: The same problem, but configured to be solved with a number of optimizing planners. The planners are given more time than in the previous benchmark so that we can compare convergence rates in the progress plots.
- `Abstract.cfg`: This is a more challenging 3D rigid body planning problem with several narrow passages, and several homotopy classes.
- `Home.cfg`: This is also a challenging 3D rigid body planning problem. There is a long path between start and goal that is relatively easy to find, but there also other, shorter paths that are much harder to find. This is therefore a good benchmark for optimizing planners.
- `pipedream_ring.cfg`: A 3D rigid body planning problem that contains one long, curvy narrow passage. Nevertheless, most planners can solve this problem within a few seconds.
- `BugTrap_dcar.cfg`: A challenging kinodynamic motion planning problem: a second-order car has to drive out of a “bug trap” obstacle.
- `Maze_kcar.cfg`: Another kinodynamic motion planning problem: a first-order car has to navigate through a maze. The dynamics are simpler than in the previous benchmark, but the obstacles are more complex.

<div class="row" id="videocollection">
  <div class="video">
    <iframe src="https://player.vimeo.com/video/58686592?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="216" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe><br>
    <b>Cubicles</b>
  </div>
  <div class="video">
    <iframe src="https://player.vimeo.com/video/107884951?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="265" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe><br>
    <b>Abstract</b>
  </div>
  <div class="video">
    <iframe src="https://player.vimeo.com/video/58686593?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="195" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe><br>
    <b>Home</b>
  </div>
  <div class="video">
    <iframe src="https://player.vimeo.com/video/107885658?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="258" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe><br>
    <b>Pipedream – ring</b>
  </div>
  <div class="video">
    <iframe src="https://player.vimeo.com/video/107887115?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="281" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe><br>
    <b>Bugtrap – second-order car</b>
  </div>
  <div class="video">
    <iframe src="https://player.vimeo.com/video/58686594?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="280" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe><br>
    <b>Maze – kinematic car</b>
  </div>
</div>

We have also included the results of a few benchmarks where the robot cannot be modeled by a rigid body. These benchmarks are included with OMPL and OMPL.app as demo programs. We have included the results:
- __KinematicBenchmark20__ (produced by running `demo_KinematicChainBenchmark 20`): A kinematic chain with 20 degrees of freedom has to move out of a curved narrow passage by essentially folding up onto itself (while avoiding self-collisions!) before fully extending outside of the narrow passage. The implementation of this benchmark has improved significantly, so the benchmark times will not exactly match the graphs shown in [this paper](https://dx.doi.org/10.1109/ICRA.2013.6630908), where the benchmark is introduced.
- __EasySwap*__ (produced by running `demo_AnytimePathShortening easy alternate 30 rrtconnect` and `demo_AnytimePathShortening easy none 30 rrtstar`): This benchmark illustrates the benefit of Anytime Path Shortening, a generic wrapper around one or more geometric motion planners that repeatedly applies [shortcutting](\ref ompl::geometric::PathSimplifier) and [hybridization](\ref ompl::geometric::PathHybridization) to a set of solution paths. As dimensionality of the configuration space increases, this approach starts to compare very favorably to asymptotically optimal planners like RRT*. The benchmark consists of two rigid bodies separated by a wall with a not-so-narrow passage having to swap positions. The configuration space is thus 12-dimensional.

## <a name="overallPerformance"></a>Plots of overall performance

The overall performance plots can show how different planners compare on various measures. The most common performance measure is the time it took a planner to find a feasible solution. For very hard problems where most planners time out without finding a solution, it might be informative to look at _solution difference_: the gap between the best found solution and the goal. Explanations of the various benchmark data collected by OMPL can be found [here](https://ompl.kavrakilab.org/benchmark.html#benchmark_log).

The overall performance page allows you to select a motion planning problem that was benchmarked, a particular benchmark attribute to plot, the OMPL version (in case the database contains data for multiple versions), and the planners to compare. If there is only choice in a particular category, it will be disabled (since there is no other choice available).

Most of the measures are plotted as [box plots](https://en.wikipedia.org/wiki/Box_plot). Missing data is ignored. This is _very_ important to keep in mind: if a planner failed to solve a problem 99 times out of a 100 runs, then the average solution length is determined by one run! To make missing data more apparent, a table below the plot shows how many data points there were for each planner and how many of those were missing values (i.e., `NULL`, `None`, `NA`, etc.).

If your benchmark database contains results for parametrized benchmarks, then you can select results for different parameter values. By default, results are aggregated over *all* parameter values. You can also choose to show performance for selected planners across all parameter values by selecting “all (separate)” from the corresponding parameter selection widget.

The plots can be downloaded in two formats:
- **PDF.** This is useful if the plot is more or less “camera-ready” and might just need some touch ups with, e.g., Adobe Illustrator.
- **RData.** This contains both the plot as well as all the data shown in the plot in a file format that can be loaded into R with the `load` command (or just double-click on the file if you have [RStudio](https://www.rstudio.com) installed). The plot can be completely customized, further analysis can be applied to the data, or the data can be plotted in an entirely different way.

## <a name="progress"></a>Progress of planners over time

Some planners in OMPL can not only report information _after_ a run is completed, but also periodically report information _during_ a run. In particular, for asymptotically optimal planners it is interesting to look at the convergence rate of the best path cost. Typically, the path cost is simply path length, but OMPL allows you to specify different [optimization objectives](https://ompl.kavrakilab.org/optimalPlanning.html). See also the [benchmarking tutorial](https://ompl.kavrakilab.org/benchmark.html) for information on how to specify objectives in the input files for `ompl_benchmark`.

By default, Planner Arena will plot the smoothed mean as well as a 95% confidence interval for the mean. Analogous to the performance plots, missing data is ignored. During the first couple seconds of a run, a planner may never find a solution path. Below the progress plot, we therefore plot the number of data points available for a particular planner at a particular 1 second time interval.

## <a name="regression"></a>Comparison of different versions of the same planners.

Regression plots show how the performance of the same planners change over different versions of OMPL. This is mostly a tool for the OMPL developers that can help in the identification of changes with unintended side-effects on performance. However, it also allows a user to easily compare the performance of a user's modifications to the planners in OMPL to the latest official release.

In regression plots, the results are shown as a bar plot with error bars.

## <a name="databaseInfo"></a>Information about the benchmark database

On the “Database info” page there are two tabs. Both show information for the motion planning problem selected under “Overall performance.” The first tab show how the benchmark was set up and on what kind of machine the benchmark was run. The second tab shows more detailed information on how the planners were configured. Almost any planner in OMPL has some parameters and this tab will show exactly the parameter values for each planner.

## <a name="changeDatabase"></a>Changing the benchmark database

Finally, it is possible to upload your own database of benchmark data. We have limited the maximum database size to 30MB. If your database is larger, you can [run Planner Arena locally](https://ompl.kavrakilab.org/plannerarena.html). The “Change database” page allows you to switch back to the default database after you have uploaded your own database. You can also download the default database. This might be useful if you want to extend the database with your own benchmarking results and compare our default benchmark data with your own results.

## <a name="installation"></a>Installing Planner Arena locally

See [this page](https://ompl.kavrakilab.org/plannerarena.html) for detailed instructions on how to run Planner Arena on your own computer.
