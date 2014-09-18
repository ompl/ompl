# Help

## About Planner Arena

**Planner Arena** is a site for benchmarking sampling-based planners. The site is set up to show the performance of implementations of various sampling-based planning algorithms in the [Open Motion Planning Library (OMPL)](http://ompl.kavrakilab.org). We have chosen a few benchmark problems that highlight some interesting aspects of motion planning.

**Planner Arena** is also a site you can use to analyze your own motion planning benchmark data. The easiest way to do so is to use either the `ompl_benchmark` program in OMPL.app or the `Benchmark` class in your own code. See the [relevant documentation](http://ompl.kavrakilab.org/benchmark.html) on the OMPL site. The log files that are produced by the OMPL benchmarking facilities get turned into a SQLite database using a script. The database schema is described on this page as well. This means that you could produce benchmark databases with some other software for entirely different planning algorithms (or different implementations of algorithms in OMPL) and use Planner Arena to visualize the data.

Below, we will describe:
- [The benchmarks included in the default database.](#sampleBenchmarks)
- The different visualizations of benchmark data:
  - [Overall performance.](#overallPerformance)
  - [Progress of planners over time.](#progress)
  - [Comparison of different versions of the same planners.](#regression)
- [Detailed information about the database.](#databaseInfo)
- [Changing the database used.](#changeDatabase)

## <a name="sampleBenchmarks"></a>Sample benchmark descriptions

The benchmark problems below are taken from OMPL. The first movie is an example of the classic narrow passage problem. To pass through the narrow passage the red block has to twist and turn. The second problem contains a number of different homotopy classes of paths that connect the start and goal state. The “easiest” path is a long path where the table travels along the long corridor on the left and top of the environment, but, as the animation shows, there exist shorter paths. The last two examples show a kinematic car navigating in a maze and out of a “bug trap,” repectively.

<div class="row">
  <div class="span4">
    <iframe src="http://player.vimeo.com/video/58709589?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="195" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe>
  </div>
  <div class="span4">
    <iframe src="http://player.vimeo.com/video/58686593?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="195" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe>
  </div>
  <div class="span4">
    <iframe src="http://player.vimeo.com/video/58686594?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="280" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe>
  </div>
  <div class="span4">
    <iframe src="http://player.vimeo.com/video/58686591?title=0&amp;byline=0&amp;portrait=0&amp;color=ffffff&amp;autoplay=1&amp;loop=1" width="280" height="280" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowFullScreen></iframe>
  </div>
</div>

## <a name="overallPerformance"></a>Plots of overall performance

The overall performance plots can show how different planners compare on various measures. The most common performance measure is the time it took a planner to find a feasible solution. For very hard problems where most planners time out without finding a solution, it might be informative to look at _solution difference_: the gap between the best found solution and the goal. Explanations of the various benchmark data collected by OMPL can be found [here](http://ompl.kavrakilab.org/benchmark.html#benchmark_log).

The overall performance page allows you to select a motion planning problem that was benchmarked, a particular benchmark attribute to plot, the OMPL version (in case the database contains data for multiple versions), and the planners to compare. If there is only choice in a particular category, it will be disabled (since there is no other choice available).

Most of the measures are plotted as [box plots](http://en.wikipedia.org/wiki/Box_plot). Missing data is ignored. This is _very_ important to keep in mind: if a planner failed to solve a problem 99 times out of a 100 runs, then the average solution length is determined by one run! To make missing data more apparent, a table below the plot shows how many data points there were for each planner and how many of those were missing values (i.e., `NULL`, `None`, `NA`, etc.).


## <a name="progress"></a>Progress plots

Some planners in OMPL can not only report information _after_ a run is completed, but also periodically report information _during_ a run. In particular, for asymptotically optimal planners it is interesting to look at the convergence rate of the best path cost. Typically, the path cost is simply path length, but OMPL allows you to specify different [optimization objectives](http://ompl.kavrakilab.org/optimalPlanning.html). See also the [benchmarking tutorial](http://ompl.kavrakilab.org/benchmark.html) for information on how to specify objectives in the input files for `ompl_benchmark`.

By default, Planner Arena will plot the smoothed mean as well as a 95% confidence interval for the mean. Analogous to the performance plots, missing data is ignored. During the first couple seconds of a run, a planner may never find a solution path. Below the progress plot, we therefore plot the number of data points available for a particular planner at a particular 1 second time interval.

## <a name="regression"></a>Regression plots

Regression plots show how the performance of the same planners change over different versions of OMPL. This is mostly a tool for the OMPL developers that can help in the identification of changes with unintended side-effects on performance. However, it also allows a user to easily compare the performance of a user's modifications to the planners in OMPL to the latest official release.

In regression plots, the results are shown as a bar plot with error bars.

## <a name="databaseInfo"></a>Information about the benchmark database

On the “Database info” page there are two tabs. Both show information for the motion planning problem selected under “Overall performance.” The first tab show how the benchmark was set up and on what kind of machine the benchmark was run. The second tab shows more detailed information on how the planners were configured. Almost any planner in OMPL has some parameters and this tab will show exactly the parameter values for each planner.


## <a name="changeDatabase"></a>Changing the benchmark database

Finally, it is possible to upload your own database of benchmark data. We have limited the maximum database size to 30MB. If your database is larger, you can [run Planner Arena locally](http://ompl.kavrakilab.org/plannerarena.html). The “Change database” page allows you to switch back to the default database after you have uploaded your own database. You can also download the default database. This might be useful if you want to extend the database with your own benchmarking results and compare our default benchmark data with your own results.
