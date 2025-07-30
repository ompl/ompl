# How to Benchmark Planners {#benchmark}

[TOC]

OMPL contains a ompl::Benchmark class that facilitates solving a motion planning problem repeatedly with different parameters, different planners, different samplers, or even differently configured versions of the same planning algorithm. Below, we will describe how you can use this class.

For interactive visualization of benchmark databases, please see [plannerarena.org](http://plannerarena.org).

## Writing benchmarking code {#benchmark_code}

Benchmarking a set of planners on a specified problem using the Benchmark class in your own code is a simple task in OMPL. The steps involved are as follows:

- Configure the benchmark problem using ompl::geometric::SimpleSetup or ompl::control::SimpleSetup
- Create a ompl::Benchmark object that takes the problem as input
- Optionally, specify some parameters for the benchmark object using ompl::Benchmark::addExperimentParameter, which is useful when aggregating benchmark results over parametrized benchmarks.
- Add one or more planners to the benchmark
- Optionally add events to be called before and/or after the execution of a planner
- Run the benchmark problem a specified number of times, subject to specified time and memory limits

The following code snippet shows you how to do this. We will start with some initial code that you have probably already used:

~~~{.cpp}
#include "ompl/tools/benchmark/Benchmark.h"

// A function that matches the ompl::base::PlannerAllocator type.
// It will be used later to allocate an instance of EST
ompl::base::PlannerPtr myConfiguredPlanner(const ompl::base::SpaceInformationPtr &si)
{
    geometric::EST *est = new ompl::geometric::EST(si);
    est->setRange(100.0);
    return ompl::base::PlannerPtr(est);
}

// Create a state space for the space we are planning in
ompl::geometric::SimpleSetup ss(space);

// Configure the problem to solve: set start state(s)
// and goal representation
// Everything must be set up to the point ss.solve()
// can be called. Setting up a planner is not needed.
~~~

Benchmarking code starts here:

~~~{.cpp}
// First we create a benchmark class:
ompl::tools::Benchmark b(ss, "my experiment");

// Optionally, specify some benchmark parameters (doesn't change how the benchmark is run)
b.addExperimentParameter("num_dofs", "INTEGER", "6")
b.addExperimentParameter("num_obstacles", "INTEGER", "10")

// We add the planners to evaluate.
b.addPlanner(base::PlannerPtr(new geometric::KPIECE1(ss.getSpaceInformation())));
b.addPlanner(base::PlannerPtr(new geometric::RRT(ss.getSpaceInformation())));
b.addPlanner(base::PlannerPtr(new geometric::SBL(ss.getSpaceInformation())));
b.addPlanner(base::PlannerPtr(new geometric::LBKPIECE1(ss.getSpaceInformation())));
// etc

// For planners that we want to configure in specific ways,
// the ompl::base::PlannerAllocator should be used:
b.addPlannerAllocator(std::bind(&myConfiguredPlanner, std::placeholders::_1));
// etc.

// Now we can benchmark: 5 second time limit for each plan computation,
// 100 MB maximum memory usage per plan computation, 50 runs for each planner
// and true means that a text-mode progress bar should be displayed while
// computation is running.
ompl::tools::Benchmark::Request req;
req.maxTime = 5.0;
req.maxMem = 100.0;
req.runCount = 50;
req.displayProgress = true;
b.benchmark(req);

// This will generate a file of the form ompl_host_time.log
b.saveResultsToFile();
~~~

Adding callbacks for before and after the execution of a run is also possible:

~~~{.cpp}
// Assume these functions are defined
void optionalPreRunEvent(const base::PlannerPtr &planner)
{
    // do whatever configuration we want to the planner,
    // including changing of problem definition (input states)
    // via planner->getProblemDefinition()
}

void optionalPostRunEvent(const base::PlannerPtr &planner, tools::Benchmark::RunProperties &run)
{
    // do any cleanup, or set values for upcoming run (or upcoming call to the pre-run event).

    // adding elements to the set of collected run properties is also possible;
    // (the added data will be recorded in the log file)

    run["some extra property name INTEGER"] = "some value";
    // The format of added data is string key, string value pairs,
    // with the convention that the last word in string key is one of
    // REAL, INTEGER, BOOLEAN, STRING. (this will be the type of the field
    // when the log file is processed and saved as a database).
    // The values are always converted to string.
}

// After the Benchmark class is defined, the events can be optionally registered:
b.setPreRunEvent(std::bind(&optionalPreRunEvent, std::placeholders::_1));
b.setPostRunEvent(std::bind(&optionalPostRunEvent, std::placeholders::_1, std::placeholders::_2));
~~~

## Processing the benchmarking log file {#benchmark_log}

Once the C++ code computing the results has been executed, a log file is generated. This contains information about the settings of the planners, the parameters of the problem tested on, etc. To visualize this information, we provide a script that parses the log files:

~~~{.sh}
ompl/scripts/ompl_benchmark_statistics.py logfile.log -d mydatabase.db
~~~

This will generate a SQLite database containing the parsed data. If no database name is specified, the named is assumed to be benchmark.db. Once this database is generated, we can visualize the results. The recommended way is to upload the database to [Planner Arena](http://plannerarena.org) and navigate through the different plots. Planner Arena can also be run locally with the `plannerarena` script (requires R to be installed). Alternatively, you can also produce some basic plots with `ompl_benchmark_statistics.py` like so:

~~~{.sh}
ompl/scripts/ompl_benchmark_statistics.py -d mydatabase.db -p boxplot.pdf
~~~

This will generate a series of plots, one for each of the attributes described below, showing the results for each planner. [Below](#benchmark_sample_results) we have included some sample benchmark results.

If you would like to process the data in different ways, you can generate a dump file that you can load in a MySQL database:

~~~{.sh}
ompl/scripts/ompl_benchmark_statistics.py -d mydatabase.db -m mydump.sql
~~~

For more details on how to use the benchmark script, see:

~~~{.sh}
scripts/ompl_benchmark_statistics.py --help
~~~

Collected benchmark data for each experiment:

- __name:__ name of experiment (optional)
- __totaltime:__ the total duration for conducting the experiment (seconds)
- __timelimit:__ the maximum time allowed for every planner execution (seconds)
- __memorylimit:__ the maximum memory allowed for every planner execution (MB)
- __hostname:__ the name of the host on which the experiment was run
- __date:__ the date and time when the experiment was started

Collected benchmark data for each planner execution:

- __time:__ (real) the amount of time spent planning, in seconds
- __memory:__ (real) the amount of memory spent planning, in MB. Note: this may be inaccurate since memory is often freed in a lazy fashion
- __solved:__ (boolean) flag indicating whether the planner found a solution. Note: the solution can be approximate
- __approximate solution:__ (boolean) flag indicating whether the found solution is approximate (does not reach the goal, but moves towards it)
- __solution difference:__ (real) if the solution is approximate, this is the distance from the end-point of the found approximate solution to the actual goal
- __solution length:__ (real) the length of the found solution
- __solution smoothness:__ (real) the smoothness of the found solution (the closer to 0, the smoother the path is)
- __solution clearance:__ (real) the clearance of the found solution (the higher the value, the larger the distance to invalid regions)
- __solution segments:__ (integer) the number of segments on the solution path
- __correct solution:__ (boolean) flag indicating whether the found solution is correct (a separate check is conducted). This should always be true.
- __correct solution strict:__ (boolean) flag indicating whether the found solution is correct when checked at a finer resolution than the planner used when validating motion segments. If this is sometimes false it means that the used state validation resolution is too high (only applies when using ompl::base::DiscreteMotionValidator).
- __simplification time:__ (real) the time spend simplifying the solution path, in seconds
- __simplified solution length:__ (real) the length of the found solution after simplification
- __simplified solution smoothness:__ (real) the smoothness of the found solution after simplification (the closer to 0, the smoother the path is)
- __simplified solution clearance:__ (real) the clearance of the found solution after simplification (the higher the value, the larger the distance to invalid regions)
- __simplified solution segments:__ (integer) the number of segments on solution path after simplification
- __simplified correct solution:__ (boolean) flag indicating whether the found solution is correct after simplification. This should always be true.
- __simplified correct solution strict:__ (boolean) flag indicating whether the found solution is correct after simplification, when checked at a finer resolution.
- __graph states:__ (integer) the number of states in the constructed graph
- __graph motions:__ (integer) the number of edges (motions) in the constructed graph
- __valid segment fraction:__ (real) the fraction of segments that turned out to be valid (using ompl::base::MotionValidator) out of all the segments that were checked for validity
- more planner-specific properties

Planning algorithms can also register callback functions that the Benchmark class will use to measure progress properties at regular intervals during a run of the planning algorithm. Currently only RRT* uses this functionality. The RRT* constructor registers, among others, a function that returns the cost of the best path found so far:

~~~{.cpp}
addPlannerProgressProperty("best cost REAL", std::bind(&RRTstar::getBestCost, this));
~~~

With the Benchmark class one can thus measure how the cost is decreasing over time. The ompl_benchmark_statistics.py script will automatically generate plots of progress properties as a function of time.

## Sample benchmark results {#benchmark_sample_results}

Below are sample results for running benchmarks for two example problems. See the [gallery](gallery.html#gallery_rigidbody) for visualizations of sample solutions to both problems. The results below were run on a recent model Apple MacBook Pro (2.66 GHz Intel Core i7, 8GB of RAM). It is important to note that none of the planner parameters were tuned; all benchmarks were run with default settings. From these results one cannot draw any firm conclusions about which planner is “better” than some other planner.

These are the PDF files with plots as generated by the ompl_benchmark_statistics.py script:

- [The “cubicles” problem with default settings](images/cubicles.pdf)
- [The “Twistycool” problem with default settings](images/Twistycool.pdf)

The plots show comparisons between ompl::geometric::RRTConnect, ompl::geometric::RRT, ompl::geometric::BKPIECE1, ompl::geometric::LBKPIECE1, ompl::geometric::KPIECE1, ompl::geometric::SBL, ompl::geometric::EST, and ompl::geometric::PRM. Each planner is run 500 times with a 10 second time limit for the cubicles problem for each sampling strategy, while for the Twistycool problem each planner is run 50 times with a 60 second time limit.

For integer and real-valued measurements the script will compute [box plots](https://en.wikipedia.org/wiki/Box_plot). For example, here is the plot for the real-valued attribute __time__ for the cubicles environment:

<div class="row"><div class="col-md-8 col-sm-10 offset-md-2 offset-sm-1"><img src="images/cubicles_time.png" class="img-fluid"></div></div>

For boolean measurements the script will create bar charts with the percentage of __true__ values. For example, here is the plot for the boolean attribute __solved__ for the Twistycool environment, a much harder problem:

<div class="row"><div class="col-md-8 col-sm-10 offset-md-2 offset-sm-1"><img src="images/Twistycool_solved.png" class="img-fluid"></div></div>

Whenever measurements are not always available for a particular attribute, the columns for each planner are labeled with the number of runs for which no data was available. For instance, the boolean attribute __correct solution__ is not set if a solution is not found.

## The benchmark logfile format {#benchmark_logfile_format}

The benchmark log files have a pretty simple structure. Below we have included their syntax in [Extended Backus-Naur Form](https://en.wikipedia.org/wiki/Extended_Backus–Naur_Form). This may be useful for someone interested in extending other planning libraries with similar logging capabilities (which would be helpful in a direct comparison of the performance of planning libraries). Log files in this format can be parsed by ompl_benchmark_statistics.py (see next section).

~~~{.bnf}
logfile               ::= preamble planners_data;
preamble              ::= [version] experiment [exp_property_count exp_properties] hostname date setup [cpuinfo]
                          random_seed time_limit memory_limit [num_runs]
                          total_time [num_enums enums] num_planners;
version               ::= library_name " version " version_number EOL;
experiment            ::= "Experiment " experiment_name EOL;
exp_property_count    ::= int " experiment properties" EOL;
exp_properties        ::= exp_property | exp_property exp_properties;
exp_property          ::= name property_type "=" num EOL;
hostname              ::= "Running on " host EOL;
date                  ::= "Starting at " date_string EOL;
setup                 ::= multi_line_string;
cpuinfo               ::= multi_line_string;
multi_line_string     ::= "<<<|" EOL strings "|>>>" EOL;
strings               ::= string EOL | string EOL strings
random_seed           ::= int " is the random seed" EOL;
time_limit            ::= float " seconds per run" EOL;
memory_limit          ::= float " MB per run" EOL;
num_runs              ::= int " runs per planner" EOL;
total_time            ::= float " seconds spent to collect the data" EOL;
num_enums             ::= num " enum type" EOL;
enums                 ::= enum | enum enums;
enum                  ::= enum_name "|" enum_values EOL;
enum_values           ::= enum_value | enum_value "|" enum_values;
num_planners          ::= int " planners" EOL;
planners_data         ::= planner_data | planner_data planners_data;
planner_data          ::= planner_name EOL int " common properties" EOL
                          planner_properties int " properties for each run" EOL
                          run_properties int " runs" EOL run_measurements
                          [int "progress properties for each run" EOL
                          progress_properties int " runs" EOL
                          progress_measurements] "." EOL;
planner_properties    ::= "" | planner_property planner_properties;
planner_property      ::= property_name " = " property_value EOL;
run_properties        ::= property | property run_properties;
progress_properties   ::= property | property progress_properties;
property              ::= property_name " " property_type EOL;
property_type         ::= "BOOLEAN" | "INTEGER" | "REAL";
run_measurements      ::= run_measurement | run_measurement run_measurements;
run_measurement       ::= data "; " | data "; " run_measurement;
data                  ::= num | "inf" | "nan" | "";
progress_measurements ::= progress_measurement EOL
                         | progress_measurement EOL progress_measurements;
progress_measurement  ::= prog_run_data | prog_run_data ";" progress_measurement;
prog_run_data         ::= data "," | data "," prog_run_data;
~~~

Here, `EOL` denotes a newline character, `int` denotes an integer, `float` denotes a floating point number, `num` denotes an integer or float value and undefined symbols correspond to strings without whitespace characters. The exception is `property_name` which is a string that _can_ have whitespace characters. It is also assumed that if the log file says there is data for _k_ planners that that really is the case (likewise for the number of run measurements and the optional progress measurements).

## The benchmark database schema {#benchmark_database}

<div class="col-sm-4 float-right">
  <img src="images/benchmarkdb_schema.png" width="100%">
  <br/>
  <b>The benchmark database schema</b>
</div>
The ompl_benchmark_statistics.py script can produce a series of plots from a database of benchmark results, but in many cases you may want to produce your own custom plots. For this it useful to understand the schema used for the database. There are five tables in a benchmark database:

- **experiments**. This table contains the following information:
  - *id:* an ID used in the `runs` table to denote that a run was part of a given experiment.
  - *name:* name of the experiment.
  - *totaltime:* total duration of the experiment in seconds.
  - *timelimit:* time limit for each individual run in seconds.
  - *memorylimit:* memory limit for each individual run in MB.
  - *runcount:* the number of times each planner configuration was run.
  - *version:* the version of OMPL that was used.
  - *hostname:* the host name of the machine on which the experiment was performed.
  - *cpuinfo:* CPU information about the machine on which the experiment was performed.
  - *date:* the date on which the experiment was performed.
  - *seed:* the random seed used.
  - *setup:* a string containing a “print-out” of all the settings of the SimpleSetup object used during benchmarking.
- **plannerConfigs**. There are a number of planner types (such as PRM and RRT), but each planner can typically be configured with a number of parameters. A planner configuration refers to a planner type with specific parameter settings. The `plannerConfigs` table contains the following information:
  - *id:* an ID used in the `runs` table to denote that a given planner configuration was used for a run.
  - *name:* the name of the configuration. This can be just the planner name, but when using different parameter settings of the same planner it is essential to use more specific names.
  - *settings:* a string containing a “print-out” of all the settings of the planner.
- **enums**: This table contains description of enumerate types that are measured during benchmarking. By default there is only one such such type defined: ompl::base::PlannerStatus. The table contains the following information:
  - *name:* name of the enumerate type (e.g., “status”).
  - *value:* numerical value used in the runs
  - *description:* text description of each value (e.g. “Exact solution,” “Approximate solution,” “Timeout,” etc.)
- **runs**. The `runs` table contains information for every run in every experiment. Each run is identified by the following fields:
  - *id:* ID of the run
  - *experimentid:* ID of the experiment to which this run belonged.
  - *plannerid:* ID of the planner configuration used for this run.
  .
  In addition, there will be many benchmark statistics. None are *required*, but the OMPL planners all report the properties described above such as time, memory, solution length, simplification time, etc. It is possible that not all planners report the same properties. In that case, planners that do not report such properties will have NULL values in the corresponding fields.
- **progress**. Some planners (such as RRT*) can also periodically report properties *during* a run. This can be useful to analyze the convergence or growth rate. The `progress` table contains the following information:
  - *runid:* the ID of the run for which progress data was tracked.
  - *time:* the time (in sec.) at which the property was measured.
  .
  The actual properties stored depend on the planner, but in the case of RRT* it stores the following additional fields:
  - *iterations:* the number of iterations.
  - *collision_checks:* the number of collision checks (or, more precisely, the number state validator calls).
  - *best_cost:* the cost of the best solution found so far.

Using SQL queries one can easily select a subset of the data or compute <a href="https://en.wikipedia.org/wiki/Join_(SQL)">joins</a> of tables.
Consider the following snippet of R code:

~~~{.splus}
library("ggplot2")
library("RSQLite")
con <- dbConnect(dbDriver("SQLite"), "benchmark.db")

# read data
runs <- dbGetQuery(con, "SELECT REPLACE(plannerConfigs.name,'geometric_','') AS name, runs.* FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid")
progress <- dbGetQuery(con, "SELECT REPLACE(plannerConfigs.name,'geometric_','') AS name, progress.* FROM plannerConfigs INNER JOIN runs INNER JOIN progress ON plannerConfigs.id=runs.plannerid AND runs.id=progress.runid")

# plot some data
pdf('plots.pdf', width=8, height=6)
qplot(name, time, data=runs, geom=c("jitter","boxplot"))
qplot(time, best_cost, data=progress, alpha=I(1/2), colour=name, geom=c("point", "smooth"))

dev.off()
dbDisconnect(con)
~~~

For a small database with 1 experiment and 5 planner configurations we then obtain the following two plots:
\htmlonly
<div class="row">
<div class="col-md-6 col-sm-6">
  <img src="images/R_time.png" width="100%"><br>
<b>Time to find a solution.</b> Note that that RRT* does not terminate because it keeps trying to find a more optimal solution.
</div>
<div class="col-md-6 col-sm-6">
  <img src="images/R_progress.png" width="100%"><br>
<b>Length of shortest path found after a given number of seconds.</b> Only RRT* currently uses progress properties. Although the variability among individual runs is quite high, one can definitely tell that different parameter settings (for the range in this case) lead to statistically significant different behavior.
</div>
</div>
\endhtmlonly

\note Similar code is used for [Planner Arena](http://plannerarena.org), a web site for interactive visualization of benchmark databases. The Planner Arena code is part of the OMPL source. Instructions for running Planner Arena locally can be found [here](plannerarena.html).
