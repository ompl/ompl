# OMPL Planner Arena {#plannerarena}

The OMPL Planner Arena code allows you to easily create plots from a benchmark database produced from benchmark log files. See also <http://plannerarena.org> and the [benchmarking tutorial](benchmark.html).

## Dependencies

- R 3.1.0 or higher
  \note The R packages on Ubuntu 14.04 (and likely other Linux distributions) are too old. See the [CRAN Ubuntu page](https://cran.r-project.org/bin/linux/ubuntu/README.html) for details on how to get packages for the latest version of R.
- On Ubuntu you need to install libv8-dev.
- The following R packages: shinyjs, V8, tidyverse, Hmisc, RSQLite, and markdown. These packages can be installed like so:

      R -e "install.packages(c('shinyjs', 'V8', 'tidyverse', 'Hmisc', 'RSQLite', 'markdown'), repos='http://cran.r-project.org')"

## Running

First, produce a series of benchmark log files. Next, produce a database called benchmark.db with `ompl_benchmark_statistics.py`. After that, you need to run the script `plannerarena`, which launches Planner Arena (if the script is not in your path, it is in the directory `ompl/scripts/plannerarena`). After you upload your database via the web page that should have opened in your browser, it should look similar to <http://plannerarena.org>.
