# OMPL Planner Arena

The OMPL Planner Arena code allows you to easily create plots from a benchmark database produced from benchmark log files. See also http://plannerarena.org and the [benchmarking tutorial](benchmark.html).


## Dependencies

- R 3.0.0 or higher
- The following R packages: shiny, ggplot2, Hmisc, RSQLite, and markdown. These packages can be installed like so:

      R -e "install.packages(c('shiny', 'ggplot2', 'Hmisc', 'RSQLite', 'markdown'), repos='http://cran.r-project.org')"


## Running

First, produce a series of benchmark log files. Next, produce a database called benchmark.db with \c ompl_benchmark_statistics.py. After that, you need to run the script `plannerarena`, which launches Planner Arena (if the script is not in your path, it is in `ompl/scripts/plannerarena` directory). After you upload your database via the web page that should have opened in your browser, it should look similar to http://plannerarena.org.
