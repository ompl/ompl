# OMPL Planner Arena

The OMPL Planner Arena code allows you to easily create plots from a benchmark database produced from logs by the \c regression_test.sh script. See also http://plannerarena.org.


## Dependencies

- R 3.0 or higher
- The following R packages: shiny, ggplot2, Hmisc, RSQLite. These packages can be installed like so:

      R -e "install.packages(c('shiny', 'ggplot2', 'Hmisc', 'RSQLite'), repos='http://cran.r-project.org')"


## Running

First, produce a series of log files with the \c regression_test.sh script. Next, produce a database called benchmark.db with \c ompl_benchmark_statistics.py. Move benchmark.db to ompl/regression_tests/plannerarena/www/. Finally, start R and type the following commands:

    library(shiny)
    setwd("/path/to/ompl/regression_tests")
    runApp("plannerarena")

If everything was installed correctly, your browser will open http://127.0.0.1:7227, which should look similar to http://plannerarena.org. If you want to install OMPL Planner Arena on your own server, follow the directions at http://rstudio.github.io/shiny-server/latest/.

