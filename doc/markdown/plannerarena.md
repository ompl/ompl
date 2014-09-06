# OMPL Planner Arena

The OMPL Planner Arena code allows you to easily create plots from a benchmark database produced from benchmark log files. See also http://plannerarena.org.


## Dependencies

- R 2.14.1 or higher
- The following R packages: shiny, ggplot2, Hmisc, RSQLite. These packages can be installed like so:

      R -e "install.packages(c('shiny', 'ggplot2', 'Hmisc', 'RSQLite', 'markdown'), repos='http://cran.r-project.org')"


## Running

First, produce a series of benchmark log files (e.g., with the \c regression_test.sh script). Next, produce a database called benchmark.db with \c ompl_benchmark_statistics.py. Move benchmark.db to ompl/regression_tests/plannerarena/www/. Finally, start R and type the following commands:

    library(shiny)
    setwd("/path/to/ompl/scripts/plannerarena")
    runApp()

If everything was installed correctly, your browser will open a page that should look similar to http://plannerarena.org. If you want to install OMPL Planner Arena on your own server, follow the directions at http://rstudio.github.io/shiny-server/latest/.
