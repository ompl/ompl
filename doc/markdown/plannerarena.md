# OMPL Planner Arena

The OMPL Planner Arena code allows you to easily create plots from a benchmark database produced from benchmark log files. See also http://plannerarena.org and the [benchmarking tutorial](benchmark.html).


## Dependencies

- R 3.0.0 or higher
- The following R packages: shiny, ggplot2, Hmisc, RSQLite, and markdown. These packages can be installed like so:

      R -e "install.packages(c('shiny', 'ggplot2', 'Hmisc', 'RSQLite', 'markdown'), repos='http://cran.r-project.org')"


## Running

First, produce a series of benchmark log files. Next, produce a database called benchmark.db with \c ompl_benchmark_statistics.py. Start R and type the following commands:

    library(shiny)
    setwd("/path/to/ompl/scripts/plannerarena")
    runApp()

If everything was installed correctly, your browser will open a page. After you upload your database, it should look similar to http://plannerarena.org. If you want to install OMPL Planner Arena on your own server, follow the directions at http://rstudio.github.io/shiny-server/latest/.
