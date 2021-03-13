# OMPL Planner Arena {#plannerarena}

The OMPL Planner Arena code allows you to easily create plots from a benchmark database produced from benchmark log files. See also <http://plannerarena.org> and the [benchmarking tutorial](benchmark.html).

## Dependencies

- R 3.1.0 or higher
- On Ubuntu you need to run `sudo apt install libv8-dev`.
- The following R packages: shinyjs, V8, tidyverse, Hmisc, pool, RSQLite, and markdown. These packages can be installed like so:

      R -e "install.packages(c('shinyjs', 'V8', 'tidyverse', 'Hmisc', 'pool', 'RSQLite', 'markdown'), repos='https://cran.r-project.org')"

## Running

First, produce a series of benchmark log files. Next, produce a database called benchmark.db with `ompl_benchmark_statistics.py`. After that, you need to run the script `plannerarena`, which launches Planner Arena (if the script is not in your path, it is in the directory `ompl/scripts/plannerarena`). After you upload your database via the web page that should have opened in your browser, it should look similar to <http://plannerarena.org>.

## Docker

PlannerArena is also available as a Docker container. Once you have Docker installed, you can download and run PlannerArena like so:

    docker pull mmoll/plannerarena
    docker run --rm -p 8888:8888 plannerarena

Go to <http://127.0.0.1:8888> and upload your benchmark data. If your benchmark database files exceed 50MB, set the environment variable `OMPL_PLANNERARENA_MAX_DB_SIZE` to the desired limit in bytes in the [Dockerfile for Planner Arena](https://github.com/ompl/ompl/blob/main/scripts/docker/plannerarena.Dockerfile) and rebuild the docker container.
