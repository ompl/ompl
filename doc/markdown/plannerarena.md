# OMPL Planner Arena {#plannerarena}

The OMPL Planner Arena code allows you to easily create plots from a benchmark database produced from benchmark log files. See also <http://plannerarena.org> and the [benchmarking tutorial](benchmark.html).

## Installing

Simply install the Python package like so:

```bash
pip3 install plannerarena
```

## Running

First, produce a series of benchmark log files. Next, produce a database called benchmark.db with `ompl_benchmark_statistics.py`. After that, you need to run the script `plannerarena`, which launches Planner Arena (if the script is not in your path, it might be in `${HOME}/.local/bin`). After you upload your database via the web page that should have opened in your browser, it should look similar to <http://plannerarena.org>.
