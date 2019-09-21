# Submit Your Contribution {#contrib}

We welcome external contributions. If you would like us to list your contribution on our [page](thirdparty.html), please [contact us](contact.html). If you would like us to include your code in OMPL, please read on. We want to make the process of contributing code as easy as possible. To this end we came up with the following guidelines:

1. It is possible that your code simply uses OMPL as a dependency. In that case, bundling that code with OMPL might not make sense, but we can still list this as contribution with links to an external web site if users of OMPL may be interested in this code.

2. If you would like to contribute code to OMPL, be it new algorithms or fixes / optimizations, we welcome contributions via pull requests on Github.

3. In the unlikely event that the submitted code is of poor quality, we will try to suggest the changes necessary to include the contribution in OMPL, and postpone merging until the necessary changes are made.

   In almost all cases there will be minor issues. [GitHub](https://github.com/ompl/ompl) makes it very easy to have a two-way conversation with the OMPL developers about the code. Common issues that are typically easy to fix include: [const-correctness](http://en.wikipedia.org/wiki/Const-correctness) of methods and their arguments, missing/incorrect documentation, [memory leaks](http://en.wikipedia.org/wiki/Memory_leak), and gross deviations form the [OMPL style guide](styleGuide.html).

4. All source code files must have a BSD license at the top of each file plus the names of the authors. Also, the code should contain doxygen-style documentation, so that the API documentation can be automatically generated and included in the OMPL web site. Finally, the code should be formatted according to the [OMPL style guide](styleGuide.html).

5. We will add a description to the [contributions page](thirdparty.html) (accessible from the “Community” menu at the top of the page). The authors will be contacted to discuss the description of the contribution and proper acknowledgements. Videos or screen-shots are welcome.
