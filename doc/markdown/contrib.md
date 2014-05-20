# Submit Your Contribution

We welcome external contributions. If you would like us to list your contribution on our [page](thirdparty.html), please [contact us](contact.html). If you would like us to include your code in OMPL, please read on. We want to make the process of contributing code as easy as possible. To this end we came up with the following guidelines:

1. It is possible that your code simply uses OMPL as a dependency. In that case, bundling that code with OMPL might not make sense, but we can still list this as contribution with links to an external web site if users of OMPL may be interested in this code.

2. If you would like to contribute code to OMPL, be it new algorithms or fixes, optimizations, we prefer to handle contributions in one of three ways:

    - Mercurial <b>[preferred]</b>: contributors clone the OMPL repository on bitbucket.org like so:

          $ hg clone https://bitbucket.org/ompl/ompl

      Contributors should then create their own branch, add their files in the appropriate places, and either send us a “pull” request (if the repository is accessible to us) or email us a [bundle](http://www.selenic.com/mercurial/hg.1.html#bundle). We will then merge your code in that separate branch, test the code and then merge it to the default branch. If you have a simple fix, using the default branch is fine.
    - Git: contributors clone the OMPL repository on github.com like so:

          $ git clone https://github.com/ompl/ompl.git

      Analogous to mercurial, contributors should then create their own branch, add their files in the appropriate places, and send us a “pull” request. We will then merge your code in that separate branch, test the code and then merge it to the default branch. If you have a simple fix, using the default branch is fine.
    - tar balls / zip files / etc.: contributors just email us their files and we put them in the appropriate place.

3. In the unlikely event that the submitted code is of poor quality, we will try to suggest the changes necessary to include the contribution in OMPL, and postpone merging until the necessary changes are made.

   In almost all cases there will be minor issues. [Bitbucket](https://bitbucket.org/ompl/ompl) makes it very easy to have a two-way conversation with the OMPL developers about the code. Common issues that are typically easy to fix include: [const-correctnes](http://en.wikipedia.org/wiki/Const-correctness) of methods and their arguments, missing/incorrect documentation, [memory leaks](http://en.wikipedia.org/wiki/Memory_leak), and gross deviations form the [OMPL style guide](styleGuide.html).

4. All source code files must have a BSD license at the top of each file plus the names of the authors. Also, the code should contain doxygen-style documentation, so that the API documentation can be automatically generated and included in the OMPL web site. Finally, the code should be formatted according to the [OMPL style guide](styleGuide.html).

5. We will add a description to the [contributions page](thirdparty.html) (accessible from the “Community” menu at the top of the page). The authors will be contacted to discuss the description of the contribution and proper acknowledgements. Videos or screen-shots are welcome.
