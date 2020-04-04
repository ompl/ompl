# Path Visualization {#pathVisualization}

OMPL does not contain any built-in visualization tools, but it is usually not too difficult to visualize paths with external software. The basic idea is to print the path as a matrix, save it to a text file, and open this file in an external program. In OMPL there are two different kinds of paths: ompl::geometric::PathGeometric and ompl::control::PathControl. Both classes have a method called `printAsMatrix()`. For ompl::geometric::PathGeometric the matrix consists of all the states along the path, one per row. For ompl::control::PathControl each row also contains the controls and control duration needed to reach the state in this row starting from the state in the previous row (the controls and duration in the first row are all zeros).
In the case of ompl::control::PathControl, you may want to convert the path first to a geometric path. This conversion automatically interpolates the path at the propagation step size.
The basic usage is as follows.

\htmlonly
<nav>
<div class="nav nav-tabs" role="tablist">
  <a class="nav-item nav-link active" data-toggle="tab" href="#python" aria-controls="python" role="tab"><b>Python</b></a>
  <a class="nav-item nav-link" data-toggle="tab" href="#cpp" aria-controls="cpp" role="tab" data-toggle="tab"><b>C++</b></a>
</div>
</nav>
<div class="tab-content">
  <div role="tabpanel" class="tab-pane fade show active" id="python">
\endhtmlonly
\code{.py}
solved = ss.solve(20.0)
if solved:
    # if ss is a ompl::geometric::SimpleSetup object
    print(ss.getSolutionPath().printAsMatrix())

    # if ss is a ompl::control::SimpleSetup object
    print(ss.getSolutionPath().asGeometric().printAsMatrix())
\endcode
\htmlonly
</div>
<div role="tabpanel" class="tab-pane fade" id="cpp">
\endhtmlonly
~~~{.cpp}
bool solved = ss.solve(20.0);
if (solved)
{
    // if ss is a ompl::geometric::SimpleSetup object
    ss.getSolutionPath().printAsMatrix(std::cout);

    // if ss is a ompl::control::SimpleSetup object
    ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
}
~~~
\htmlonly
</div>
</div>
\endhtmlonly

Run your program and save the output of `printAsMatrix()` to a file called, say, path.txt. Let us assume the path consists of SE(3) states. This means that the first three columns represent the 3D position component of the state and the next four columns the orientation represented by a unit quaternion. Below we will describe how to visualize this path with a variety of programs.

* [OMPL.app GUI](https://ompl.kavrakilab.org/gui.html): The OMPL.app GUI can be used to “play back” any path consisting of SE(2) or SE(3) paths. First, load a mesh representing the robot and (optionally) a mesh representing the environment. Next, load the path file `path.txt`. You can either let the GUI loop through the states along the path or show them all simultaneously.
* [Matplotlib](https://matplotlib.org):
~~~{.py}
from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
data = numpy.loadtxt('path.txt')
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(data[:,1],data[:,2],data[:,3],'.-')
plt.show()
~~~

* [Matlab](https://www.mathworks.com/products/matlab/) / [Octave](https://www.gnu.org/software/octave/):
~~~{.matlab}
data=load('path.txt');
plot3(data(:,1), data(:,2), data(:,3), 'o-');
axis equal, grid on, rotate3d on;
~~~

* [Excel](https://office.microsoft.com/en-us/excel/): When you open path.txt with Excel, it will guide you through the process of importing it into a spreadsheet. You should select space-delimited data. Unfortunately, it looks like it is difficult to make plots of a 3D curve in Excel. However, you _can_ plot 2D curves. To create such a plot, select the first two columns of data and select "Insert>Chart...". Next, click on "Scatter>Straight Marked Scatter". (This is for Microsoft Excel for Mac 2011; the menu items might be called something slightly differently in the Microsoft Windows version.)

* [R](https://www.r-project.org): First, you need to install the `scatterplot3d` package:
~~~{.splus}
install.packages(scatterplot3d)
~~~

  Then you can plot the path like so:
~~~{.splus}
library(scatterplot3d)
data <- read.table('path.txt')
scatterplot3d(data[,('V1')], data[,('V2')], data[,('V3')], color = "blue", type = "l", angle = 120, xlab='x', ylab='y', zlab='z')
~~~

* [gnuplot](http://www.gnuplot.info):
~~~
set grid xtics ytics ztics
splot "path.txt" using 1:2:3 with linespoints
~~~

Plotting little coordinate frames at each position to represent the orientation encoded by the quaternion in columns 4 through 7 is straightforward and left as an exercise to the reader.
