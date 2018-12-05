# Installation {#installation}

\htmlonly
<div class="panel panel-default">
  <div class="panel-body">
    <h2>Select your operating system:</h2>
    <!-- Nav tabs -->
    <ul class="nav nav-pills" role="tablist">
      <li role="presentation" class="active"><a href="#ubuntu" aria-controls="ubuntu" role="tab" data-toggle="pill">Ubuntu</a></li>
      <li role="presentation"><a href="#fedora" aria-controls="fedora" role="tab" data-toggle="pill">Fedora</a></li>
      <li role="presentation"><a href="#linux" aria-controls="linux" role="tab" data-toggle="pill">Linux (generic)</a></li>
      <li role="presentation"><a href="#osx" aria-controls="osx" role="tab" data-toggle="pill">macOS</a></li>
      <li role="presentation"><a href="#windows" aria-controls="windows" role="tab" data-toggle="pill">MS Windows</a></li>
    </ul>
  </div>
</div>

<!-- Tab panes -->
<div class="tab-content">
  <div role="tabpanel" class="tab-pane active" id="ubuntu">
    <h2>Ubuntu</h2>
    <ul class="nav nav-tabs" role="tablist">
      <li role="presentation" class="active"><a href="#ubuntusource" aria-controls="ubuntusource" role="tab" data-toggle="tab">From source</a></li>
      <li role="presentation"><a href="#ubuntubinary" aria-controls="ubuntubinary" role="tab" data-toggle="tab">Binary</a></li>
      <li role="presentation"><a href="#ubunturos" aria-controls="ubunturos" role="tab" data-toggle="tab">ROS</a></li>
    </ul>
    <div class="tab-content">
      <div role="tabpanel" class="tab-pane active" id="ubuntusource">
        <a href="install-ompl-ubuntu.sh">Download the OMPL installation script</a>. First, make the script executable:
        <pre class="fragment">chmod u+x install-ompl-ubuntu.sh</pre>
        Next, there are three ways to run this script:
         <ul>
           <li><code>./install-ompl-ubuntu.sh</code> will install OMPL without Python bindings</li>
           <li><code>./install-ompl-ubuntu.sh --python</code> will install OMPL with Python bindings</li>
           <li><code>./install-ompl-ubuntu.sh --app</code> will install OMPL.app with Python bindings</li>
         </ul>
         The script downloads and installs OMPL and all dependencies via <code>apt-get</code> &amp; <code>pip</code> and from source. It will ask for your password to install things. The script has been tested on vanilla installs of Ubuntu 14.04 (Trusty), 15.10 (Wily), 16.04 (Xenial), 17.10 (Artful), and 18.04 (Bionic).
      </div>
      <div role="tabpanel" class="tab-pane" id="ubuntubinary">
        Simply type:
        <pre class="fragment">apt-get install libompl-dev ompl-demos</pre>
        Note that this package does not include Python bindings.
      </div>
      <div role="tabpanel" class="tab-pane" id="ubunturos">
        Debian packages for OMPL are also found in ROS distributions. Note that these packages do not include Python bindings. To install the ROS version of OMPL you need to add the ROS repository to your list of sources (you have probably have done this already if you are using ROS):
        <pre class="fragment">sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -</pre>
        and install OMPL:
        <pre class="fragment">sudo apt-get update
sudo apt-get install ros-`rosversion -d`-ompl</pre>
        Please see <a href="http://moveit.ros.org">MoveIt!</a> for further information.
      </div>
    </div>
  </div>

  <!-- Fedora -->
  <div role="tabpanel" class="tab-pane" id="fedora">
    <h2>Fedora</h2>
    Simply type:
    <pre class="fragment">sudo yum install ompl</pre>
    Note that this package does not include Python bindings.
  </div>

  <!-- Linux (generic) -->
  <div role="tabpanel" class="tab-pane" id="linux">
    <h2>Linux (generic)</h2>
    <p>OMPL requires <a href="http://www.boost.org">Boost</a> (version 1.54 or higher) <a href="http://www.cmake.org">CMake</a> (version 2.8.7 or higher), and <a href="http://eigen.tuxfamily.org">Eigen</a> (version 3.3 or higher). Some additional features are available if <a href="http://www.ode.org">ODE</a> is installed.
    To be able to generate python bindings you need to install the <a href="http://www.python.org">Python</a> library and header files and <a href="installPyPlusPlus.html">Py++</a>.
    Finally, you need a C++11 compiler (g++-5 or newer).</p>
    <p>Once the dependencies are installed, OMPL can then be compiled like so:</p>
    <ul>
    <li>Create a build directory and run cmake: <pre class="fragment">cd ompl
mkdir -p build/Release
cd build/Release
cmake ../..</pre></li>
    <li>Optionally, generate the Python bindings with <code>make -j 4 update_bindings</code>.</li>
    <li>Optionally, run the test programs by typing <code>make test</code>.</li>
    <li>Optionally, generate the documentation (i.e., a local copy of this web site) by typing <code>make doc</code> (requires <a href="http://www.doxygen.org">Doxygen</a> and <a href="http://www.graphviz.org">Graphviz</a> to be installed).</li>
    </ul>
    <p>The build system includes a <a href="buildOptions.html">number of options</a> that you can enable or disable.</p>
  </div>

  <!-- macOS -->
  <div role="tabpanel" class="tab-pane" id="osx">
    <h2>macOS</h2>
    <ul class="nav nav-tabs" role="tablist">
      <li role="presentation" class="active"><a href="#osxmacports" aria-controls="osxmacports" role="tab" data-toggle="tab">MacPorts</a></li>
      <li role="presentation"><a href="#osxhomebrew" aria-controls="osxhomebrew" role="tab" data-toggle="tab">Homebrew</a></li>
    </ul>
    <div class="tab-content">
      <div role="tabpanel" class="tab-pane active" id="osxmacports">
        Install <a href="http://www.macports.org">MacPorts</a> and type:<pre class="fragment">sudo port sync \; install ompl</pre>
        If you want to build OMPL from source, you can install just the OMPL dependencies like so:
        <pre class="fragment">sudo port install `port -q info --depends ompl | sed 's/,//g'`</pre>
      </div>
      <div role="tabpanel" class="tab-pane" id="osxhomebrew">
        Install <a href="http://brew.sh">Homebrew</a> and type:
        <pre class="fragment">brew install ompl</pre>
        Note that the <a href="http://braumeister.org/formula/ompl">Homebrew formula</a> does not include Python bindings. You could install all the dependencies for OMPL and the python bindings and build OMPL from source:
        <pre class="fragment">brew install eigen castxml numpy boost-python3 pypy3 flann</pre>
        Make sure to use Homebrew's python3 in that case by calling <code>cmake</code> like so:
        <pre class="fragment">cmake -DPYTHON_EXEC=/usr/local/bin/python3 ...</pre>
      </div>
    </div>
  </div>

  <!-- Windows -->
  <div role="tabpanel" class="tab-pane" id="windows">
    <h2>MS Windows</h2>
    It is recommended to use <a href="https://vcpkg.readthedocs.io/en/latest/">vcpkg</a>, a Microsoft-supported package manager for open source software. Use <a href="https://bitbucket.org/ompl/ompl/src/tip/.appveyor.yml">our Appveyor configuration file</a> for a list of packages to install and build instructions.
  </div>
</div>
\endhtmlonly
