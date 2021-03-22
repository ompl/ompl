# Installation {#installation}

\htmlonly
<div class="panel">
  <h2>Select your operating system:</h2>
  <!-- Nav tabs -->
  <ul class="nav nav-pills" role="tablist">
    <li class="nav-item"><a class="nav-link active" id="ubuntu-tab" data-toggle="pill" href="#ubuntu" role="tab" aria-controls="ubuntu" aria-selected="true">Ubuntu</a></li>
    <li class="nav-item"><a class="nav-link" id="fedora-tab" data-toggle="pill" href="#fedora" role="tab" aria-controls="fedora" aria-selected="false">Fedora</a></li>
    <li class="nav-item"><a class="nav-link" id="linux-tab" data-toggle="pill" href="#linux" role="tab" aria-controls="linux" aria-selected="false">Linux (generic)</a></li>
    <li class="nav-item"><a class="nav-link" id="osx-tab" data-toggle="pill" href="#osx" role="tab" aria-controls="macos" aria-selected="false">macOS</a></li>
    <li class="nav-item"><a class="nav-link" id="windows-tab" data-toggle="pill" href="#windows" role="tab" aria-controls="windows" aria-selected="false">MS Windows</a></li>
  </ul>
</div>

<!-- Tab panes -->
<div class="tab-content">
  <div class="tab-pane fade show active" id="ubuntu" role="tabpanel" aria-labelledby="ubuntu-tab">
    <h2>Ubuntu</h2>
    <nav>
    <div class="nav nav-tabs" role="tablist">
      <a class="nav-item nav-link active" data-toggle="tab" href="#ubuntusource" aria-controls="ubuntusource" role="tab">From source</a>
      <a class="nav-item nav-link" data-toggle="tab" href="#ubuntubinary" aria-controls="ubuntubinary" role="tab" data-toggle="tab">Binary</a>
      <a class="nav-item nav-link" data-toggle="tab" href="#ubunturos" aria-controls="ubunturos" role="tab" data-toggle="tab">ROS</a>
    </div>
    </nav>
    <div class="tab-content">
      <div role="tabpanel" class="tab-pane fade show active" id="ubuntusource">
        <a href="install-ompl-ubuntu.sh">Download the OMPL installation script</a>. First, make the script executable:
        <pre class="fragment">chmod u+x install-ompl-ubuntu.sh</pre>
        Next, there are several ways to run this script:
         <ul>
           <li><code>./install-ompl-ubuntu.sh</code> will install the latest release of OMPL without Python bindings</li>
           <li><code>./install-ompl-ubuntu.sh --python</code> will install the latest release of OMPL with Python bindings</li>
           <li><code>./install-ompl-ubuntu.sh --app</code> will install the latest release of OMPL.app with Python bindings</li>
           <li><code>./install-ompl-ubuntu.sh --github</code> will install the main branch of OMPL (this can be combined with the other flags above)</li>         </ul>
         The script downloads and installs OMPL and all dependencies via <code>apt-get</code> &amp; <code>pip</code> and from source. It will ask for your password to install things. The script has been tested on vanilla installs of 16.04 (Xenial) and higher. The Python binding generation requires a lot of RAM; having 6GB or more available is recommended.
      </div>
      <div role="tabpanel" class="tab-pane fade" id="ubuntubinary">
        Simply type:
        <pre class="fragment">apt-get install libompl-dev ompl-demos</pre>
        Note that this package does not include Python bindings.
      </div>
      <div role="tabpanel" class="tab-pane fade" id="ubunturos">
        Debian packages for OMPL are also found in ROS distributions. Note that these packages do not include Python bindings. To install the ROS version of OMPL you need to add the ROS repository to your list of sources (you have probably have done this already if you are using ROS):
        <pre class="fragment">sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -</pre>
        and install OMPL:
        <pre class="fragment">sudo apt-get update
sudo apt-get install ros-`rosversion -d`-ompl</pre>
        Please see <a href="https://moveit.ros.org">MoveIt</a> for further information.
      </div>
    </div>
  </div>

  <!-- Fedora -->
  <div class="tab-pane fade" id="fedora" role="tabpanel" aria-labelledby="fedora-tab">
    <h2>Fedora</h2>
    Simply type:
    <pre class="fragment">sudo yum install ompl</pre>
    Note that this package does not include Python bindings.
  </div>

  <!-- Linux (generic) -->
  <div class="tab-pane fade" id="linux" role="tabpanel" aria-labelledby="linux-tab">
    <h2>Linux (generic)</h2>
    <p>OMPL requires <a href="https://www.boost.org">Boost</a> (version 1.58 or higher), <a href="https://www.cmake.org">CMake</a> (version 3.5 or higher), and <a href="http://eigen.tuxfamily.org">Eigen</a> (version 3.3 or higher). Some additional features are available if <a href="http://www.ode.org">ODE</a> is installed.
    To be able to generate python bindings you need to install the <a href="https://www.python.org">Python</a> library and header files and <a href="installPyPlusPlus.html">Py++</a>.
    Finally, you need a C++14 compiler (g++-5 or newer).</p>
    <p>Once the dependencies are installed, OMPL can then be compiled like so:</p>
    <ul>
    <li>Create a build directory and run cmake: <pre class="fragment">cd ompl
mkdir -p build/Release
cd build/Release
cmake ../..</pre></li>
    <li>Optionally, generate the Python bindings with <code>make -j 4 update_bindings</code>. The Python binding generation requires a lot of RAM; having 6GB or more available is recommended.</li>
    <li>Compile OMPL by typing <code>make -j 4</code>.</li>
    <li>Optionally, run the test programs by typing <code>make test</code>.</li>
    <li>Optionally, generate the documentation (i.e., a local copy of this web site) by typing <code>make doc</code> (requires <a href="http://www.doxygen.org">Doxygen</a> and <a href="http://www.graphviz.org">Graphviz</a> to be installed).</li>
    </ul>
    <p>The build system includes a <a href="buildOptions.html">number of options</a> that you can enable or disable.</p>
  </div>

  <!-- macOS -->
  <div class="tab-pane fade" id="osx" role="tabpanel" aria-labelledby="osx-tab">
    <h2>macOS</h2>
    <div class="nav nav-tabs" role="tablist">
      <a class="nav-item nav-link active" data-toggle="tab" href="#osxmacports" aria-controls="osxmacports" role="tab">MacPorts</a>
      <a class="nav-item nav-link" data-toggle="tab" href="#osxhomebrew" aria-controls="osxhomebrew" role="tab">Homebrew</a>
    </div>
    <div class="tab-content">
      <div role="tabpanel" class="tab-pane fade show active" id="osxmacports">
        Install <a href="https://www.macports.org">MacPorts</a> and type:<pre class="fragment">sudo port sync \; install ompl</pre>
        If you want to build OMPL from source, you can install just the OMPL dependencies like so:
        <pre class="fragment">sudo port install `port -q info --depends ompl | sed 's/,//g'`</pre>
      </div>
      <div role="tabpanel" class="tab-pane fade" id="osxhomebrew">
        Install <a href="https://brew.sh">Homebrew</a> and type:
        <pre class="fragment">brew install ompl</pre>
        Note that the <a href="https://formulae.brew.sh/formula/ompl">Homebrew formula</a> does not include Python bindings. You could install all the dependencies for OMPL and the Python bindings and build OMPL from source:
        <pre class="fragment">brew install eigen castxml numpy boost-python3 pypy3 flann</pre>
        Make sure to use Homebrew's python3 in that case by calling <code>cmake</code> like so:
        <pre class="fragment">cmake -DPYTHON_EXEC=/usr/local/bin/python3 ...</pre>
      </div>
    </div>
  </div>

  <!-- Windows -->
  <div class="tab-pane fade" id="windows" role="tabpanel" aria-labelledby="windows-tab">
    <h2>MS Windows</h2>
    It is recommended to use <a href="https://vcpkg.readthedocs.io/en/latest/">vcpkg</a>, a Microsoft-supported package manager for open source software. Once you have vcpkg installed, you can install OMPL like so:
    <pre class="fragment">vcpkg install ompl</pre>
    Note that the vcpkg installation does not include Python bindings.
  </div>
</div>
\endhtmlonly
