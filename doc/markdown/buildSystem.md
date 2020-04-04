# Integrate Your Own Code with OMPL's Build System {#buildSystem}

When developing your own code that relies on OMPL, you have several options:

1. __Install OMPL and use your own build system:__ First, if you are installing OMPL “by hand” (i.e., not through your package manager), run the following commands in your OMPL build directory:

       cmake -DCMAKE_INSTALL_PREFIX=/some/path
       make install

   See [Build Options](buildOptions.html) for details on how to enable/disable different OMPL-specific features. Below are the specifics for different build systems:

   - __CMake:__ For ease of use with CMake, we have included a CMake module. This normally gets installed in the CMake module path, so if you use CMake in your own project, you can simply use this command in your CMakeLists.txt: `find_package(ompl)`. This will define the following variables:

      - `OMPL_FOUND`         - `TRUE`
      - `OMPL_INCLUDE_DIRS`  - The OMPL include directory
      - `OMPL_LIBRARIES`     - The OMPL library
      - `OMPLAPP_LIBRARIES`  - The OMPL.app libraries (if installed)
      - `OMPL_VERSION`       - The OMPL version in the form <major>.<minor>.<patchlevel>
      - `OMPL_MAJOR_VERSION` - Major version
      - `OMPL_MINOR_VERSION` - Minor version
      - `OMPL_PATCH_VERSION` - Patch version

   - __Makefiles:__ If you use Makefiles, add “`-I/usr/local/include`” (or, e.g., “`-I${HOME}/ompl/src`”) to your compile flags, and “`-L/usr/local/lib -lompl`” (or, e.g., “`-L${HOME}/ompl/build/Release/lib -lompl`”) to your link flags. The compile and link flags can also be obtained using  “`pkg-config --cflags ompl`” and  “`pkg-config --libs ompl`”, respectively.
   - __Autotools:__ Use the pkg-config autoconf macro PKG_CHECK_MODULES([OMPL],[ompl >= 0.10]). This is will define `OMPL_LIBS` and `OMPL_CFLAGS` if OMPL was found.
   - __Eclipse CDT:__ Below is a brief set of instructions for getting OMPL to work with Eclipse CDT. _These instructions have been verified to work with Eclipse Indigo with CDT_.

      1. Click File -> New -> New Project.
      2. Choose C++ Project; click Next.
      3. Give your project a name; choose Executable -> Empty Project for type; click Next.
      4. Click Advanced Settings to open the Properties window for your project.
      5. Go to C/C++ Build -> Settings in the left pane.
      6. Under the Tool Settings tab, choose Cross G++ Compiler -> Includes. To the "Include paths" section, add the location of the OMPL source tree. For example, on a Linux system with default installation path, you should use "/usr/local/include". Click Apply.
      7. Again under the Tool Settings tab, choose Cross G++ Linker -> Libraries. To the "Libraries" section, add "ompl" (and, if needed, any Boost libraries, "ompl_app_base" and "ompl_app") . To the "Library search path" section, add the location of the OMPL library files. For example, on a Linux system with default installation path, you should use "/usr/local/lib". Click Apply.
      8. Click OK to leave the Properties window. Click Finish.

   - __IDE's such as MS Visual Studio and Xcode:__ consult your IDE's manual.
2. __Add your own code in OMPL's directory structure:__ This option is recommend if you extend functionality of OMPL that you wish to contribute back to the OMPL project (see [Third-Party Contributions](thirdparty.html) for details). OMPL uses [CMake](https://www.cmake.org) for its build system. CMake can generate Makefiles and project files for many IDE's. If you create C++ code under ompl/src/ompl/[folder], __you need to re-run CMake.__ CMake will detect your code if [folder] is one of the directories already included by OMPL. If you want your code to be in a different location, you should update ompl/src/ompl/CMakeLists.txt accordingly. See the [Python documentation](python.html#updating_python_bindings) for specific instructions on how to create python bindings for your own code.
