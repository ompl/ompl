# Installation of Py++

We have included a script to download and install [Py++](https://bitbucket.org/ompl/pyplusplus) and its dependencies ([pygccxml](https://bitbucket.org/ompl/pygccxml) and [GCC-XML](https://github.com/gccxml/gccxml)). The script is located at ompl/src/external/installPyPlusPlus.{sh,bat}. You can run this script (after running cmake) like so:

    make installpyplusplus

The script may ask for your password so that it can install these programs. If the script completes successfully, run cmake again and you should be able to generate the Python bindings by typing “make update_bindings”.

If you are using OS X and MacPorts, it is recommended you install Py++ like so:

    sudo port sync
    sudo port install py27-pyplusplus-devel
