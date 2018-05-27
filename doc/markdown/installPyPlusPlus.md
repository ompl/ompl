# Installation of Py++ {#installPyPlusPlus}

[Py++](https://bitbucket.org/ompl/pyplusplus) depends on [pygccxml](https://github.com/gccxml/pygccxml), which in turn depends on [CastXML](https://github.com/CastXML/CastXML). You can install these packages from source yourself, but you can also use package managers. For CastXML there are binaries available at <https://midas3.kitware.com/midas/folder/13152>.

## Ubuntu

For Ubuntu 15.10 and newer you can install `castxml` with `sudo apt-get install castxml`.

The pygccxml and Py++ packages can be installed with `pip` (a package manager specifically for Python packages):

    sudo apt-get install pip
    sudo -H pip install -vU pygccxml pyplusplus

## macOS

If you use MacPorts on macOS, then Py++ and its dependencies can be installed like so:

    sudo port install py27-pyplusplus # if you use python2.7
    sudo port install py36-pyplusplus # if you use python3.6
