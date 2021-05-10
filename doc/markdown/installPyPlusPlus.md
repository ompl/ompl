# Installation of Py++ {#installPyPlusPlus}

[Py++](https://github.com/ompl/pyplusplus) depends on [pygccxml](https://github.com/gccxml/pygccxml), which in turn depends on [CastXML](https://github.com/CastXML/CastXML). You can install these packages from source yourself, but you can also use package managers. For CastXML there are binaries available at <https://data.kitware.com/#collection/57b5c9e58d777f126827f5a1/folder/57b5de948d777f10f2696370>.

## Ubuntu

Install `castxml` with `sudo apt-get install castxml`.

The pygccxml and Py++ packages can be installed with `pip3` (a package manager specifically for Python packages):

    sudo apt-get install pip3
    sudo -H pip3 install -vU pygccxml pyplusplus

## macOS

On macOS you can also use `pip3`, but if using MacPorts, then Py++ and its dependencies can be installed like so:

    sudo port install py38-pyplusplus # if you use python3.8
