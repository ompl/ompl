# Installation of Py++

[Py++](https://sourceforge.net/projects/pygccxml) requires pygccxml and GCC-XML. You need the versions of GCC-XML, pygccxml, and Py++ that are taken directly from their respective repositories. Directions for installing GCC-XML can be found [here](http://www.gccxml.org/HTML/Download.html), and for installing pygccxml/Py++ they can be found [here](http://sourceforge.net/scm/?type=svn&group_id=118209). The combined set of instructions is included in the script ompl/src/external/installPyPlusPlus.{sh,bat}. This script will also apply a patch to pygccxml that is necessary to prevent it from treating gccxml warnings as errors. You can run this script (after running cmake) like so:

    make installpyplusplus

The script may ask for your password so that it can install these programs. If the script completes successfully, run cmake again and you should be able to generate the Python bindings by typing “make update_bindings”.

If you are using OS X and MacPorts, it is recommended you install Py++ like so:

    sudo port sync
    sudo port install py27-pyplusplus-devel
