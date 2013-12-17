# cmake's file(DOWNLOAD ...) command doesn't do https until version 2.8.10
file(DOWNLOAD
    https://github.com/gccxml/gccxml/archive/b040a46352e4d5c11a0304e4fcb6f7842008942a.tar.gz
    "${TEMPDIR}/gccxml-2013-09-19.tgz" SHOW_PROGRESS)
file(DOWNLOAD
    https://bitbucket.org/ompl/pygccxml/downloads/pygccxml-r579.tgz
    "${TEMPDIR}/pygccxml-r579.tgz" SHOW_PROGRESS)
file(DOWNLOAD
    https://bitbucket.org/ompl/pyplusplus/downloads/pyplusplus-r1246.tgz
    "${TEMPDIR}/pyplusplus-r1246.tgz" SHOW_PROGRESS)
