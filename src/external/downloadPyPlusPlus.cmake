# cmake's file(DOWNLOAD ...) command doesn't do https until version 2.8.10
file(DOWNLOAD
    https://github.com/gccxml/gccxml/archive/567213ac765c99d5dfd23b14000b3c7b76274fcb.tar.gz
    "${TEMPDIR}/gccxml-2013-05-02.tgz" SHOW_PROGRESS)
file(DOWNLOAD
    https://bitbucket.org/ompl/pygccxml/downloads/pygccxml-r579.tgz
    "${TEMPDIR}/pygccxml-r579.tgz" SHOW_PROGRESS)
file(DOWNLOAD
    https://bitbucket.org/ompl/pyplusplus/downloads/pyplusplus-r1246.tgz
    "${TEMPDIR}/pyplusplus-r1246.tgz" SHOW_PROGRESS)
