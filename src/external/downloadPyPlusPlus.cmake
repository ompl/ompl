# cmake's file(DOWNLOAD ...) command doesn't do https until version 2.8.10
file(DOWNLOAD
    https://github.com/gccxml/gccxml/archive/ab651a2aa866351bdd089a4bf1d57f6a9bec2a66.tar.gz
    "${TEMPDIR}/gccxml.tgz" SHOW_PROGRESS)
file(DOWNLOAD
    https://github.com/gccxml/pygccxml/archive/v1.6.1.tar.gz
    "${TEMPDIR}/pygccxml.tgz" SHOW_PROGRESS)
file(DOWNLOAD
    https://bitbucket.org/ompl/pyplusplus/downloads/pyplusplus-r1246.tgz
    "${TEMPDIR}/pyplusplus.tgz" SHOW_PROGRESS)
