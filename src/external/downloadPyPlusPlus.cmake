file(DOWNLOAD
    http://downloads.sourceforge.net/project/ompl/dependencies/gccxml-2012-05-17.tgz
    "${TEMPDIR}/gccxml-2012-05-17.tgz"
    SHOW_PROGRESS
    EXPECTED_MD5 e2ad8bd675af9009530e5e175ed441bc)
file(DOWNLOAD
    http://downloads.sourceforge.net/project/ompl/dependencies/pygccxml-r1856.tgz
    "${TEMPDIR}/pygccxml-r1856.tgz"
    SHOW_PROGRESS
    EXPECTED_MD5 144b9f6bb329312fa7e0d73ea92514b5)
file(DOWNLOAD
    http://downloads.sourceforge.net/project/ompl/dependencies/pyplusplus-r1856.tgz
    "${TEMPDIR}/pyplusplus-r1856.tgz"
    SHOW_PROGRESS
    EXPECTED_MD5 d41d8cd98f00b204e9800998ecf8427e)