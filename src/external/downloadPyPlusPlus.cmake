file(DOWNLOAD
    http://downloads.sourceforge.net/project/ompl/dependencies/gccxml-2012-05-17.tgz
    "${TEMPDIR}/gccxml-2012-05-17.tgz"
    SHOW_PROGRESS
    EXPECTED_MD5 e2ad8bd675af9009530e5e175ed441bc)
file(DOWNLOAD
    https://bitbucket.org/ompl/pygccxml/downloads/pygccxml-r575.tgz
    "${TEMPDIR}/pygccxml-r575.tgz"
    SHOW_PROGRESS
    EXPECTED_MD5 925e5200b1479af724b2e301fd12ef92)
file(DOWNLOAD
    https://bitbucket.org/ompl/pyplusplus/downloads/pyplusplus-r1238.tgz
    "${TEMPDIR}/pyplusplus-r1238.tgz"
    SHOW_PROGRESS
    EXPECTED_MD5 17bddfb3418ac7d4f3eb87f6408b06e8)
