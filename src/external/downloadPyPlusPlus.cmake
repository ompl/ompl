# cmake's file(DOWNLOAD ...) command doesn't do https until version 2.8.10
file(DOWNLOAD
    https://github.com/gccxml/gccxml/archive/567213ac765c99d5dfd23b14000b3c7b76274fcb.tar.gz
    "${TEMPDIR}/gccxml-2013-05-02.tgz"
    SHOW_PROGRESS
    EXPECTED_MD5 30f30e7cb246b8f00e58fa8562ce1c4c)
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
