#!/bin/bash

if [ ! -e tests/regression_tests/ ] ; then
    echo "Need to run this from the OMPL root source dir."
    exit
fi

ompl_major_version() {
    grep "set(OMPL_MAJOR_VERSION" CMakeModules/OMPLVersion.cmake | sed 's/[^0-9]//g'
}

ompl_minor_version() {
    grep "set(OMPL_MINOR_VERSION" CMakeModules/OMPLVersion.cmake | sed 's/[^0-9]//g'
}

TAGS=`cat tests/regression_tests/VERSIONS`
NPROC=`(nproc || sysctl -n hw.ncpu || echo 4) 2>/dev/null`

HG_REPO=https://bitbucket.org/ompl/ompl
SRC_LOCATION=/tmp

CURRENT_DIR=`pwd`
LOG_RESULTS=${SRC_LOCATION}/ompl-`date "+%Y-%m-%d_%H:%M:%S"`-results
mkdir -p "$LOG_RESULTS" || exit

# For bogomips:
(lscpu || sysctl hw || wmic cpu list full) > "$LOG_RESULTS/cpuinfo" 2> /dev/null

rm -rf $SRC_LOCATION/ompl || exit
hg clone $HG_REPO ${SRC_LOCATION}/ompl || exit
cd ${SRC_LOCATION}/ompl || exit

for tag in $TAGS
do
    echo "Updating to $tag ..."
    rm -rf tests build
    hg clean
    hg revert --all
    hg up $tag
    OMPL_MAJOR_VERSION=$(ompl_major_version)
    OMPL_MINOR_VERSION=$(ompl_minor_version)
    echo "Patching $tag ..."
    cp -r "$CURRENT_DIR/tests" .
    if ! grep -ql Boost_SYSTEM_LIBRARY src/ompl/CMakeLists.txt ; then
        echo 'target_link_libraries(ompl ${Boost_SYSTEM_LIBRARY})' >> src/ompl/CMakeLists.txt
    fi
    if ! grep -ql Boost_THREAD_LIBRARY src/ompl/CMakeLists.txt ; then
        echo 'target_link_libraries(ompl ${Boost_THREAD_LIBRARY})' >> src/ompl/CMakeLists.txt
    fi
    if ! grep -ql Boost_SERIALIZATION_LIBRARY src/ompl/CMakeLists.txt ; then
        echo 'target_link_libraries(ompl ${Boost_SERIALIZATION_LIBRARY})' >> src/ompl/CMakeLists.txt
    fi
    sed -i "s/^#if defined _POSIX_VERSION$/#if defined _POSIX_VERSION || defined _POSIX2_VERSION || defined __linux__/" src/ompl/tools/benchmark/src/MachineSpecs.cpp

    echo "Building $tag ..."
    mkdir -p build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DOMPL_REGISTRATION=OFF -DPYTHON_EXEC=/usr/bin/python2.7 ..
    make -j$NPROC regression_test
    echo "Running $tag ..."
    ./bin/regression_test

    # add OMPL version number to top of the log file
    if [ $OMPL_MAJOR_VERSION -lt 1 ] ; then
        if [ $OMPL_MINOR_VERSION -lt 15 ] ; then
            for log in *.log; do
                echo OMPL version $tag > .tmp.log
                cat $log >> .tmp.log
                mv .tmp.log $log
            done
        fi
    fi
    echo "Copying results for $tag ..."
    ls -1 *.log
    mv *.log "$LOG_RESULTS/"
    cd ..
done

echo "Done. Results are in $LOG_RESULTS/"
