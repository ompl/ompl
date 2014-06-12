#!/bin/bash

if [ ! -e regression_tests/ ] ; then
    echo "Need to run this from the OMPL root source dir."
    exit
fi

ompl_major_version() {
    grep "set(OMPL_MAJOR_VERSION" CMakeModules/OMPLVersion.cmake | sed 's/[^0-9]//g'
}

ompl_minor_version() {
    grep "set(OMPL_MINOR_VERSION" CMakeModules/OMPLVersion.cmake | sed 's/[^0-9]//g'
}

TAGS="
0.9.4
0.9.5
0.10.0
0.10.1
0.10.2
0.11.0
0.11.1
0.12.0
0.12.1
0.12.2
0.13.0
0.14.0
0.14.1
0.14.2
"

HG_REPO=https://bitbucket.org/ompl/ompl
SRC_LOCATION=/tmp/

CURRENT_DIR=`pwd`
LOG_RESULTS=${SRC_LOCATION}/ompl-`date "+%Y-%m-%d_%H:%M:%S"`-results
mkdir "$LOG_RESULTS"

hg clone $HG_REPO $SRC_LOCATION/ompl
cd $SRC_LOCATION/ompl

for tag in $TAGS
do
    echo "Updating to $tag ..."
    rm -rf regression_tests tests build
    hg revert --all
    hg up $tag
    echo "Patching $tag ..."
    cp -r "$CURRENT_DIR/regression_tests" .
    cp -r "$CURRENT_DIR/tests" .

    # Add build rule if needed.
    if [ -f CMakeModules/OMPLVersion.cmake ] ; then
    OMPL_MAJOR_VERSION=$(ompl_major_version)
    OMPL_MINOR_VERSION=$(ompl_minor_version)
    if [ $OMPL_MAJOR_VERSION -lt 1 ] ; then
        if [ $OMPL_MINOR_VERSION -lt 15 ] ; then
        echo "add_subdirectory(regression_tests)" >> CMakeLists.txt
        fi
    fi
    fi

    # No need to build tests or demos
    echo "" > tests/CMakeLists.txt
    echo "" > demos/CMakeLists.txt
    if [ -e src/ompl/contrib/rrt_star/CMakeLists.txt ] ; then
    echo "" > src/ompl/contrib/rrt_star/CMakeLists.txt
    fi

    echo "Building $tag ..."
    mkdir -p build
    cd build
    cmake ..
    make -j4
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
    echo "Copying results for $tag ..."
    ls -1 *.log
    mv *.log "$LOG_RESULTS/"
    cd ..
done
