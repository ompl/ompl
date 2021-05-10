#!/bin/bash

if [ ! -e tests/regression_tests/ ] ; then
    echo "Need to run this from the OMPL root source dir."
    exit
fi


TAGS=`cat tests/regression_tests/VERSIONS`
NPROC=`(nproc || sysctl -n hw.ncpu || echo 4) 2>/dev/null`

GIT_REPO=https://github.com/ompl/ompl
SRC_LOCATION=/tmp

CURRENT_DIR=`pwd`
LOG_RESULTS=${SRC_LOCATION}/ompl-`date "+%Y-%m-%d_%H:%M:%S"`-results
mkdir -p "$LOG_RESULTS" || exit

# For bogomips:
(lscpu || sysctl hw || wmic cpu list full) > "$LOG_RESULTS/cpuinfo" 2> /dev/null

rm -rf $SRC_LOCATION/ompl || exit
git clone $GIT_REPO ${SRC_LOCATION}/ompl || exit
cd ${SRC_LOCATION}/ompl || exit

for tag in $TAGS
do
    echo "Updating to $tag ..."
    rm -rf tests build
    git clean -fdx
    git checkout $tag
    git reset --hard

    echo "Building $tag ..."
    mkdir -p build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DOMPL_REGISTRATION=OFF -DPYTHON_EXEC=/usr/bin/python2.7 ..
    make -j$NPROC regression_test
    echo "Running $tag ..."
    ./bin/regression_test

    echo "Copying results for $tag ..."
    ls -1 *.log
    mv *.log "$LOG_RESULTS/"
    cd ..
done

echo "Done. Results are in $LOG_RESULTS/"
