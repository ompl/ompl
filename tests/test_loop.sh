#!/bin/bash

# This script simply runs a test many times in gdb
TEST_EXE=../build/bin/test_2dmap_geometric

COUNT=100
if [ $# -gt 0 ]
then
    COUNT=$1
fi

for i in $(seq $COUNT)
do
    echo "Running test $i of $COUNT" > ompl_run_test_count
    gdb --batch --ex run --args $TEST_EXE
done
rm ompl_run_test_count
