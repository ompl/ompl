#!/bin/bash

control_c()
# run if user hits control-c
{
  echo -en "\n*** User requested termination ***\n"
  exit 1
}

# This script simply runs a test many times in gdb
TEST_EXE=../build/bin/test_2denvs_geometric

COUNT=100
if [ $# -gt 0 ]
then
    COUNT=$1
fi

trap control_c SIGINT
rm -f ompl_run_test_codes

for i in $(seq $COUNT)
do
    echo "Running test $i of $COUNT" > ompl_run_test_count
    gdb --return-child-result --batch --ex run --args $TEST_EXE
    RET_CODE=$?
    echo $RET_CODE >> ompl_run_test_codes
    if [ $RET_CODE -ne 0 ]
    then
        echo "Error found!"
    fi
done
rm -f ompl_run_test_count
