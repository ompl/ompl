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

USE_GDB=false
if [ $# -gt 1 ]
then
    USE_GDB=true
    echo "Using gdb."
fi

ulimit -c unlimited

trap control_c SIGINT
rm -f ompl_run_test_codes

for i in $(seq $COUNT)
do
    echo "Running test $i of $COUNT" > ompl_run_test_count
    if $USE_GDB ; then
	gdb --return-child-result --batch --ex run --args $TEST_EXE
    else
	$TEST_EXE
    fi
    RET_CODE=$?
    echo $RET_CODE >> ompl_run_test_codes
    if [ $RET_CODE -ne 0 ]
    then
        echo "Error found!"
	if [ -e core ]
	then
	    echo "Renaming core file."
	    mv core core.$i
	fi
    fi
done
rm -f ompl_run_test_count
