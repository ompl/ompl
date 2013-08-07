#!/bin/bash
# The typical use will be watch ./test_results.sh
echo 
echo -n "Set of return codes so far: [ "
cat ompl_run_test_codes | sort | uniq | tr "\\n" " "
echo "]"
cat ompl_run_test_count && cat ompl_run_test_codes | perl -e 'print reverse <>'
