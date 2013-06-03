#!/bin/bash
# The typical use will be watch ./test_results.sh
cat ompl_run_test_count && cat ompl_run_test_codes | perl -e 'print reverse <>'
