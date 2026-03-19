#!/usr/bin/env python3

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Mark Moll, Ioan Sucan, Luis G. Torres

import argparse
from pathlib import Path

from ompl.tools import (
    readBenchmarkLog,
    computeViews,
    plotStatistics,
    saveAsMysql,
)

def main():
    parser = argparse.ArgumentParser(
        description='Process benchmark logs and create an SQLite3 database.')
    parser.add_argument('-d', '--database', default='benchmark.db', \
        help='Filename of benchmark database')
    parser.add_argument('-a', '--append', action='store_true', default=False, \
        help='Append data to database (as opposed to overwriting an existing database)')
    parser.add_argument('-v', '--view', action='store_true', default=False, \
        help='Compute the views for best planner configurations')
    parser.add_argument('-m', '--mysql', action='store_true', default=False, \
        help='Save SQLite3 database as a MySQL dump file')
    parser.add_argument('--moveit', action='store_true', default=False, \
        help='Log files are produced by MoveIt!')
    parser.add_argument('logfile', nargs='*')
    args = parser.parse_args()

    if not args.append and Path(args.database).exists() and args.logfile:
        Path(args.database).unlink()

    if args.logfile:
        readBenchmarkLog(args.database, args.logfile, args.moveit)
        # If we update the database, we recompute the views as well
        args.view = True

    if args.view:
        computeViews(args.database, args.moveit)

    if args.mysql:
        saveAsMysql(args.database)

if __name__ == '__main__':
    main()
