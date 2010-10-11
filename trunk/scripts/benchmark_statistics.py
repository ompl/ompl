#!/usr/bin/env python

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

# Author: Mark Moll

from sys import argv, exit
from os.path import basename, splitext
import matplotlib
matplotlib.use('pdf')
from matplotlib.backends.backend_pdf import PdfPages 
import matplotlib.pyplot as plt
import numpy as np

def read_benchmark_log(filename):
	"""Parse a benchmark log file and return a dictionary of dictionaries 
	with all the parsed data. This dictionary can then be used to easily
	plot selected results or save them in a different file format."""
	logfile = open(filename,'r')
	result = {}
	result['num_planners'] = int(logfile.readline().split()[0])
	result['time_limit'] = float(logfile.readline().split()[0])
	result['memory_limit'] = float(logfile.readline().split()[0])
	result['total_duration'] = float(logfile.readline().split()[0])
	result['planner'] = {}
	for i in range(result['num_planners']):
		p = {}
		planner_name = logfile.readline()[:-1]
		num_properties = int(logfile.readline().split()[0])
		properties = []
		for j in range(num_properties):
			properties.append(logfile.readline()[:-1])
		num_runs = int(logfile.readline().split()[0])
		runs = np.empty( (num_runs, num_properties) )
		for j in range(num_runs):
			runs[j,:] = [np.nan if len(x)==0 else float(x) 
				for x in logfile.readline().split('; ')[:-1]]
		p['measurements'] = {}
		for j in range(len(properties)):
			p['measurements'][properties[j]] = runs[:,j]
		num_averages = int(logfile.readline().split()[0])
		averages = {}
		for j in range(num_averages):
			line = logfile.readline().split('= ')
			averages[line[0]] = float(line[1])
		p['averages'] = averages
		logfile.readline()
		result['planner'][planner_name] = p
	logfile.close()
	return result
	
def plot_attribute(data, attribute):
	"""Create a box plot for a particular attribute. It will include data for
	all planners that have data for this attribute."""
	plt.clf()
	ax = plt.gca()
	labels = []
	measurements = []
	nan_counts = []
	for planner, stats in data['planner'].items():
		if stats['measurements'].has_key(attribute):
			nan_count = 0
			cur_measurements = []
			for m in stats['measurements'][attribute]:
				if np.isnan(m):
					nan_count = nan_count+1
				else:
					cur_measurements.append(m)
			labels.append(planner)
			measurements.append(cur_measurements)
			nan_counts.append(nan_count)
	bp = plt.boxplot(measurements, notch=0, sym='k+', vert=1, whis=1.5)
	xtickNames = plt.setp(ax,xticklabels=labels)
	plt.setp(xtickNames, rotation=30)
	ax.set_xlabel('Motion planning algorithm')
	ax.set_ylabel(attribute)
	ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
	if max(nan_counts)>0:
		maxy = max([max(y) for y in measurements])
		for i in range(len(labels)):
			ax.text(i+1, .95*maxy, str(nan_counts[i]), horizontalalignment='center', size='small')
	plt.show()
	
def plot_statistics(fname, data):
	"""Create a PDF file with box plots for all attributes."""
	pp = PdfPages(fname)
	attributes = data['planner'].items()[0][1]['measurements'].keys()
	attributes.sort()
	for attribute in attributes:
		plot_attribute(data,attribute)
		pp.savefig(plt.gcf())
	pp.close()

def save_as_sql(fname, data):
	sqldump = open(fname+".sql",'w')
	for planner, stats in data['planner'].items():		
		fields = ", ".join(map(lambda x: "`" + x + "` DOUBLE NULL", stats["measurements"]))
		table_cmd = "DROP TABLE IF EXISTS `" + planner + "`;\nCREATE TABLE `"+planner+"` (\n" + fields + ");\n"
		sqldump.write(table_cmd)
		runs = max(map(lambda x : len(stats["measurements"][x]), stats["measurements"]))
		for r in range(runs):
			values = ", ".join(map(lambda x : "'" + str(stats["measurements"][x][r]) + "'", stats["measurements"]))
			names = ", ".join(map(lambda x: "`" + x + "`", stats["measurements"]))
			sqldump.write("INSERT INTO `" + planner + "` (" + names + ") VALUES(" + values + ");\n")
	sqldump.close()
	
if __name__ == "__main__":
	if len(argv) < 2:
		print "Usage: benchmark_statistics.py <benchmark.log> [<filename>.<sql|m>]"
		exit()
	
	data = read_benchmark_log(argv[1])
	
	if len(argv) > 2:
		(fname, ext) = argv[2].split(".")
		if ext == "sql":
			print "Generating SQL dump in " + argv[2]
			save_as_sql(fname, data)
	else:
		plot_statistics(splitext(basename(argv[1]))[0]+'.pdf', data)
	
