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
import sqlite3
import datetime
import matplotlib
matplotlib.use('pdf')
from matplotlib import __version__ as matplotlibversion
from matplotlib.backends.backend_pdf import PdfPages 
import matplotlib.pyplot as plt
import numpy as np
from optparse import OptionParser, OptionGroup

def read_benchmark_log(dbname, filenames):
	"""Parse benchmark log files and store the parsed data in a sqlite3 database."""

	conn = sqlite3.connect(dbname)
	c = conn.cursor()
	c.execute('PRAGMA FOREIGN_KEYS = ON')
	c.execute("""CREATE TABLE IF NOT EXISTS experiments
		(id INTEGER PRIMARY KEY AUTOINCREMENT, name VARCHAR(512), totaltime REAL, timelimit REAL, memorylimit REAL, hostname VARCHAR(1024), date DATETIME, setup TEXT)""")
	c.execute("""CREATE TABLE IF NOT EXISTS planners
		(id INTEGER PRIMARY KEY AUTOINCREMENT, name VARCHAR(512) NOT NULL, settings TEXT)""")
	for filename in filenames:
		print "Processing " + filename
		logfile = open(filename,'r')
		expname =  logfile.readline().split()[-1]
		hostname = logfile.readline().split()[-1]
		date = " ".join(logfile.readline().split()[2:])
		logfile.readline() # skip <<<|
		expsetup = ""
		expline = logfile.readline()
		while not expline.startswith("|>>>"):
			expsetup = expsetup + expline
			expline = logfile.readline()
		timelimit = float(logfile.readline().split()[0])
		memorylimit = float(logfile.readline().split()[0])
		totaltime = float(logfile.readline().split()[0])

		c.execute('INSERT INTO experiments VALUES (?,?,?,?,?,?,?,?)',
			  (None, expname, totaltime, timelimit, memorylimit, hostname, date, expsetup) )
		c.execute('SELECT last_insert_rowid()')
		experiment_id = c.fetchone()[0]
		num_planners = int(logfile.readline().split()[0])

		for i in range(num_planners):
			planner_name = logfile.readline()[:-1]
			print "Parsing data for", planner_name
			
			# read common data for planner
			num_common = int(logfile.readline().split()[0])
			settings = ""
			for j in range(num_common):
				settings = settings + logfile.readline()

			# find planner id
			c.execute("SELECT id FROM planners WHERE (name=? AND settings=?)", (planner_name, settings,))
			p = c.fetchone()
			if p==None:
				c.execute("INSERT INTO planners VALUES (?,?,?)", (None, planner_name, settings,))
				c.execute('SELECT last_insert_rowid()')
				planner_id = c.fetchone()[0]
			else:
				planner_id = p[0]
			
			# read run properties

			# number of properties to read from log file
			num_properties = int(logfile.readline().split()[0])

			# load a dictionary of properties and types
			# we keep the names of the properties in a list as well, to ensure the correct order of properties
			properties = {}
			propNames = ['experimentid', 'plannerid']
			for j in range(num_properties):
				field = logfile.readline().split()
				ftype = field[-1]
				fname = "_".join(field[:-1])
				properties[fname] = ftype
				propNames.append(fname)

			# create the table, if needed
			table_columns = "experimentid INTEGER, plannerid INTEGER"
			for k, v in properties.iteritems():
				table_columns = table_columns + ', ' + k + ' ' + v
			table_columns = table_columns + ", FOREIGN KEY(experimentid) REFERENCES experiments(id) ON DELETE CASCADE"
			table_columns = table_columns + ", FOREIGN KEY(plannerid) REFERENCES planners(id) ON DELETE CASCADE"

			planner_table = 'planner_%s' % planner_name
			c.execute("CREATE TABLE IF NOT EXISTS %s (%s)" %  (planner_table, table_columns))
			
			# check if the table has all the needed columns; if not, add them
			c.execute('SELECT * FROM %s' % planner_table)
			added_columns = [ t[0] for t in c.description]
			for col in properties.keys():
				if not col in added_columns:
					c.execute('ALTER TABLE ' + planner_table + ' ADD ' + col + ' ' + properties[col] + ';')
			
			# add measurements 
			insert_fmt_str = 'INSERT INTO ' + planner_table + ' (' + ','.join(propNames) + ') VALUES (' + ','.join('?'*(num_properties + 2)) + ')'

			num_runs = int(logfile.readline().split()[0])
			for j in range(num_runs):
				run = tuple([experiment_id, planner_id] + [None if len(x)==0 else float(x) 
					for x in logfile.readline().split('; ')[:-1]])
				c.execute(insert_fmt_str, run)
			
			logfile.readline()
		logfile.close()
	conn.commit()
	c.close()
	
def plot_attribute(cur, planners, attribute, typename):
	"""Create a box plot for a particular attribute. It will include data for
	all planners that have data for this attribute."""
	plt.clf()
	ax = plt.gca()
	labels = []
	measurements = []
	nan_counts = []
	is_bool = True
	for planner in planners:
		cur.execute('SELECT * FROM %s' % planner)
		attributes = [ t[0] for t in cur.description]
 		if attribute in attributes:
			cur.execute('SELECT %s FROM %s' % (attribute, planner))
			result = [ t[0] for t in cur.fetchall() ]
			nan_counts.append(len([x for x in result if x==None]))
			measurement = [x for x in result if not x==None]
			cur.execute('SELECT DISTINCT %s FROM %s' % (attribute, planner))
			is_bool = is_bool and set([t[0] for t in cur.fetchall() if not t[0]==None]).issubset(set([0,1]))
			measurements.append(measurement)
			labels.append(planner.replace('planner_',''))
	if is_bool:
		width = .5
		measurements_percentage = [sum(m)*100./len(m) for m in measurements]
		ind = range(len(measurements))
		plt.bar(ind, measurements_percentage, width)
		xtickNames = plt.xticks([x+width/2. for x in ind], labels, rotation=30)
		ax.set_ylabel(attribute.replace('_',' ') + ' (%)')
	else:
		if int(matplotlibversion.split('.')[0])<1:
			plt.boxplot(measurements, notch=0, sym='k+', vert=1, whis=1.5)
		else:
			plt.boxplot(measurements, notch=0, sym='k+', vert=1, whis=1.5, bootstrap=1000)
		ax.set_ylabel(attribute.replace('_',' '))
		xtickNames = plt.setp(ax,xticklabels=labels)
		plt.setp(xtickNames, rotation=30)
	ax.set_xlabel('Motion planning algorithm')
	ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
	if max(nan_counts)>0:
		maxy = max([max(y) for y in measurements])
		for i in range(len(labels)):
			x = i+width/2 if is_bool else i+1
			ax.text(x, .95*maxy, str(nan_counts[i]), horizontalalignment='center', size='small')
	plt.show()
	
def plot_statistics(dbname, fname):
	"""Create a PDF file with box plots for all attributes."""
	conn = sqlite3.connect(dbname)
	c = conn.cursor()
	c.execute('PRAGMA FOREIGN_KEYS = ON')
	c.execute("SELECT name FROM sqlite_master WHERE type='table'")
	table_names = [ str(t[0]) for t in c.fetchall() ]
	planner_names = [ t for t in table_names if t.startswith('planner_') ]
        attributes = []
	types = {}
	# merge possible attributes from all planners
        for p in planner_names:
                c.execute('SELECT * FROM %s' % p)
                atr = [ t[0] for t in c.description]
                atr.remove('plannerid')
                atr.remove('experimentid')
                for a in atr:
                        if a not in attributes:
                                c.execute('SELECT typeof(%s) FROM %s' % (a, p))
                                attributes.append(a)
                                types[a] = c.fetchone()[0]
        attributes.sort()

	pp = PdfPages(fname)
	for atr in attributes:
		if types[atr]=='integer' or types[atr]=='real':
			plot_attribute(c, planner_names, atr, types[atr])
			pp.savefig(plt.gcf())
	pp.close()

def save_as_mysql(dbname, mysqldump):
	# See http://stackoverflow.com/questions/1067060/perl-to-python
	import re
	conn = sqlite3.connect(dbname)
	mysqldump = open(mysqldump,'w')
	
	# make sure all tables are dropped in an order that keepd foreign keys valid
	c = conn.cursor()
	c.execute("SELECT name FROM sqlite_master WHERE type='table'")
	table_names = [ str(t[0]) for t in c.fetchall() ]
	c.close()
	last = ['experiments', 'planners']
	for table in table_names:
		if table.startswith("sqlite"):
			continue
		if not table in last:
			mysqldump.write("DROP TABLE IF EXISTS `%s`;\n" % table)
	for table in last:
		if table in table_names:
			mysqldump.write("DROP TABLE IF EXISTS `%s`;\n" % table)

	for line in conn.iterdump():
		process = False
		for nope in ('BEGIN TRANSACTION','COMMIT',
			'sqlite_sequence','CREATE UNIQUE INDEX'):
			if nope in line: break
	  	else:
			process = True
		if not process: continue
		line = re.sub(r"[\n\r\t ]+", " ", line)
		m = re.search('CREATE TABLE ([a-zA-Z0-9_]*)(.*)', line)
		if m:
			name, sub = m.groups()
			sub = sub.replace('"','`')
			line = '''CREATE TABLE IF NOT EXISTS %(name)s%(sub)s'''
			line = line % dict(name=name, sub=sub)
			# make sure we use an engine that supports foreign keys
			line = line.rstrip("\n\t ;") + " ENGINE = InnoDB;\n"
		else:
			m = re.search('INSERT INTO "([a-zA-Z0-9_]*)"(.*)', line)
			if m:
				line = 'INSERT INTO %s%s\n' % m.groups()
				line = line.replace('"', r'\"')
				line = line.replace('"', "'")

		line = re.sub(r"([^'])'t'(.)", "\\1THIS_IS_TRUE\\2", line)
		line = line.replace('THIS_IS_TRUE', '1')
		line = re.sub(r"([^'])'f'(.)", "\\1THIS_IS_FALSE\\2", line)
		line = line.replace('THIS_IS_FALSE', '0')
		line = line.replace('AUTOINCREMENT', 'AUTO_INCREMENT')
		mysqldump.write(line)
	mysqldump.close()
	

if __name__ == "__main__":
	usage = """%prog [options] [<benchmark.log> ...]"""
	parser = OptionParser(usage)
	parser.add_option("-d", "--database", dest="dbname", default="benchmark.db",
		help="Filename of benchmark database [default: %default]")
	parser.add_option("-p", "--plot", dest="plot", default=None,
		help="Create a PDF of plots")
	parser.add_option("-m", "--mysql", dest="mysqldb", default=None,
		help="Save SQLite3 database as a MySQL dump file")
	(options, args) = parser.parse_args()
	
	if len(args)>0:
		read_benchmark_log(options.dbname, args)

	if options.plot:
		plot_statistics(options.dbname, options.plot)

	if options.mysqldb:
		save_as_mysql(options.dbname, options.mysqldb)

