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

# Author: Mark Moll, Ioan Sucan, Luis G. Torres

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
from math import floor
from optparse import OptionParser, OptionGroup

def read_benchmark_log(dbname, filenames):
    """Parse benchmark log files and store the parsed data in a sqlite3 database."""

    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute('PRAGMA FOREIGN_KEYS = ON')
    c.execute("""CREATE TABLE IF NOT EXISTS experiments
        (id INTEGER PRIMARY KEY AUTOINCREMENT, name VARCHAR(512), totaltime REAL, timelimit REAL, memorylimit REAL, runcount INTEGER, hostname VARCHAR(1024), date DATETIME, seed INTEGER, setup TEXT)""")
    c.execute("""CREATE TABLE IF NOT EXISTS planner_configs
        (id INTEGER PRIMARY KEY AUTOINCREMENT, planner_name VARCHAR(512) NOT NULL, settings TEXT)""")
    c.execute("""CREATE TABLE IF NOT EXISTS enums
        (name VARCHAR(512), value INTEGER, description TEXT, PRIMARY KEY (name,value))""")
    for filename in filenames:
        print("Processing " + filename)
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
        rseed = int(logfile.readline().split()[0])
        timelimit = float(logfile.readline().split()[0])
        memorylimit = float(logfile.readline().split()[0])
        nrruns = float(logfile.readline().split()[0])
        totaltime = float(logfile.readline().split()[0])
        num_enums = int(logfile.readline().split()[0])
        for i in range(num_enums):
            enum = logfile.readline()[:-1].split('|')
            c.execute('SELECT * FROM enums WHERE name IS "%s"' % enum[0])
            if c.fetchone()==None:
                for j in range(len(enum)-1):
                    c.execute('INSERT INTO enums VALUES (?,?,?)',
                        (enum[0],j,enum[j+1]))
        c.execute('INSERT INTO experiments VALUES (?,?,?,?,?,?,?,?,?,?)',
              (None, expname, totaltime, timelimit, memorylimit, nrruns, hostname, date, rseed, expsetup) )
        c.execute('SELECT last_insert_rowid()')
        experiment_id = c.fetchone()[0]
        num_planners = int(logfile.readline().split()[0])

        for i in range(num_planners):
            planner_name = logfile.readline()[:-1]
            print("Parsing data for " + planner_name)

            # read common data for planner
            num_common = int(logfile.readline().split()[0])
            settings = ""
            for j in range(num_common):
                settings = settings + logfile.readline() + ';'

            # find planner id
            c.execute("SELECT id FROM planner_configs WHERE (planner_name=? AND settings=?)", (planner_name, settings,))
            p = c.fetchone()
            if p==None:
                c.execute("INSERT INTO planner_configs VALUES (?,?,?)", (None, planner_name, settings,))
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
            propNames = ['id', 'experimentid', 'plannerid']
            for j in range(num_properties):
                field = logfile.readline().split()
                ftype = field[-1]
                fname = "_".join(field[:-1])
                properties[fname] = ftype
                propNames.append(fname)

            # create the table, if needed
            table_columns = "id INTEGER PRIMARY KEY AUTOINCREMENT, experimentid INTEGER, plannerid INTEGER"
            for k, v in properties.items():
                table_columns = table_columns + ', ' + k + ' ' + v
            table_columns = table_columns + ", FOREIGN KEY(experimentid) REFERENCES experiments(id) ON DELETE CASCADE"
            table_columns = table_columns + ", FOREIGN KEY(plannerid) REFERENCES planner_configs(id) ON DELETE CASCADE"

            planner_table = 'planner_%s' % planner_name
            c.execute("CREATE TABLE IF NOT EXISTS `%s` (%s)" %  (planner_table, table_columns))

            # check if the table has all the needed columns; if not, add them
            c.execute('SELECT * FROM `%s`' % planner_table)
            added_columns = [ t[0] for t in c.description]
            for col in properties.keys():
                if not col in added_columns:
                    c.execute('ALTER TABLE `' + planner_table + '` ADD ' + col + ' ' + properties[col] + ';')

            # add measurements
            insert_fmt_str = 'INSERT INTO `' + planner_table + '` (' + ','.join(propNames) + ') VALUES (' + ','.join('?'*(num_properties + 3)) + ')'

            num_runs = int(logfile.readline().split()[0])
            run_ids = []
            for j in range(num_runs):
                run = tuple([None, experiment_id, planner_id] + [None if len(x)==0 else float(x)
                    for x in logfile.readline().split('; ')[:-1]])
                c.execute(insert_fmt_str, run)
                
                # extract primary keys of each run row so we can
                # reference them in the planner progress data table if
                # needed
                c.execute('SELECT last_insert_rowid()')
                run_ids.append(c.fetchone()[0])
                
            nextLine = logfile.readline().strip()

            # Read in planner progress data if it's supplied
            if nextLine != '.':
                num_prog_props = int(nextLine.split()[0])
                prog_prop_names = []
                prog_prop_types = []
                for i in range(num_prog_props):
                    field = logfile.readline().split()
                    prog_prop_types.append(field[-1])
                    prog_prop_names.append("_".join(field[:-1]))
            
                # create the table for run progress properties of this planner
                #
                # \TODO: do we need more disambiguating info in table
                # name like exp id?  
                #
                # \TODO: might consider indexing on
                # runid+time if things start taking too long
                table_name = planner_name + '_planner_progress'
                table_columns = 'runid INTEGER'
                table_columns += ''.join([', %s %s' % (pname,ptype) for 
                                          (pname,ptype) in zip(prog_prop_names,prog_prop_types)])
                table_columns += ', FOREIGN KEY(runid) REFERENCES %s(id)' % planner_table 
                c.execute("CREATE TABLE IF NOT EXISTS `%s` (%s)" % (table_name, table_columns))

                num_runs = int(logfile.readline().split()[0])
                insert_fmt_str = 'INSERT INTO `' + table_name + '` (runid,' + ','.join(prog_prop_names) + ') VALUES (' + ','.join('?'*(num_prog_props+1)) + ')'
                for j in range(num_runs):
                    data_series = logfile.readline().split(';')[:-1]
                    for data_sample in data_series:
                        # \TODO don't really like always using float() here;
                        # should be able to dispatch depending on data
                        # type
                        values = tuple([run_ids[j]]+[float(x) for x in data_sample.split(',')[:-1]])
                        c.execute(insert_fmt_str, values)
                    
                logfile.readline()
        logfile.close()
    conn.commit()
    c.close()

def plot_attribute(cur, planners, attribute, typename):
    """Create a plot for a particular attribute. It will include data for
    all planners that have data for this attribute."""
    plt.clf()
    ax = plt.gca()
    labels = []
    measurements = []
    nan_counts = []
    cur.execute('SELECT count(*) FROM enums where name IS "%s"' % attribute)
    num_vals = cur.fetchone()[0]
    is_enum = False if num_vals==0 else True
    is_bool = not is_enum
    for planner in planners:
        cur.execute('SELECT * FROM `%s`' % planner)
        attributes = [ t[0] for t in cur.description]
        if attribute in attributes:
            cur.execute('SELECT `%s` FROM `%s` WHERE `%s` IS NOT NULL' % (attribute, planner, attribute))
            measurement = [ 0 if np.isinf(t[0]) else t[0] for t in cur.fetchall() ]
            cur.execute('SELECT count(*) FROM `%s` WHERE `%s` IS NULL' % (planner, attribute))
            nan_counts.append(cur.fetchone()[0])
            cur.execute('SELECT DISTINCT `%s` FROM `%s`' % (attribute, planner))
            is_bool = is_bool and set([t[0] for t in cur.fetchall() if not t[0]==None]).issubset(set([0,1]))
            if is_enum:
                scale = 100. / len(measurement)
                measurements.append([measurement.count(i)*scale for i in range(num_vals)])
            else:
                measurements.append(measurement)
            labels.append(planner.replace('planner_geometric_','').replace('planner_control_',''))

    if is_enum:
        width = .5
        measurements = np.transpose(np.vstack(measurements))
        colsum = np.sum(measurements, axis=1)
        rows = np.where(colsum != 0)[0]
        heights = np.zeros((1,measurements.shape[1]))
        ind = range(measurements.shape[1])
        legend_labels = []
        for i in rows:
            cur.execute('SELECT `description` FROM enums WHERE name IS "%s" AND value IS "%d"' % (attribute,i))
            plt.bar(ind, measurements[i], width, bottom=heights[0],
                color=matplotlib.cm.hot(int(floor(i*256/num_vals))), label=cur.fetchone()[0])
            heights = heights + measurements[i]
        xtickNames = plt.xticks([x+width/2. for x in ind], labels, rotation=30)
        ax.set_ylabel(attribute.replace('_',' ') + ' (%)')
        box = ax.get_position()
        ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])
        props = matplotlib.font_manager.FontProperties()
        props.set_size('small')
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), prop = props)
    elif is_bool:
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
        plt.setp(xtickNames, rotation=25)
    ax.set_xlabel('Motion planning algorithm')
    ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
    if max(nan_counts)>0:
        maxy = max([max(y) for y in measurements])
        for i in range(len(labels)):
            x = i+width/2 if is_bool else i+1
            ax.text(x, .95*maxy, str(nan_counts[i]), horizontalalignment='center', size='small')
    plt.show()

def plot_progress_attribute(cur, table_names, attribute):
    """Plot data for a single planner progress attribute. Will create an
average time-plot with error bars of the attribute over all runs for
each planner."""        

    import numpy.ma as ma

    plt.clf()
    ax = plt.gca()
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(attribute.replace('_',' '))
    planner_names = []
    planners = [t for t in table_names if t.endswith('planner_progress')]
    for planner in planners:
        cur.execute('SELECT * FROM `%s` LIMIT 1' % planner)
        attributes = [t[0] for t in cur.description]
        if attribute in attributes:
            planner_names.append(planner[:planner.rfind('_planner_progress')])
            cur.execute('SELECT DISTINCT runid FROM `%s`' % planner)
            runids = [t[0] for t in cur.fetchall()]
            timeTable = []
            dataTable = []
            for r in runids:
                # Select data for given run
                cur.execute('SELECT time, %s FROM `%s` WHERE runid = %s ORDER BY time' % (attribute,planner,r))
                (time, data) = zip(*(cur.fetchall()))
                timeTable.append(time)
                dataTable.append(data)
            # It's conceivable that the sampling process may have
            # generated more samples for one run than another; in this
            # case, truncate all data series to length of shortest
            # one.
            fewestSamples = min(len(time[:]) for time in timeTable)
            times = np.array(timeTable[0][:fewestSamples])
            dataArrays = np.array([data[:fewestSamples] for data in dataTable])

            # Only include time samples where all runs had data to
            # report for this attribute (no NaNs)
            # isTimeValid = np.array([True]*fewestSamples)
            # for r in dataTable:
            #     valids = np.array([e is not None for e in r])
            #     isTimeValid = np.logical_and(isTimeValid, valids)
            # filteredDataTable = []
            # for r in dataTable:
            #     filteredDataTable.append(np.array(r)[isTimeValid].tolist())
            # dataArrays = np.array(filteredDataTable)
            
            filteredData = ma.masked_array(dataArrays, np.equal(dataArrays, None), dtype=float)

            means = np.mean(filteredData, axis=0)
            stddevs = np.std(filteredData, axis=0, ddof=1)
                
            # plot average with error bars
            plt.errorbar(times, means, yerr=2*stddevs, errorevery=len(times) // 20)
            ax.legend(planner_names)
    plt.show()

def plot_statistics(dbname, fname):
    """Create a PDF file with box plots for all attributes."""
    print("Generating plot...")
    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute('PRAGMA FOREIGN_KEYS = ON')
    c.execute("SELECT name FROM sqlite_master WHERE type='table'")
    table_names = [ str(t[0]) for t in c.fetchall() ]
    planner_names = [ t for t in table_names if t.startswith('planner_') and t != 'planner_configs' ]
    attributes = []
    types = {}
    experiments = []
    # merge possible attributes from all planners
    for p in planner_names:
        c.execute('SELECT * FROM `%s` LIMIT 1' % p)
        atr = [ t[0] for t in c.description]
        atr.remove('id')
        atr.remove('plannerid')
        atr.remove('experimentid')
        for a in atr:
            if a not in attributes:
                c.execute('SELECT typeof(`%s`) FROM `%s` WHERE `%s` IS NOT NULL LIMIT 1' % (a, p, a))
                attributes.append(a)
                types[a] = c.fetchone()[0]
        c.execute('SELECT DISTINCT experimentid FROM `%s`' % p)
        eid = [t[0] for t in c.fetchall() if not t[0]==None]
        for e in eid:
            if e not in experiments:
                experiments.append(e)                
    attributes.sort()        
            
    pp = PdfPages(fname)
    for atr in attributes:
        if types[atr]=='integer' or types[atr]=='real':
            plot_attribute(c, planner_names, atr, types[atr])
            pp.savefig(plt.gcf())
    plt.clf()

    # merge possible progress attributes from all planners
    progress_table_names = [t for t in table_names if t.endswith('planner_progress')]
    prog_attributes = []
    for p in progress_table_names:
        c.execute('SELECT * FROM `%s` LIMIT 1' % p)
        atr = [t[0] for t in c.description]
        atr.remove('runid')
        atr.remove('time')
        for a in atr:
            if a not in prog_attributes:
                prog_attributes.append(a)

    for atr in prog_attributes:
        plot_progress_attribute(c, table_names, atr)
        pp.savefig(plt.gcf())
    plt.clf()

    pagey = 0.9
    pagex = 0.06
    for e in experiments:
        # get the number of runs, per planner, for this experiment
        runcount = []
        for p in planner_names:
            c.execute('SELECT count(*) FROM `%s` WHERE experimentid = %s' % (p, e))
            runcount.append(c.fetchone()[0])

        # check if this number is the same for all planners
        runs = "Number of averaged runs: "
        if len([r for r in runcount if not r == runcount[0]]) > 0:
            runs = runs + ", ".join([planner_names[i].replace('planner_geometric_','').replace('planner_control_','') +
                         "=" + str(runcount[i]) for i in range(len(runcount))])
        else:
            runs = runs + str(runcount[0])

        c.execute('SELECT name, timelimit, memorylimit FROM experiments WHERE id = %s' % e)
        d = c.fetchone()
        plt.figtext(pagex, pagey, "Experiment '%s'" % d[0])
        plt.figtext(pagex, pagey-0.05, runs)
        plt.figtext(pagex, pagey-0.10, "Time limit per run: %s seconds" % d[1])
        plt.figtext(pagex, pagey-0.15, "Memory limit per run: %s MB" % d[2])
        pagey -= 0.22
    plt.show()
    pp.savefig(plt.gcf())
    pp.close()

def save_as_mysql(dbname, mysqldump):
    # See http://stackoverflow.com/questions/1067060/perl-to-python
    import re
    print("Saving as MySQL dump file...")

    conn = sqlite3.connect(dbname)
    mysqldump = open(mysqldump,'w')

    # make sure all tables are dropped in an order that keepd foreign keys valid
    c = conn.cursor()
    c.execute("SELECT name FROM sqlite_master WHERE type='table'")
    table_names = [ str(t[0]) for t in c.fetchall() ]
    c.close()
    last = ['experiments', 'planner_configs']
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
            'sqlite_sequence','CREATE UNIQUE INDEX', 'CREATE VIEW'):
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

def compute_views(dbname):
    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute('PRAGMA FOREIGN_KEYS = ON')

    # best configuration per problem, for each planner
    c.execute('SELECT DISTINCT planner_name FROM planner_configs')
    planners = [p[0] for p in c.fetchall() if not p[0] == None]
    c.execute('SELECT DISTINCT name FROM experiments')
    exps = [e[0] for e in c.fetchall() if not e[0] == None]
    for p in planners:
        # the table name for this planner
        tname = 'planner_' + p

        # check if simplification time is available
        has_simplification_time = False
        c.execute('SELECT * FROM `%s` LIMIT 1' % tname)
        if 'simplification_time' in [t[0] for t in c.description]:
            has_simplification_time = True

        for enm in exps:
            # select all runs, in all configurations, for a particular problem and a particular planner
            s0 = 'SELECT * FROM `%s` INNER JOIN experiments ON `%s`.experimentid = experiments.id WHERE experiments.name = "%s"' % (tname, tname, enm)
            # select the highest solve rate and shortest average runtime for each planner configuration
            if has_simplification_time:
                s1 = 'SELECT plannerid, AVG(solved) AS avg_slv, AVG(time + simplification_time) AS total_time FROM (%s) GROUP BY plannerid ORDER BY avg_slv DESC, total_time ASC LIMIT 1' % s0
            else:
                s1 = 'SELECT plannerid, AVG(solved) AS avg_slv, AVG(time) AS total_time FROM (%s) GROUP BY plannerid ORDER BY avg_slv DESC, total_time ASC LIMIT 1' % s0
            c.execute(s1)
            best = c.fetchone()
            if not best == None:
                if not best[0] == None:
                    bp = 'best_' + enm + '_' + p
                    print("Best plannner configuration for planner " + p + " on problem '" + enm + "' is " + str(best[0]))
                    c.execute('DROP VIEW IF EXISTS `%s`' % bp)
                    c.execute('CREATE VIEW IF NOT EXISTS `%s` AS SELECT * FROM (%s) WHERE plannerid = %s' % (bp, s0, best[0]))

        if has_simplification_time:
            c.execute('SELECT plannerid, AVG(solved) AS avg_slv, AVG(time + simplification_time) AS total_time FROM `%s` GROUP BY plannerid ORDER BY avg_slv DESC, total_time ASC LIMIT 1' % tname)
        else:
            c.execute('SELECT plannerid, AVG(solved) AS avg_slv, AVG(time) AS total_time FROM `%s` GROUP BY plannerid ORDER BY avg_slv DESC, total_time ASC LIMIT 1' % tname)
        best = c.fetchone()
        if not best == None:
            if not best[0] == None:
                bp = 'best_' + p
                print("Best overall plannner configuration for planner " + p + " on is " + str(best[0]))
                c.execute('DROP VIEW IF EXISTS `%s`' % bp)
                c.execute('CREATE VIEW IF NOT EXISTS `%s` AS SELECT * FROM `%s` WHERE plannerid = %s' % (bp, tname, best[0]))
    conn.commit()
    c.close()

if __name__ == "__main__":
    usage = """%prog [options] [<benchmark.log> ...]"""
    parser = OptionParser(usage)
    parser.add_option("-d", "--database", dest="dbname", default="benchmark.db",
        help="Filename of benchmark database [default: %default]")
    parser.add_option("-v", "--view", action="store_true", dest="view", default=False,
        help="Compute the views for best planner configurations")
    parser.add_option("-p", "--plot", dest="plot", default=None,
        help="Create a PDF of plots")
    parser.add_option("-m", "--mysql", dest="mysqldb", default=None,
        help="Save SQLite3 database as a MySQL dump file")
    (options, args) = parser.parse_args()

    if len(args)>0:
        read_benchmark_log(options.dbname, args)
        # If we update the database, we recompute the views as well
        options.view = True

    if options.view:
        compute_views(options.dbname)

    if options.plot:
        plot_statistics(options.dbname, options.plot)

    if options.mysqldb:
        save_as_mysql(options.dbname, options.mysqldb)

