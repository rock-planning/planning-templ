#!/usr/bin/env python3
# This script is the updated version -
# after identifying that the solution analyser did not work properly
import matplotlib.patches as mpatches
import matplotlib.ticker as mticker
import matplotlib.pyplot as plt
import numpy as np
import os
import matplotlib as mpl
import argparse
import sys
import time
import datetime as dt
import re
import matplotlib.colors as colors
import math

import plotting_helper as tikz

parser = argparse.ArgumentParser(description="Generate experiment pictures")
parser.add_argument("-i","--interactive",help="use the script non-interactively",
        action="store_true")
parser.add_argument("-l","--label",help="additional label for the output file")
parser.add_argument("-d","--directories",help="list of solution directories containing a solution_analysis.log")
args = parser.parse_args()


def getDataCollection(logfiles):
    collection = []
    for f in logfiles:
        dataPsi, successData, noSuccessData = getData(f)
        collection.append({ "data": dataPsi, "filename": f})
    return collection

# Extra the data from the logfile
# and return a hash per identified system with the logdata columns:
# relative time into mission, content-size, localized_pointclouds count,
# request count, spatial constraints
def getData(logfile,
        referenceStartTime = None, referenceEndTime = None):
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, logfile)

    logdata = np.loadtxt(filename, ndmin = 2)
    #print(logdata)
    success_cols = logdata[ logdata[:,idxOf('efficacy')] == 1.0 ]
    nosuccess_cols = logdata[ logdata[:,idxOf('efficacy')] != 1.0 ]
    return logdata, success_cols, nosuccess_cols

def idxOf(name):
    array = ['session-id',
              'alpha','beta','sigma',
              'efficacy','efficiency','safety',
              'timehorizon','travel-distance','reconfiguration-cost',
              'total-agent-count','mobile-agent-count'
              #'overall-runtime','solution-runtime','solution-runtime-mean','solution-runtime-stdev',
              #'solution-found','solution-stopped',
              #'propagate','fail','node','depth',
              #'restart','nogood','flaws','cost'
            ]
    return array.index(name)

def getGlobalMax(dataCollection, indexname):
    current_max = 0
    for c in dataCollection:
        current_max = getLocalMax(c["data"], indexname, min = current_max )
    return current_max

def getLocalMax(data, indexname, min = 0):
    col = data[:,idxOf(indexname)]
    value = max(col)
    if value > min:
        return value
    else:
        return min

def createScatterplotMobileAgents(axis, data, title = "",
        xlabel = "", ylabel = "",
        **kwargs):
    legendPatches = []
    axis.yaxis.grid(True)
    axis.set_ylabel(ylabel)
    axis.set_xlabel(xlabel)
    axis.set_xlim(0.2,1.1)
    axis.set_title(title)

    y = data[:,idxOf("efficiency")],
    x = data[:,idxOf("efficacy")],
    z = data[:,idxOf("safety")],
    agentcount = [ data[:,idxOf("mobile-agent-count")] ]
    cax = axis.scatter(x,y,c = np.array(agentcount), s = 50, marker = 'o',
            cmap="RdYlGn_r",
            alpha=0.8,
            linewidths=0.2,
            edgecolors='grey',
            **kwargs
            )
    return cax

def createScatterplotTotalAgents(axis, data, title = "",
        xlabel = "", ylabel = "",
        **kwargs):
    legendPatches = []
    axis.yaxis.grid(True)
    axis.set_ylabel(ylabel)
    axis.set_xlabel(xlabel)
    axis.set_xlim(0.2,1.1)
    axis.set_title(title)

    y = data[:,idxOf("efficiency")],
    x = data[:,idxOf("efficacy")],
    z = data[:,idxOf("safety")],
    agentcount = [ data[:,idxOf("total-agent-count")] ]
    cax = axis.scatter(x,y,c = np.array(agentcount), s = 50, marker = 'o',
            cmap="RdYlGn_r",
            alpha=0.8,
            linewidths=0.2,
            edgecolors='grey',
            **kwargs
            )

    return cax

def createScatterplotSafety(axis, data, best = None, title = "", xlabel = "", ylabel = "",
        vmin = 0.6, vmax = 0.9):
    legendPatches = []
    axis.yaxis.grid(True)
    axis.set_ylabel(ylabel)
    axis.set_xlabel(xlabel)
    axis.set_xlim(0.1,1.1)
    axis.set_title(title)

    y = data[:,idxOf("efficiency")],
    x = data[:,idxOf("efficacy")],
    z = data[:,idxOf("safety")],
    cax = axis.scatter(x,y,c = z, s = 50, marker = 'o',
            cmap="RdYlGn",
            alpha=0.8,
            linewidths=0.2,
            edgecolors='grey',
            vmin = vmin,
            vmax = vmax)
    if best != None:
        maxPos, maxPerformance, maxEfficiency = best
        print("Best found: {}".format(best))
        axis.scatter(maxPos[1]+0.03,maxPos[0]*maxEfficiency, c = 'black', s =30, marker = '*', vmin = vmin, vmax = vmax)
    return cax

def createParetoLine(axis, data, best, title = "", xlabel = "", ylabel = "",
    xobjective = "efficiency", yobjective = "safety",
    min_efficiency_in_kwh = 0,
    max_efficiency_in_kwh = 6000):
    i = 1.0
    labels = []
    #for i in 0.2,0.4,0.6,0.8,1.0:
    axis.set_title(title)
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)
    axis.yaxis.grid(True)
    axis.xaxis.grid(True)
    axis.set_ylim(0.6,0.9)
    axis.set_xlim(min_efficiency_in_kwh-250,max_efficiency_in_kwh + 250)
    tmpdata = data[ data[:, idxOf('efficacy')] == i]
    xo = np.array(tmpdata[:,idxOf( xobjective )])
    yo = np.array(tmpdata[:,idxOf( yobjective )])
    labels.append("{}".format(i))
    cax = axis.plot(xo,yo, 'bo' )
    return cax


def createBoxPlot(axis, data, best, title = "", xlabel = "", ylabel = "",
        objective = "efficiency", ymin = None, ymax = None, final = False):
    legendPatches = []
    axis.yaxis.grid(True)
    axis.set_ylabel(ylabel)
    axis.set_xlabel(xlabel)
    axis.set_title(title)
    if ymin != None and ymax != None:
        axis.set_ylim(ymin,ymax)

    maxPos,maxPerformance, maxEfficiency = best

    x = []
    y = []
    supportx = []
    supporty = []
    z = []
    labels = []
    for i in 0.2,0.4,0.6,0.8,1.0:
        tmpdata = data[ data[:, idxOf('efficacy')] == i]
        e = np.array(tmpdata[:,idxOf( objective )])
        labels.append("{}".format(i))
        y.append(e)
        supporty.append(len(e))
        z.append(np.std(e))
    cax = axis.boxplot(y)
    #axis.set_xticklabels(labels)
    axisright = axis.twinx()
    axisright.plot([1.0,2.0,3.0,4.0,5.0],supporty, color='g',alpha=0.7)
    axisright.tick_params('y',colors='g')
    axisright.set_ylim(0,200)
    axisright.set_ylabel("Number of solutions", color='g')
    if final:
        axisright.get_yaxis().set_visible(True)
    else:
        axisright.get_yaxis().set_visible(False)

    return cax

def maxEfficiency(data, fulfillment = 0.0):
    """
        extract the maximum efficiecy for the given data - which is an array
        of data sets
    """
    maxE = 0
    for i in range(0, len(data)):
        efficacy_mask = data[i][:,idxOf("efficacy")] >= fulfillment
        filtered_data = data[i][efficacy_mask, :]

        e = filtered_data[:,idxOf("efficiency")]
        if np.max(e) > maxE:
            maxE = np.max(e)
    return maxE

def minEfficiency(data, fulfillment = 0.0):
    """
        extract the maximum efficiecy for the given data - which is an array
        of data sets
    """
    minE = sys.maxsize
    for i in range(0, len(data)):
        efficacy_mask = data[i][:,idxOf("efficacy")] >= fulfillment
        filtered_data = data[i][efficacy_mask, :]

        e = filtered_data[:,idxOf("efficiency")]
        if np.min(e) < minE:
            minE = np.min(e)
    return minE

def identifyBest(data, maxEfficiency):
    y = data[:,idxOf("efficiency")],
    x = data[:,idxOf("efficacy")],
    z = data[:,idxOf("safety")],

    ymax = maxEfficiency
    max_performance = 0
    max_pos = []
    # Iterate over all sessions
    for i in range(0,len(y[0])):
        # -1.0, 100, 10
        performance = -y[0][i]/ymax + 100*x[0][i] + 10*z[0][i]
        if performance > max_performance:
            max_performance = performance
            max_pos= [ y[0][i]/ymax,x[0][i],z[0][i],i]

    #mean = [ numpy.mean(y[), numpy.mean(x), numpy.mean(z) ]
    #stdev = [ numpy.stdev(y), numpy.stdev(x), numpy.stdev(z) ]
    #cax = axis.errorplot(base, mean, stdev)
    return max_pos,max_performance, maxEfficiency

def compareFloat(dataCollection, colname = "safety",
        title = "Comparison of safety",
        colorbar_title = "Safety"):

    global_max_value = getGlobalMax(dataCollection, colname)
    print(f"Global max {colname}: {global_max_value}")
    f, ax = plt.subplots(plotrows,plotcols, sharex = True, sharey = True, figsize=tikz.Tikz.figsize_by_ratio(0.5))

    axes = []
    if len(dataCollection) == 1:
        axes.append(ax)
    else:
        axes = ax

    f.suptitle(title)
    plt.setp(axes, xticks = [0.2,0.4,0.6,0.8,1.0])
    scatter = None
    m_ylabel = "Efficiency\n(in kWh)"
    for i in range(numberOfSubplots):
        ylabel = ""
        if i % plotcols == 0:
            ylabel = m_ylabel
        scatter = createScatterplotSafety(axes[i], dataCollection[i]["data"],
                xlabel = "", ylabel = ylabel,
                vmin = 0, vmax = 1.0)
    plt.subplots_adjust(right = 0.8)

    #ticks = range(11)
    cbar = plt.colorbar(scatter) #, ticks=ticks )
    cbar.ax.set_title(colorbar_title)
    #ytick_labels = [ str(x/10.0) for x in ticks ]
    #print(ytick_labels)
    #cbar.ax.set_yticklabels(ytick_labels)

    outfilename = os.path.join(dirname,f"templ-psi-comparison-{colname}.pdf")
    tikz.Tikz.plot(outfilename, { 'interactive': args.interactive, 'type': 'pdf',
        'tight_layout': False, 'align_xlabels': False, 'crop': True})

def compareInt(dataCollection, colname = "mobile-agent-count",
        title = "Comparison of mobile agent count",
        colorbar_title = "Mobile Agents"):
    global_max_value = getGlobalMax(dataCollection, colname) + 1
    f, ax = plt.subplots(plotrows,plotcols, sharex = True, sharey = True, figsize=tikz.Tikz.figsize_by_ratio(0.5))

    axes = []
    if len(dataCollection) == 1:
        axes.append(ax)
    else:
        axes = ax

    f.suptitle(title)
    plt.setp(axes, xticks = [0.2,0.4,0.6,0.8,1.0])
    scatter = None
    m_ylabel = "Efficiency\n(in kWh)"
    for i in range(numberOfSubplots):
        ylabel = ""
        if i % plotcols == 0:
            ylabel = m_ylabel
        scatter = createScatterplotMobileAgents(axes[i], dataCollection[i]["data"],
                xlabel = "", ylabel = ylabel,
                vmin = 0, vmax = global_max_value)
    plt.subplots_adjust(right = 0.8)

    ticks = range(0, int(global_max_value), 1)
    cbar = plt.colorbar(scatter, ticks=ticks )
    cbar.ax.set_title(colorbar_title)
    cbar.ax.set_yticklabels([ str(x) for x in ticks])

    outfilename = os.path.join(dirname,f"templ-psi-comparison-{colname}.pdf")
    tikz.Tikz.plot(outfilename, { 'interactive': args.interactive, 'type': 'pdf',
        'tight_layout': False, 'align_xlabels': False, 'crop': True})


if args.label:
    label = args.label

solution_dirs = ["."]
if args.directories:
    solution_dirs = args.directories.split(",")
else:
    raise Exception("No log dir to (solution_analysis.log) provided")

solution_logs = []
for sdir in solution_dirs:
    solution_log = os.path.join(sdir, "solution_analysis.log")
    if not os.path.exists(solution_log):
        raise ValueError(f"Solution {solution_path} does not exist")
    solution_logs.append(solution_log)


dirname = os.path.join( os.path.dirname(__file__),"output")
if not os.path.isdir(dirname):
    os.mkdir(dirname)

dataCollection = getDataCollection(solution_logs)
print(solution_logs)

numberOfSubplots = len(dataCollection)
plotrows = math.ceil(numberOfSubplots/3.0)
plotcols = min(3,numberOfSubplots)

## compare to safety
compareFloat(dataCollection,
        colname = "safety",
        title = "Comparison by safety",
        colorbar_title = "Safety")

# compare to total agent count
compareInt(dataCollection,
        colname = "total-agent-count",
        title = "Comparison by total agent count",
        colorbar_title = "Total Agents")

# compare to number of mobile agents
compareInt(dataCollection,
        colname = "mobile-agent-count",
        title = "Comparison by mobile agent count",
        colorbar_title = "Mobile Agents")
