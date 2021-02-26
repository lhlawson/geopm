#!/usr/bin/python3
#
#  Copyright (c) 2015, 2016, 2017, 2018, 2019, 2020, Intel Corporation
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in
#        the documentation and/or other materials provided with the
#        distribution.
#
#      * Neither the name of Intel Corporation nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

import sys
import numpy as np
import pandas as pd
import glob
import matplotlib.pyplot as plt

import code
import subprocess

max_cores=44 #TODO pull from machine.json
down_sample_rate = 20
sample_rate = 20
p1 = 2.1e9 #TODO pull from machine.json
p01 = 3.7e9 #TODO pull from machine.json
heatmap_min = [0.0, 1.0e9, 1.0e9]
heatmap_max = [1.0, p01, p01]
sanitize_data=False

#def parse_experiment(file_regex='', ignore_cores=[0,int(max_cores/2)]):
def parse_experiment(file_regex='', ignore_cores=[]):
    trace_list = sorted(glob.glob('*0.trace-*'))
    trace_list = [trace for trace in trace_list if file_regex in trace and '.png' not in trace]

    df_list = [];
    for idx,trace in enumerate(trace_list):
        file_append = '{}'.format(trace.split('.')[1])

        df_list.append(parse_trace(trace, ignore_cores))
        hists_per_node(df_list[idx], file_append)
        heatmaps_per_node(df_list[idx], file_append)
        linecharts_per_node(df_list[idx], file_append)

    df_concat = pd.concat(df_list, axis=1)[[col for col in df_list[0].columns if 'SCALABILITY-node' in col]]
    heatmaps_per_system(df_concat, df_list[0]['TIME'], trace_list, 'system_level')

    df_concat = pd.concat(df_list, axis=1)[[col for col in df_list[0].columns if 'FREQUENCY_ACTUAL-node' in col or 'SCALABILITY-node' in col]] # or 'UNCORE_PERF_STATUS' in col]]
    #using iloc to truncate a potentially larger df.  This isn't ideal, but works for now
    linecharts_per_system(df_concat.iloc[:len(df_list[0]['TIME'])], df_list[0]['TIME'], trace_list, 'system_level')

    #write to feather files

    code.interact(local=locals())

def linecharts_per_system(df, time, trace_list, file_append):
    node_count = len(trace_list)
    node_names = [trace.split('.')[1] for trace in trace_list]

    for idx,chart in enumerate(['SCALABILITY','FREQUENCY_ACTUAL']):
        fig1 = plt.figure(figsize=(40,10))
        ax = fig1.add_subplot(1, 1, 1)
        ax.set_xlabel("Time")
        ax.set_ylabel(chart)
        ax.set_title("{} with sample rate {} and down sample to {}".format(chart, sample_rate, down_sample_rate))

        try:
            #TODO: add labels
            ax.plot(time.to_list(), df['{}-node'.format(chart)])

        except Exception as e:
            print(e)
            print('something went wrong plotting linecharts per node')
            code.interact(local=locals())

        fig1.savefig('{}-linemap_{}.png'.format(chart, file_append), bbox_inches='tight')
        plt.close(fig1)



def linecharts_per_node(df, file_append):
    sys.stdout.write("\t\tCharting per node line chars with file append of {}\n".format(file_append))

    for idx,chart in enumerate(['SCALABILITY','FREQUENCY_ACTUAL']): #, 'MSR::UNCORE_PERF_STATUS:FREQ']):

        #Per Package
        fig1 = plt.figure(figsize=(40,20))
        for package in range(2):
            ax = fig1.add_subplot(2, 1, 1+package)
            ax.set_title("{} package {}".format(chart, package))
            ax.set_ylim(bottom=heatmap_min[idx],  top=heatmap_max[idx])
            x = df['TIME'].to_list()
            y = df['{}-package-{}'.format(chart, package)].to_list()
            ax.plot(x, y, label='{} - Package Average'.format(chart))

        fig1.savefig('{}_per_package-linechart_{}.png'.format(chart, file_append), bbox_inches='tight')
        plt.close(fig1)

def hists_per_node(df, file_append):
    df = (df[[x for x in df.columns if 'SCALABILITY-core' in x]])
    sc = []

    for cut in range(10):
        sc.append((((df > cut/10) & (df < (cut+1)/10)).sum()).sum())


    fig1 = plt.figure(figsize=(40,10))
    ax = fig1.add_subplot(1, 1, 1)
    ax.set_title("Scalability Histogram node ".format(file_append))
    ax.set_ylabel('% Samples')
    ax.set_xlabel('Scalability');
    ax.set_ylim(bottom=0.0,  top=1.0)
    ax.bar(x=[0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9], height=sc/sum(sc), align='edge', width=0.1)

    fig1.savefig('Scalability Histogram-_{}.png'.format(file_append), bbox_inches='tight')
    plt.close(fig1)


def heatmaps_per_system(df, time, trace_list, file_append):
    node_count = len(trace_list)
    node_names = [trace.split('.')[1] for trace in trace_list]
    sys.stdout.write("\t\tCharting system level heatmap\n")
    fig1 = plt.figure(figsize=(40,10))
    ax = fig1.add_subplot(1, 1, 1)
    ax.set_xlabel("Time")
    ax.set_ylabel("Node")

    try:
        y = list(range(node_count))
        ax.set_yticks(y)
        ax.set_yticklabels(node_names)

        #REDUCED NOISE
        num_ticks = 10
        xtick = np.linspace(time.min(), time.max(), num=num_ticks, endpoint=True)
        ax.set_xticklabels(xtick, rotation=45)
        ax.set_xticks(list(range(0, len(df), int(len(df)/num_ticks))))

        im = ax.pcolormesh(df.T, cmap='viridis', vmin=heatmap_min[0], vmax=heatmap_max[0])

    except Exception as e:
        print(e)
        print('something went wrong plotting heatmaps per node')
        code.interact(local=locals())

    fig1.colorbar(im, ax=ax).set_label("")

    fig1.savefig('scalability-heatmap_{}.png'.format(file_append), bbox_inches='tight')
    plt.close(fig1)


def heatmaps_per_node(df, file_append):
    sys.stdout.write("\t\tCharting heatmaps with file append of {}\n".format(file_append))

    for idx,chart in enumerate(['SCALABILITY', 'SCALABLE_FREQUENCY','FREQUENCY_ACTUAL']):
        fig1 = plt.figure(figsize=(40,20))
        fig1.tight_layout()
        #PER PACKAGE
        #fig1 = plt.figure(figsize=(40,10))
        ax = fig1.add_subplot(2, 1, 1)
        ax.set_xlabel("Time")
        ax.set_ylabel("Package")
        ax.set_title("{} with sample rate {} and down sample to {}".format(chart, sample_rate, down_sample_rate))

        try:
            y = list(range(max_cores))
            ax.set_yticks(y)

            #REDUCED NOISE
            num_ticks = 10
            xtick = np.linspace(df['TIME'].min(), df['TIME'].max(), num=num_ticks, endpoint=True)
            ax.set_xticklabels(xtick, rotation=45)
            ax.set_xticks(list(range(0, len(df), int(len(df)/num_ticks))))

            im = ax.pcolormesh([df[chart + '-package-0'],df[chart + '-package-1']], cmap='viridis', vmin=heatmap_min[idx], vmax=heatmap_max[idx])
        except Exception as e:
            print(e)
            print('something went wrong plotting heatmap per package')

        fig1.colorbar(im, ax=ax).set_label(chart)

        #PER CORE
        ax = fig1.add_subplot(2, 1, 2)
        ax.set_xlabel("Time")
        ax.set_ylabel("Core")

        try:
            y = list(range(max_cores))
            ax.set_yticks(y)

            x = df[[x for x in df.columns if chart+'-core' in x]]

            #REDUCED NOISE
            num_ticks = 10
            xtick = np.linspace(df['TIME'].min(), df['TIME'].max(), num=num_ticks, endpoint=True)
            ax.set_xticklabels(xtick, rotation=45)
            ax.set_xticks(list(range(0, len(df), int(len(df)/num_ticks))))

            im = ax.pcolormesh(x.T, cmap='viridis', vmin=heatmap_min[idx], vmax=heatmap_max[idx])

        except Exception as e:
            print(e)
            print('something went wrong plotting heatmap per core')

        fig1.colorbar(im, ax=ax).set_label(chart)

        fig1.savefig('{}-heatmap_{}.png'.format(chart, file_append), bbox_inches='tight')
        plt.close(fig1)


def parse_trace(trace, ignore_cores):

    sys.stdout.write("File being parsed: {}\n".format(trace))
    acnt_columns = [ f'MSR::APERF:ACNT-core-{core}' for core in range(max_cores)]
    pcnt_columns = [ f'MSR::PPERF:PCNT-core-{core}' for core in range(max_cores)]
    mcnt_columns = [ f'MSR::MPERF:MCNT-core-{core}' for core in range(max_cores)]

    try:
        df = pd.read_csv(
            trace, sep='|', comment='#',
            usecols=['TIME'] + acnt_columns + mcnt_columns + pcnt_columns
        )
    except Exception as e:
        sys.stdout.write("\tUsing reduced csv parse failed.\n")
        sys.stdout.write("\t\t{}.\n".format(e))
        df = pd.read_csv(trace, delimiter='|', skiprows=5)

    sys.stdout.write("\tFile read into dataframe\n")

    #TIME STRIDE
    index_skip = int(down_sample_rate/sample_rate) #TODO: sample rate can be determined
                                                   #      from the trace and not simply assumed
    df = (df.iloc[::index_skip, :]).reset_index(drop=True)

    #BUILD CORE LEVEL VALUES
    sys.stdout.write("\tBuilding core level values\n")
    for core in range(max_cores):
        if core not in ignore_cores: #ignore GEOPM and OS cores for now
            df["ACNT_DELTA-core-" + str(core)] = df["MSR::APERF:ACNT-core-"+str(core)].diff()
            df["MCNT_DELTA-core-" + str(core)] = df["MSR::MPERF:MCNT-core-"+str(core)].diff()
            df["FREQUENCY_ACTUAL-core-" + str(core)] = (df["ACNT_DELTA-core-"+str(core)]/df["MCNT_DELTA-core-"+str(core)])*(p1)

            #borrowed from Daniel, blame him
            try:
                df[f'REGION_HASH-core-{core}'] = df[f'REGION_HASH-core-{core}'].replace({'NAN': UNMARKED_REGION_HASH}).transform(lambda x: int(x, base=16))
            except:
                pass

            if(sanitize_data):
                df.loc[df['FREQUENCY_ACTUAL-core-' +str(core)] > p01, 'FREQUENCY_ACTUAL-core-' + str(core)] = p01
                df.loc[df['FREQUENCY_ACTUAL-core-' +str(core)] < 1.0e9, 'FREQUENCY_ACTUAL-core-' + str(core)] = 1.0e9

            df["PCNT_DELTA-core-" + str(core)] = df["MSR::PPERF:PCNT-core-"+str(core)].diff()
            if('SCALABILITY-core-'+str(core) not in df.columns):
                df["SCALABILITY-core-" + str(core)] = (df["PCNT_DELTA-core-"+str(core)]/df["ACNT_DELTA-core-"+str(core)])
                if(sanitize_data):
                    df.loc[df['SCALABILITY-core-' +str(core)] > 1, 'SCALABILITY-core-' +str(core)] = 1
                    df.loc[df['SCALABILITY-core-' +str(core)] < 0, 'SCALABILITY-core-' +str(core)] = 0
            if('SCALABLE_FREQUENCY-core-'+str(core) not in df.columns):
                df["SCALABLE_FREQUENCY-core-" + str(core)] = (df["PCNT_DELTA-core-"+str(core)]/df["MCNT_DELTA-core-"+str(core)])*(p1)


    #BUILD NODE LEVEL VALUES
    df["FREQUENCY_ACTUAL-node"] = df[[col for col in df.columns if "FREQUENCY_ACTUAL-core" in col]].mean(axis=1)
    df["SCALABILITY-node"] = df[[col for col in df.columns if "SCALABILITY-core" in col]].mean(axis=1)

    #BUILD PACKAGE LEVEL VALUES
    temp_df = df[[x for x in df.columns if 'SCALABILITY-core-' in x]]
    pkg = np.split(temp_df, [int(len(temp_df.columns)/2)], axis=1)
    df["SCALABILITY-package-0"] = pkg[0].mean(axis=1)
    df["SCALABILITY-package-1"] = pkg[1].mean(axis=1)

    temp_df = df[[x for x in df.columns if 'FREQUENCY_ACTUAL-core-' in x]]
    pkg = np.split(temp_df, [int(len(temp_df.columns)/2)], axis=1)
    df["FREQUENCY_ACTUAL-package-0"] = pkg[0].mean(axis=1)
    df["FREQUENCY_ACTUAL-package-1"] = pkg[1].mean(axis=1)

    temp_df = df[[x for x in df.columns if 'SCALABLE_FREQUENCY-core-' in x]]
    pkg = np.split(temp_df, [int(len(temp_df.columns)/2)], axis=1)
    df["SCALABLE_FREQUENCY-package-0"] = pkg[0].mean(axis=1)
    df["SCALABLE_FREQUENCY-package-1"] = pkg[1].mean(axis=1)

    return df

down_sample_rate=5; parse_experiment()
#code.interact(local=locals())
