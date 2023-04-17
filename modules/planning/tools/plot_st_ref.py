#!/usr/bin/python3.6
# encoding=utf-8
"""
python tools/plot_log.py -f /apollo/data/log/planning.INFO -t 11:50:34
"""

import argparse
from collections import defaultdict
import os
import re
import shutil
import sys
import time
import math
import datetime
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.widgets import Cursor
from matplotlib.gridspec import GridSpec
from  matplotlib import  ticker

def get_string_between(string, st, ed=''):
    """get string between keywords"""
    if string.find(st) < 0:
        return ''
    sub_string = string[string.find(st) + len(st):]
    if len(ed) == 0 or sub_string.find(ed) < 0:
        return sub_string.strip()
    return sub_string[:sub_string.find(ed)]


def get_planning_seq_num(line):
    """get planning seq num from line"""
    return get_string_between(line, 'start frame sequence id = [', ']')


def plot_frame(fig, ax, lines, line_st_num, line_ed_num):
    """plot ref frame"""
    print( 'plot line start num: ' + str(line_st_num + 1))
    print( 'plot line end num: ' + str(line_ed_num + 1))
    frame_seq = get_planning_seq_num(lines[line_st_num])
    print( 'frame seq num: ' + frame_seq)
    
    t = []
    s0 = []
    s1 = []
    for i in range(line_st_num, line_ed_num):
        line = lines[i]
        if "relative time is: " in line:
            cur_t = line.split("is: ")[1]
            cur_t.replace("\n", "")
            if len(t) > 2 and float(cur_t) < 0.05:
                break
            t.append(float(cur_t))
        if "obs position s = " in line:
            cur_s0 = line.split("s = ")[1]
            cur_s0.replace("\n", "")
            s0.append(float(cur_s0))
        if "adc position s = " in line:
            cur_s1 = line.split("s = ")[1]
            cur_s1.replace("\n", "")
            s1.append(float(cur_s1))

    # plot curve from point vectors
    plt.rcParams['figure.figsize'] = (9, 6)
    
    ax.plot(t, s0, label="obs")
    ax.plot(t, s1, label="ego")

    ax.legend()
    plt.title('st_ref')
    plt.xlabel('T/s')
    plt.ylabel('S/m')
    plt.grid(True)
    plt.show()

    return



def search_next(lines, line_search_num):
    """search forward, return frame start and end line number"""
    start_line_num = -1
    seq_id = '-1'
    for i in range(line_search_num, len(lines)):
        if 'Planning start frame sequence id' in lines[i]:
            seq_id = get_string_between(lines[i], 'sequence id = [', ']')
            start_line_num = i
            break
    if start_line_num < 0:
        return -1, -1, seq_id

    for i in range(start_line_num, len(lines)):
        if 'Planning end frame sequence id = [' + seq_id in lines[i]:
            return start_line_num, i, seq_id
    return start_line_num, -1, seq_id


def search_last(lines, line_search_num):
    end_line_num = -1
    seq_id = '-1'
    for i in range(line_search_num, 0, -1):
        if 'Planning end frame sequence id' in lines[i]:
            seq_id = get_string_between(lines[i], 'sequence id = [', ']')
            end_line_num = i
            break
    if end_line_num < 0:
        return -1, -1, seq_id

    for i in range(end_line_num, 0, -1):
        if 'Planning start frame sequence id = [' + seq_id in lines[i]:
            return i, end_line_num, seq_id
    return -1, end_line_num, seq_id

def search_time_line(lines, search_time):
    """search line with time"""
    for i in range(len(lines)):
        if search_time in lines[i]:
            return i + 1
    return 0


def search_seq_line(lines, search_seq):
    """search line with time"""
    for i in range(len(lines)):
        if 'start frame sequence id' in lines[i]:
            if 'start frame sequence id = [' + search_seq in lines[i]:
                return i + 1
    return 0


if __name__ == '__main__':
    global g_argv
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', action='store', dest='log_file_path', required=True, help='log file path')
    parser.add_argument('-t', action='store', dest='time', required=False, help='time begin to search')
    parser.add_argument('-ut', action='store', dest='unix_time', required=False, help='unix time begin to search')
    parser.add_argument('-s', action='store', dest='seq', required=False, help='sequence number to search')
    g_argv = parser.parse_args()
    print('Please wait for loading data...')

    # load log file
    print (g_argv)
    file_path = g_argv.log_file_path
    #file_path = parse_pb_log(file_path)
    search_time = g_argv.time
    search_seq = g_argv.seq
    unix_time = g_argv.unix_time
    input = open(file_path, 'r')
    lines = input.readlines()
    if search_time:
        line_search_num = search_time_line(lines, search_time)
    elif search_seq:
        line_search_num = search_seq_line(lines, search_seq)
    elif  unix_time:
        search_time = datetime.datetime.fromtimestamp(float(unix_time)).strftime('%H:%M:%S')
        print("from unixtime %s to data time %s"%(unix_time, search_time))
        line_search_num = search_time_line(lines, search_time)
    else:
        print( 'search time or sequence number or unix time is required, quit!')
        sys.exit(0)

    print(line_search_num)
    # check whether time is exist
    if line_search_num == 0:
        print( 'no such time, quit!')
        sys.exit(0)
    line_st_num, line_ed_num, seq_id = search_next(lines, line_search_num)
    # check whether found frame log is complete
    if line_st_num < 0 or line_ed_num < 0:
        print( '[ERROR] search reach last line, may reach last frame, quit!')
        sys.exit(0)

    fig, ax = plt.subplots(1, 1)
    plot_frame(fig, ax, lines, line_st_num, line_ed_num)
