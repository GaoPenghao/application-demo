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
    """plot frame"""
    print( 'plot line start num: ' + str(line_st_num + 1))
    print( 'plot line end num: ' + str(line_ed_num + 1))
    frame_seq = get_planning_seq_num(lines[line_st_num])
    print( 'frame seq num: ' + frame_seq)

    # plot curve from point vectors
    # ax_title = 'seq: ' + frame_seq
    # fig.suptitle(ax_title)
    plotter = dict()
    for i in range(line_st_num, line_ed_num):
        if "plot_" in lines[i]:
            data = lines[i].split("plot_")[1]
            data = data.split(":")
            tag = data[0]
            if plotter.get(tag) is None:
                plotter[tag] = ([], [])
            data[1].replace("\n", "")
            pt =data[1].split(",")
            if not len(pt) == 2:
                print(data[1], ":not a point")
                continue
            plotter[tag][0].append(float(pt[0]))
            plotter[tag][1].append(float(pt[1]))
       
    for tag in plotter.keys():
        if 'self' in tag or 'veh' in tag:
            continue
        if len(plotter[tag][0]) > 0:
            print(tag,plotter[tag])
            ax.plot(plotter[tag][0],
                plotter[tag][1], label=tag)
    
    # plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
    ax.legend()
    plt.rcParams['figure.figsize'] = (30, 5)
    plt.title('sl_path')
    plt.xlabel('S/m')
    plt.ylabel('L/m')
    plt.axis('equal')
    plt.grid(True)
    plt.show()

    # plt.draw()


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


class MouseEventManager(object):
    x, y = 0.0, 0.0
    xoffset, yoffset = -20, 20
    text_template = 'x: %0.2f\ny: %0.2f'
    annotation = False

    def on_click(self, event):
        # if mouse button is not right, return
        # 1: left, 2: middle, 3: right
        if event.button is not 3:
            return
        self.x, self.y = event.xdata, event.ydata
        if self.x is not None:
            print( 'mouse click x: %.2f, y: %.2f' % (event.xdata, event.ydata))
            if self.annotation:
                self.annotation.set_visible(False)
            label_text = self.text_template % (self.x, self.y)
            self.annotation = event.inaxes.annotate(label_text,
                xy=(self.x, self.y), xytext=(self.xoffset, self.yoffset),
                textcoords='offset points', ha='right', va='bottom',
                bbox=dict(boxstyle='round,pad=0.5', fc='lightcyan', alpha=0.5),
                arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0')
                )
            self.annotation.set_visible(True)
            self.annotation.figure.canvas.draw()


class Index(object):
    """button callback function"""
    def __init__(self,fig, ax, line_st_num, line_ed_num, lines):
        self.ax = ax
        self.fig = fig
        self.line_st_num = line_st_num
        self.line_ed_num = line_ed_num
        self.lines = lines
        self.reset_mouse_event()

    def reset_mouse_event(self):
        self.mouse_manager = MouseEventManager()
        fig.canvas.mpl_connect('button_release_event', self.mouse_manager.on_click)

    def next(self, step):
        """next button callback function"""
        for i in range(step):
            line_st_num, line_ed_num, seq_id =\
                    search_next(self.lines, self.line_ed_num + 1)
            # check whether found frame log is complete
            if line_st_num < 0 or line_ed_num < 0:
                print( '[ERROR] search reach last line, may reach last frame')
                return
            self.line_st_num = line_st_num
            self.line_ed_num = line_ed_num
        plot_frame(fig, self.ax, self.lines, self.line_st_num, self.line_ed_num)
        self.reset_mouse_event()

    def next1(self, event):
        self.next(1)

    def next10(self, event):
        self.next(10)

    def prev(self, step):
        """prev button callback function"""
        for i in range(step):
            line_st_num, line_ed_num, seq_id =\
                    search_last(self.lines, self.line_st_num - 1)
            # check whether found frame log is complete
            if line_st_num < 0 or line_ed_num < 0:
                print('[ERROR] search reach first line, may reach first frame')
                return
            self.line_st_num = line_st_num
            self.line_ed_num = line_ed_num
        plot_frame(fig, self.ax, self.lines, self.line_st_num, self.line_ed_num)
        self.reset_mouse_event()

    def prev1(self, event):
        self.prev(1)

    def prev10(self, event):
        self.prev(10)

    def exit(self, event):
        sys.exit(0)

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
        search_time = datetime.utcfromtimestamp(float(unix_time)).strftime('%H:%M:%S')
        print("from unixtime %s to data time %s"%(unix_time, search_time))
    else:
        print( 'search time or sequence number or unix time is required, quit!')
        sys.exit(0)

    print( line_search_num)
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
  
    # callback = Index(fig, ax, line_st_num, line_ed_num, lines)
    # prev1frame =  plt.axes([0.2, 0.01, 0.1, 0.05])
    # prev10frame = plt.axes([0.3, 0.01, 0.1, 0.05])
    # next1frame =  plt.axes([0.4, 0.01, 0.1, 0.05])
    # next10frame = plt.axes([0.5, 0.01, 0.1, 0.05])
    # exitframe =   plt.axes([0.6, 0.01, 0.1, 0.05])
    # bprev1 = Button(prev1frame, '-1')
    # bprev1.on_clicked(callback.prev1)
    # bprev10 = Button(prev10frame, '-10')
    # bprev10.on_clicked(callback.prev10)
    # bnext1 = Button(next1frame, '+1')
    # bnext1.on_clicked(callback.next1)
    # bnext10 = Button(next10frame, '+10')
    # bnext10.on_clicked(callback.next10)
    # bexit = Button(exitframe, 'exit')
    # bexit.on_clicked(callback.exit)
    # plt.gca().xaxis.set_major_formatter(ticker.FormatStrFormatter('%.6f'))
    # plt.gca().yaxis.set_major_formatter(ticker.FormatStrFormatter('%.6f'))
    # plt.show()
    # plt.ion()
