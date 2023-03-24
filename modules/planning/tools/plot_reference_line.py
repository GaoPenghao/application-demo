import json
import sys
import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
import math

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3.record import RecordReader
from modules.common_msgs.planning_msgs import planning_pb2
# from modules.planning.proto import planning_pb2
  
class RefLineInfo():
    def __init__(self, x, y, s, theta, kappa, dkappa):
        self.x = x
        self.y = y
        self.s = s
        self.theta = theta
        self.kappa = kappa
        self.dkappa = dkappa
 
def trim_path_by_distance(adc_trajectory, s):
    path_coords = []
    path_points = adc_trajectory.trajectory_point
    for point in path_points:
        if point.path_point.s <= s:
            path_coords.append([point.path_point.x, point.path_point.y])
    return path_coords
 
def project(point, ref_line):
    start_point = ref_line[0]
    end_point = start_point
    for line_point in ref_line:
        if line_point.s - ref_line[0].s < 0.2:
            continue
        point_dir = [start_point.x - point.x, start_point.y - point.y]
        line_dir = [line_point.x - start_point.x, line_point.y - start_point.y]
        dot = point_dir[0] * line_dir[0] + point_dir[1] * line_dir[1]
        if dot > 0.0:
            start_point = line_point
            continue
        s = dot / math.sqrt(line_dir[0] * line_dir[0] + line_dir[1] * line_dir[1])
        return start_point.s - s
        
 
def align_reference_lines(ref_line1, ref_line2):
    if len(ref_line1) < 2 or len(ref_line2) < 2:
        return [0.0, 0.0]
    if ref_line1[-1].s < 0.5 or ref_line2[-1].s < 0.5:
        return [0.0, 0.0]
    
    s_ref_line1 = [ref_line1[0].x, ref_line1[0].y]
    cur_index = 1
    e_ref_line1 = [ref_line1[cur_index].x, ref_line1[cur_index].y]
    while ref_line1[cur_index].s < 0.5:
        cur_index = cur_index + 1
        e_ref_line1 = [ref_line1[cur_index].x, ref_line1[cur_index].y]
        
    start_point2 = ref_line2[0]
    line_dir = [e_ref_line1[0] - s_ref_line1[0], e_ref_line1[1] - s_ref_line1[1]]
    start_dir = [s_ref_line1[0] - start_point2.x, s_ref_line1[1] - start_point2.y]
    dot = line_dir[0] * start_dir[0] + line_dir[1] * start_dir[1]
    if dot > 0.0:
        return [0.0, project(ref_line1[0], ref_line2)]
    return [project(start_point2, ref_line1), 0.0]
    
    
def get_ref_line(record_path, ref_line_index=0):
    reader = RecordReader(record_path)
    current_ref_line_index = 0
    for msg in reader.read_messages():
        if msg.topic == "/apollo/planning":
            if current_ref_line_index != ref_line_index:
                current_ref_line_index = current_ref_line_index + 1
                continue
            adc_trajectory = planning_pb2.ADCTrajectory()
            adc_trajectory.ParseFromString(msg.message)
 
            for path in adc_trajectory.debug.planning_data.path:
                if path.name != 'planning_reference_line':
                    continue
                path_coords = trim_path_by_distance(adc_trajectory, 5.0)
 
                ref_line = []
                last_theta = path.path_point[0].theta
                for point in path.path_point:
                    if point.theta - last_theta > math.pi:
                        point.theta = point.theta - 2.0 * math.pi
                    elif last_theta - point.theta > math.pi:
                        point.theta = point.theta + 2.0 * math.pi
                    ref_line.append(RefLineInfo(point.x, point.y, point.s, point.theta, point.kappa, point.dkappa))
                return ref_line
            
def plot_ref_line(start_s, ref_line, use_dot):
    x = []
    y = []
    s = []
    theta = []
    kappa = []
    dkappa = []
    scale_factor = 10.0
    for point in ref_line:
        if point.s < start_s:
            continue
        x.append(point.x)
        y.append(point.y)
        s.append(point.s)
        theta.append(point.theta)
        kappa.append(point.kappa * scale_factor)
        dkappa.append(point.dkappa * scale_factor)
    if use_dot:
        plt.plot(s, theta, 'b--', alpha=0.5, label='theta')
        plt.plot(s, kappa, 'r--', alpha=0.5, label='kappa')
        plt.plot(s, dkappa, 'g--', alpha=0.5, label='dkappa')
    else:
        plt.plot(s, theta, 'b', alpha=0.5, label='theta')
        plt.plot(s, kappa, 'r', alpha=0.5, label='kappa')
        plt.plot(s, dkappa, 'g', alpha=0.5, label='dkappa')
                    
def plot_ref_path(record_file1, record_file2):
    ref_line1 = get_ref_line(record_file1)
    ref_line2 = get_ref_line(record_file2)
    [s1, s2] = align_reference_lines(ref_line1, ref_line2)
    plot_ref_line(s1, ref_line1, True)
    plot_ref_line(s2, ref_line2, False)
    
if __name__ == '__main__':
    cyber.init()

    record_file1 = '/apollo_workspace/20230319133518.record.00000'
    record_file2 = '/apollo_workspace/20230319133518.record.00000'
 
    plot_ref_path(record_file1, record_file2)
      
    plt.legend()
    plt.show()

    cyber.shutdown()