#!/usr/bin/python3

# Command line args
import sys
# CSV
import csv
# OS
import os
# YAML output
import yaml
# Math
import math
# Numpy
import numpy as np

# Default parameters

# Mesh root directory
DEF_CFG_FILE = 'cfg/stats.yml'
# List of required parameters in cfg file
REQ_PARAMS = [\
  'ds_path',             # Dataset file path
  'baseline_dir',        # Baseline metric directory
  'metrics_dir',         # Custom metric directory
  'robot',               # Robot name
  'bl_delta_min',        # Baseline metric threshold
  'bl_delta_max',
  'bl_delta_step',
  'trial_delta'          # Trial metric threshold
]

class Grasp:
  def __init__(self, name):
    self.name = name
    self.metrics = dict()

class Object:
  def __init__(self, name):
    self.name = name
    self.grasps = dict()

    self.num_error = -1
    self.num_fail = -1
    self.num_success = -1

    # (key: threshold, value: num true positives)
    self.t_pos = dict()
    # (key: threshold, value: num true negatives)
    self.t_neg = dict()
    # (key: threshold, value: num false positives)
    self.f_pos = dict()
    # (key: threshold, value: num false negatives)
    self.f_neg = dict()

    self.accuracy = dict()
    self.precision = dict()
    self.recall = dict()
    self.f1 = dict()

  def computeStats(self, delta):

    self.num_error = self.num_fail = self.num_success = 0
    for key, grasp in self.grasps.items():
      if grasp.metrics['trial'] == -1:
        self.num_error += 1
      elif grasp.metrics['trial'] < delta:
        self.num_fail += 1
      elif grasp.metrics['trial'] >= delta:
        self.num_success += 1

  def computeAccuracy(self, delta_idx):
    self.accuracy[delta_idx] = (self.t_pos[delta_idx] + self.t_neg[delta_idx]) / \
      (self.t_pos[delta_idx] + self.t_neg[delta_idx] + \
        self.f_pos[delta_idx] + self.f_neg[delta_idx])

  def computePrecision(self, delta_idx):
    if self.t_pos[delta_idx] + self.f_pos[delta_idx] == 0:
      self.precision[delta_idx] = math.inf
    else:
      self.precision[delta_idx] = (self.t_pos[delta_idx]) / \
        (self.t_pos[delta_idx] + self.f_pos[delta_idx]) 

  def computeRecall(self, delta_idx):
    if self.t_pos[delta_idx] + self.f_neg[delta_idx] == 0:
      self.recall[delta_idx] = math.inf
    else:
      self.recall[delta_idx] = (self.t_pos[delta_idx]) / \
        (self.t_pos[delta_idx] + self.f_neg[delta_idx])

  def computeF1(self, delta_idx):
    self.f1[delta_idx] = (self.precision[delta_idx] * self.recall[delta_idx]) / \
      (self.precision[delta_idx] + self.recall[delta_idx]) * 2

  def compareBaseline(self, deltas, trial_delta):

    for delta in deltas:
      delta_i = round(delta,3)
      self.t_pos[delta_i] = 0
      self.t_neg[delta_i] = 0
      self.f_pos[delta_i] = 0
      self.f_neg[delta_i] = 0

    for key, grasp in self.grasps.items():

      # Discard invalid grasps
      if grasp.metrics['trial'] == -1:
        continue

      for delta in deltas:

        trial = grasp.metrics['trial'] >= trial_delta
        baseline = grasp.metrics['baseline'] >= delta
        delta_i = round(delta,3)

        if (trial == True and baseline == True):
          self.t_pos[delta_i] = self.t_pos[delta_i] + 1
        if (trial == False and baseline == False):
          self.t_neg[delta_i] = self.t_neg[delta_i] + 1
        if (trial == True and baseline == False):
          self.f_pos[delta_i] = self.f_pos[delta_i] + 1
        if (trial == False and baseline == True):
          self.f_neg[delta_i] = self.f_neg[delta_i] + 1

    for delta in deltas:
      delta_i = round(delta,3)
      self.computeAccuracy(delta_i)
      self.computePrecision(delta_i)
      self.computeRecall(delta_i)
      self.computeF1(delta_i)

def parseConfig(cfg_file):
  '''
  Parses configuration yml.
  '''
  cfg = []
  try:
    with open(cfg_file, 'r') as stream:
      cfg = yaml.load(stream)
      if not all(param in cfg for param in REQ_PARAMS):
        print('Could not load configuration {}: Missing parameters!'.format(cfg_file))
        exit(-1)
  except Exception as e:
    print('Could not load configuration {}: {}'.format(cfg_file, e))
    exit(-1)

  return cfg

def getObjectList(ds_path):
  '''
  Gets object name list from dataset yml.
  '''
  obj_list = []
  try:
    with open(ds_path, 'r') as stream:
      ds_obj = yaml.load(stream)
      obj_name_list = list(ds_obj.keys())
  except Exception as e:
    print('Could not load configuration {}: {}'.format(cfg_file, e))
    exit(-1)

  obj_list = [Object(name) for name in obj_name_list]
  return obj_list

def getGraspMetrics(grasps, grasp_path, metric_name, cfg):
  '''
  Gets grasp metrics from grasp.yml.
  '''
  robot = cfg['robot']

  try:
    with open(grasp_path, 'r') as stream:
      root = yaml.load(stream)
      grasp_root = root['object']['grasp_candidates'][robot]
      for key, value in grasp_root.items():
        metric = value['success']
        if key not in grasps:
          grasps[key] = Grasp(key)
        grasps[key].metrics[metric_name] = metric

  except Exception as e:
    print('Could not load configuration {}: {}'.format(cfg_file, e))
    exit(-1)

def main(argv):
  ''' Main executable
  '''

  # Parse config yml
  cfg = parseConfig(DEF_CFG_FILE)
  # Get list of objects
  obj_list = getObjectList(cfg['ds_path'])
  
  obj_list.sort(key=lambda obj: obj.name)

  # sum_success = sum_fail = sum_error = 0
  # num_obj = 0

  # max_success = -1
  # max_name = ''

  baseline_deltas = np.arange(
    round(cfg['bl_delta_min'],  3),
    round(cfg['bl_delta_max'],  3), 
    round(cfg['bl_delta_step'], 3)) 

  for obj in obj_list:
    # Obtain DR trial grasp metrics for every object
    grasp_path = os.path.join(cfg['metrics_dir'], obj.name + '.metrics.yml')
    getGraspMetrics(obj.grasps, grasp_path, 'trial', cfg)
    # Obtain baseline grasp metrics for every object
    grasp_path = os.path.join(cfg['baseline_dir'], obj.name + '.baseline.yml')
    getGraspMetrics(obj.grasps, grasp_path, 'baseline', cfg)

    # Compute number of true positives, etc. for different delta thresholds
    obj.compareBaseline(baseline_deltas, cfg['trial_delta'])

    for delta in baseline_deltas:
      delta_i = round(delta,3)
      print('{} - A {} ; P {} ; R {} ; F1 {} - d {}'.format(
        obj.name, 
        round(obj.accuracy[delta_i], 4),
        round(obj.precision[delta_i], 4),
        round(obj.recall[delta_i], 4),
        round(obj.f1[delta_i], 4),
        delta_i
      ))

    # # Count number
    # obj.computeStats(cfg['trial_delta'])

    # if obj.num_success > max_success:
    #   max_success = obj.num_success
    #   max_name = obj.name

    # # Remove objects with only invalid grasps
    # if obj.num_error != len(obj.grasps):
    #   num_obj += 1 

    #   sum_success += obj.num_success / len(obj.grasps)
    #   sum_fail += obj.num_fail / len(obj.grasps)
    #   sum_error += obj.num_error / len(obj.grasps)

  # avg_success = sum_success / num_obj
  # avg_fail = sum_fail / num_obj
  # avg_error = sum_error / num_obj

  # print("{} & {} & {} \\".format(avg_success,avg_fail,avg_error))
  # print("{} {} ".format(max_name, max_success))

if __name__ == "__main__":
  main(sys.argv)
