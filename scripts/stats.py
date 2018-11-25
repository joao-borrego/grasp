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

import matplotlib.pyplot as plt

# Default parameters

# Mesh root directory
DEF_CFG_FILE = 'cfg/stats.yml'
# List of required parameters in cfg file
REQ_PARAMS = [\
  'ds_path',             # Dataset file path
  'baseline_dir',        # Baseline metric directory
  'metrics_dir',         # Custom metric (without physics DR) directory
  'metrics_dr_dir',      # Custom metric (with physics DR) directory
  'robot',               # Robot name
  'bl_delta_min',        # Baseline metric threshold
  'bl_delta_max',
  'bl_delta_step',
  'trial_delta'          # Trial metric threshold
]

# Terminal colours
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def info(msg):
  print(bcolors.OKBLUE + "[INFO] " + msg + bcolors.ENDC)

def warn(msg):
  print(bcolors.WARNING + "[WARN] " + msg + bcolors.ENDC)

def fail(msg):
  print(bcolors.FAIL + "[ERRO] " + msg + bcolors.ENDC)

# Figure settings
colors = [
    [50,136,189],    # rgb(50,136,189)
    [253,174,97],    # rgb(253,174,97)
    [213,62,79],     # rgb(213,62,79)
    [171,221,164],   # rgb(171,221,164)
    [94,72,162],     # rgb(94,79,162)
    [94,79,162],     # #5e4fa2
    [50,136,189],    # #3288bd
    [102,194,165],   # #66c2a5
    [171,221,164],   # #abdda4
    [230,245,152],   # #e6f598
    [254,224,139],   # #fee08b
    [253,174,97],    # #fdae61
    [244,109,67],    # #f46d43
    [213,62,79],     # #d53e4f
    [158,1,66],      # #9e0142
    [164,6,6],       # #a40606
    []
]

for idx in range(len(colors)):
    colors[idx] = [(tmp/255.0) for tmp in colors[idx]]

linestyles = ['-', '--', ':','-.','--']
linewidth=[4, 4, 4, 4, 4]
fontsize_=32
plt.rc('font', family='serif')
plt.rc('xtick', labelsize='x-small')
plt.rc('ytick', labelsize='x-small')
plt.rc('text', usetex=True)

# Helper classes

class Grasp:
  def __init__(self, name):
    self.name = name
    self.metrics = dict()

class Object:
  def __init__(self, name):
    self.name = name
    self.grasps = dict()

    self.num_error = dict()
    self.num_valid = dict()
    self.valid_ratio = dict()

    self.t_pos = dict()
    self.t_neg = dict()
    self.f_pos = dict()
    self.f_neg = dict()
    self.t_pos['trial'] = dict(); self.t_pos['trial_dr'] = dict(); 
    self.t_neg['trial'] = dict(); self.t_neg['trial_dr'] = dict();
    self.f_pos['trial'] = dict(); self.f_pos['trial_dr'] = dict();
    self.f_neg['trial'] = dict(); self.f_neg['trial_dr'] = dict(); 

    self.accuracy = dict()
    self.precision = dict()
    self.recall = dict()
    self.f1 = dict()
    self.accuracy['trial'] = dict(); self.accuracy['trial_dr'] = dict();
    self.precision['trial'] = dict(); self.precision['trial_dr'] = dict();
    self.recall['trial'] = dict(); self.recall['trial_dr'] = dict();
    self.f1['trial'] = dict(); self.f1['trial_dr'] = dict();

  def computeValid(self, dr):

    self.num_error[dr] = 0
    self.num_valid[dr] = 0
    
    for key, grasp in self.grasps.items():
      if grasp.metrics[dr] == -1:
        self.num_error[dr] += 1
      else:
        self.num_valid[dr] += 1

    self.valid_ratio[dr] = self.num_valid[dr] / (self.num_valid[dr] + self.num_error[dr])

  def computeAccuracy(self, dr, delta_idx):
    self.accuracy[dr][delta_idx] = \
      (self.t_pos[dr][delta_idx] + self.t_neg[dr][delta_idx]) / \
      (self.t_pos[dr][delta_idx] + self.t_neg[dr][delta_idx] + \
        self.f_pos[dr][delta_idx] + self.f_neg[dr][delta_idx])

  def computePrecision(self, dr, delta_idx):
    if self.t_pos[dr][delta_idx] + self.f_pos[dr][delta_idx] == 0:
      self.precision[dr][delta_idx] = math.inf
    else:
      self.precision[dr][delta_idx] = (self.t_pos[dr][delta_idx]) / \
        (self.t_pos[dr][delta_idx] + self.f_pos[dr][delta_idx]) 

  def computeRecall(self, dr, delta_idx):
    if self.t_pos[dr][delta_idx] + self.f_neg[dr][delta_idx] == 0:
      self.recall[dr][delta_idx] = math.inf
    else:
      self.recall[dr][delta_idx] = (self.t_pos[dr][delta_idx]) / \
        (self.t_pos[dr][delta_idx] + self.f_neg[dr][delta_idx])

  def computeF1(self, dr, delta_idx):
    if self.precision[dr][delta_idx] + self.recall[dr][delta_idx] == 0:
      self.f1[dr][delta_idx] = math.inf
    else:
      self.f1[dr][delta_idx] = \
        (self.precision[dr][delta_idx] * self.recall[dr][delta_idx]) / \
        (self.precision[dr][delta_idx] + self.recall[dr][delta_idx]) * 2

  def compareBaseline(self, deltas, trial_delta):

    for dr in ['trial', 'trial_dr']:

      for delta in deltas:
        delta_i = "{:.6f}".format(delta)
        self.t_pos[dr][delta_i] = 0
        self.t_neg[dr][delta_i] = 0
        self.f_pos[dr][delta_i] = 0
        self.f_neg[dr][delta_i] = 0

      for key, grasp in self.grasps.items():

        # Discard invalid grasps
        if grasp.metrics[dr] != -1:

          for delta in deltas:

            delta_i = "{:.6f}".format(delta)
            trial = grasp.metrics[dr] >= trial_delta
            baseline = grasp.metrics['baseline'] >= delta

            if (trial == True and baseline == True):
              self.t_pos[dr][delta_i] = self.t_pos[dr][delta_i] + 1
            elif (trial == False and baseline == False):
              self.t_neg[dr][delta_i] = self.t_neg[dr][delta_i] + 1
            elif (trial == True and baseline == False):
              self.f_pos[dr][delta_i] = self.f_pos[dr][delta_i] + 1
            elif (trial == False and baseline == True):
              self.f_neg[dr][delta_i] = self.f_neg[dr][delta_i] + 1

          for delta in deltas:
            delta_i = "{:.6f}".format(delta)
            self.computeAccuracy(dr, delta_i)
            #self.computePrecision(dr, delta_i)
            #self.computeRecall(dr, delta_i)
            #self.computeF1(dr, delta_i)

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


def plotAccuracy(obj, baseline_deltas, cfg):

  accuracy = [obj.accuracy['trial']["{:.6f}".format(delta)] for delta in baseline_deltas]
  accuracy_dr = [obj.accuracy['trial_dr']["{:.6f}".format(delta)] for delta in baseline_deltas]

  fig, ax = plt.subplots(figsize=(5, 5))
  plt.xlabel('Threshold $\\delta$', fontsize=fontsize_)
  plt.ylabel('Accuracy', fontsize=fontsize_)
  plt.xticks(size=fontsize_ - 2)
  plt.yticks(size=fontsize_ - 2)
  plt.xlim(min(baseline_deltas), max(baseline_deltas))
  plt.ylim(0.01, 1.01)
  ax.plot(baseline_deltas, accuracy, color=colors[0], linewidth=linewidth[0])
  ax.plot(baseline_deltas, accuracy_dr, color=colors[1], linewidth=linewidth[1])
  leg = plt.legend(['w/o DR', 'w/ DR'],
   fancybox=False, ncol=1,
   loc='upper right',fontsize=fontsize_ - 10)
  plt.tight_layout()
  if (cfg['save_figures']):
    plt.savefig(os.path.join(cfg['figure_dir'], 'accuracy_' + obj.name + '.pdf'), dpi=300)
  #plt.show()
  plt.close()

def printStatistics(obj, baseline_deltas):

  print("{};{};{};{};{};{};{};{};{}".format(\
    "delta",
    "tp","tp_dr",
    "tn","tn_dr",
    "fp","fp_dr",
    "fn","fn_dr"))
  for delta in baseline_deltas:
    delta_i = "{:.6f}".format(delta)
    a = obj.t_pos['trial'][delta_i] +  obj.t_neg['trial'][delta_i] + \
       obj.f_pos['trial'][delta_i] +  obj.f_neg['trial'][delta_i]
    b = obj.t_pos['trial_dr'][delta_i] +  obj.t_neg['trial_dr'][delta_i] + \
       obj.f_pos['trial_dr'][delta_i] +  obj.f_neg['trial_dr'][delta_i]
    print("{};{:.2f};{:.2f};{:.2f};{:.2f};{:.2f};{:.2f};{:.2f};{:.2f}".format( \
      delta_i,
      obj.t_pos['trial'][delta_i] / a, obj.t_pos['trial_dr'][delta_i] / b,
      obj.t_neg['trial'][delta_i] / a, obj.t_neg['trial_dr'][delta_i] / b,
      obj.f_pos['trial'][delta_i] / a, obj.f_pos['trial_dr'][delta_i] / b,
      obj.f_neg['trial'][delta_i] / a, obj.f_neg['trial_dr'][delta_i] / b))

def main(argv):
  ''' Main executable
  '''

  # Parse config yml
  cfg = parseConfig(DEF_CFG_FILE)
  # Get list of objects
  obj_list = getObjectList(cfg['ds_path'])
  
  obj_list.sort(key=lambda obj: obj.name)

  # Average valid ratio
  num_valid_objs = 0
  avg_valid_ratio = dict()
  avg_valid_ratio['trial'] = 0
  avg_valid_ratio['trial_dr'] = 0

  info("Configuration loaded.")
  info("Output grasphs will be stored in {}.".format(cfg['figure_dir']))

  baseline_deltas = np.arange(cfg['bl_delta_min'],cfg['bl_delta_max'],cfg['bl_delta_step'])
  num_objs = len(obj_list)

  for idx, obj in enumerate(obj_list):
    # Obtain baseline grasp metrics for every object
    grasp_path = os.path.join(cfg['baseline_dir'], obj.name + '.baseline.yml')
    getGraspMetrics(obj.grasps, grasp_path, 'baseline', cfg)
    # Obtain DR trial grasp metrics for every object
    grasp_path = os.path.join(cfg['metrics_dir'], obj.name + '.metrics.yml')
    getGraspMetrics(obj.grasps, grasp_path, 'trial', cfg)
    # Obtain baseline grasp metrics for every object
    grasp_path = os.path.join(cfg['metrics_dr_dir'], obj.name + '.metrics.yml')
    getGraspMetrics(obj.grasps, grasp_path, 'trial_dr', cfg)

    info("Processing {} ({}/{})".format(obj.name, idx + 1, num_objs))

    # Count valid grasps per object
    obj.computeValid('trial')
    obj.computeValid('trial_dr')

    if (obj.num_valid['trial'] > 0 and obj.num_valid['trial_dr'] > 0):

      # Compute number of true positives, etc. for different delta thresholds
      obj.compareBaseline(baseline_deltas, cfg['trial_delta'])
      plotAccuracy(obj, baseline_deltas, cfg)
      info("    Valid grasp ratio {}".format(obj.valid_ratio['trial_dr']))

      num_valid_objs += 1
      avg_valid_ratio['trial'] += obj.valid_ratio['trial']
      avg_valid_ratio['trial_dr'] += obj.valid_ratio['trial_dr']

    else:

      warn("Object has no valid grasps. Skipping...")

    # Aux comparison of true positives, negatives, in CSV format
    #printStatistics(obj, baseline_deltas)
    
    # for delta in baseline_deltas:
    #   delta_i = str(round(delta,4))

    #   print('{} - A {} ; P {} ; R {} ; F1 {} - d {}'.format(
    #     obj.name, 
    #     str(round(obj.accuracy[delta_i], 4)),
    #     str(round(obj.precision[delta_i], 4)),
    #     str(round(obj.recall[delta_i], 4)),
    #     str(round(obj.f1[delta_i], 4)),
    #     delta_i
    #   ))

  avg_valid_ratio['trial'] /= num_valid_objs
  avg_valid_ratio['trial_dr'] /= num_valid_objs
  info("Average valid ratio for trials w/out DR {}".format(avg_valid_ratio['trial']))
  info("Average valid ratio for trials w/ DR {}".format(avg_valid_ratio['trial_dr']))

if __name__ == "__main__":
  main(sys.argv)
