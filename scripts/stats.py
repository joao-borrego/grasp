#!/usr/bin/python3

# Command line args
import sys
# CSV
import csv
# OS
import os
# YAML output
import yaml

# Default parameters

# Mesh root directory
DEF_CFG_FILE = 'cfg/stats.yml'
# List of required parameters in cfg file
REQ_PARAMS = ['ds_path','metrics_dir','robot']

class Grasp:
  def __init__(self, name, metric):
    self.name = name
    self.metric = metric

class Object:
  def __init__(self, name):
    self.name = name
    self.grasps = dict()
    self.num_error = -1
    self.num_fail = -1
    self.num_success = -1

  def computeStats(self):

    self.num_error = self.num_fail = self.num_success = 0
    for key, grasp in self.grasps.items():
      if grasp.metric == -1:
        self.num_error += 1
      elif grasp.metric == 0:
        self.num_fail += 1
      elif grasp.metric == 1:
        self.num_success += 1

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

def getGraspMetrics(obj_name, cfg):
  '''
  Gets grasp metrics from grasp.yml.
  '''
  grasp_dict = dict()
  robot = cfg['robot']
  grasp_path = os.path.join(cfg['metrics_dir'], obj_name + '.metrics.yml')

  try:
    with open(grasp_path, 'r') as stream:
      root = yaml.load(stream)
      grasp_root = root['object']['grasp_candidates'][robot]
      for key, value in grasp_root.items():
        metric = value['success']
        grasp_dict[key] = Grasp(key, metric)

  except Exception as e:
    print('Could not load configuration {}: {}'.format(cfg_file, e))
    exit(-1)

  return grasp_dict

def main(argv):
  ''' Main executable
  '''

  # Parse config yml
  cfg = parseConfig(DEF_CFG_FILE)
  # Get list of objects
  obj_list = getObjectList(cfg['ds_path'])
  
  for obj in obj_list:
    # Obtain grasp metrics for every object
    obj.grasps = getGraspMetrics(obj.name, cfg)
    # Count number
    obj.computeStats()
    print('{}: Err {}  Fail {}  Success {} - Total {}'.format(obj.name,
      obj.num_error, obj.num_fail, obj.num_success, len(obj.grasps)))

if __name__ == "__main__":
  main(sys.argv)
