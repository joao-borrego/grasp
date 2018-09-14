#!/usr/bin/python3

# Command line args
import sys, getopt
# CSV
import csv
# OS
import os
# Subprocess
import subprocess
# Copyfile
from shutil import copyfile, rmtree
# For file edit
import fileinput
# Regex
import re
# Triangular meshes
import numpy as np
import trimesh
# YAML output
import yaml

# Default parameters

# Mesh root directory
DEF_IN_DIR = 'path/to/dir'
# Template model root directory
DEF_TEMPLATE = 'datasets/template'
# Output model directory
DEF_OUT_DIR = 'data/datasets/out'
# Output dataset yml file
DEF_OUT_YML = 'data/datasets/ds.yml'
# Output mesh format
DEF_MESH_EXT = 'stl'

# Dataset file list yml file
#   If None the whole input dir is searched for meshes
DEF_DS_LIST = None
# Preproc MeshLab script
#   If None no script is applied to input meshes
DEF_SCRIPT = None

# Usage
USAGE = 'options: -i <input mesh directory>\n' +   \
    '         -l <input model list yml>\n' +   \
    '         -t <template directory>\n' +     \
    '         -s <meshlab script> \n' +        \
    '         -o <output model directory>\n' + \
    '         -c <output config yml>\n'

def parseArgs(argv):
  '''
  Parses command-line options.
  '''

  # Parameters
  in_dir = DEF_IN_DIR
  ds_list = DEF_DS_LIST
  out_dir = DEF_OUT_DIR
  template = DEF_TEMPLATE
  script = DEF_SCRIPT
  out_yml = DEF_OUT_YML
  
  usage = 'usage:   ' + argv[0] + ' [options]\n' + USAGE

  try:
    opts, args = getopt.getopt(argv[1:],
      "hi:l:o:t:s:c:",
      ["in_dir=","out_dir=","template=","script=","out_yml="])
  except getopt.GetoptError:
    print (usage)
    sys.exit(2)

  for opt, arg in opts:
    if opt == '-h':
      print (usage)
      sys.exit()
    elif opt in ("-i", "--in_dir"):
      in_dir = arg
    elif opt in ("-l", "--ds_list"):
      ds_list = arg
    elif opt in ("-o", "--out_dir"):
      out_dir = arg
    elif opt in ("-t", "--template"):
      template = arg
    elif opt in ("-s", "--script"):
      script = arg
    elif opt in ("-c", "--config"):
      out_yml = arg

  print ('\nInput mesh directory   ', in_dir)
  print ('Input model list file  ', ds_list)
  print ('Output model directory ', out_dir)
  print ('Output config file     ', out_yml)
  print ('Template path          ', template)
  print ('Meshlab script         ', script)
  
  return [in_dir, ds_list, out_dir, out_yml, template, script]

def search_and_replace(filename, search, replace):
  ''' Opens file by name and performs search and replace
  '''
  with fileinput.FileInput(filename, inplace=True) as file:
    for line in file:
      print(line.replace(search, replace), end='')

def main(argv):
  ''' Main executable
  '''

  # Obtain command-line arguments
  [in_dir, ds_list, out_dir, out_yml, template, script] = parseArgs(argv)

  # Read object list from file
  if ds_list is not None:
    try:
      with open(ds_list, 'r') as stream:
        list_yml = yaml.load(stream)
        meshes = [m for m in list_yml['models']]
    except Exception as e:
      print('Could not load model list in {}: {}'.format(ds_list, e))
      exit(-1)
  # Find every mesh in input directory
  else:
    meshes = [f for f in os.listdir(in_dir) \
      if os.path.isfile(os.path.join(in_dir, f))]

  # Initialise output yaml data
  data_yml = dict()

  progress = 0.0
  progress_inc = 100.0 / len(meshes)
  print('\nGenerating Gazebo models:\n')

  for idx, mesh in enumerate(meshes):

    # TODO - Dataset preproc

    # Get name without file extension
    obj_name = os.path.splitext(mesh)[0]

    print("\x1b[2K{:10.4f} % - {}".format(progress, obj_name),end="\r")

    # Generate file paths
    out_mesh_dir_l = [out_dir, obj_name, 'meshes']
    out_model_l = [out_dir, obj_name, 'model.sdf']
    out_cfg_l = [out_dir, obj_name, 'model.config']    
    out_mesh_l = [out_dir, obj_name, 'meshes', obj_name + '.' + DEF_MESH_EXT]

    out_mesh_dir = os.path.join(*out_mesh_dir_l)
    os.makedirs(out_mesh_dir, exist_ok=1)
    
    template_model = os.path.join(template, 'model.sdf')
    template_cfg = os.path.join(template, 'model.config')
    in_mesh = os.path.join(in_dir, mesh)

    out_model = os.path.join(*out_model_l)
    out_cfg = os.path.join(*out_cfg_l)
    out_mesh = os.path.join(*out_mesh_l)

    # Process model mesh

    # Open null stream to surpress shell command output
    FNULL = open(os.devnull, 'w')

    if script is None:
      meshlab_cmd = 'meshlabserver -i {} -o {}'.format(in_mesh, out_mesh)
    else:
      meshlab_cmd = 'meshlabserver -i {} -o {} -s {}'.format(in_mesh, out_mesh, script)

    try:
      subprocess.call([meshlab_cmd], shell=1, stdout=FNULL, stderr=subprocess.STDOUT)
    except Exception as e:
      print('Could not process {}: {}'.format(in_mesh, e))
      obj_dir = os.path.join(out_dir, obj_name)
      rmtree(obj_dir)

    # Model description files
    copyfile(template_model, out_model)
    copyfile(template_cfg, out_cfg)

    search_and_replace(out_model, 'TEMPLATE', obj_name)
    search_and_replace(out_model, 'MESH_EXT', DEF_MESH_EXT)
    search_and_replace(out_model, 'SCALE', '1 1 1')
    search_and_replace(out_cfg, 'TEMPLATE', obj_name)
    search_and_replace(out_cfg, 'DESCRIPTION', obj_name)

    # Dataset configuration file
    data_yml[obj_name] = dict()
    data_yml[obj_name]['name'] = obj_name
    data_yml[obj_name]['path'] = 'model://' + obj_name
    data_yml[obj_name]['mesh'] = out_mesh 

    # Open mesh and compute physical properties
    mesh = trimesh.load(out_mesh)
    properties = trimesh.triangles.mass_properties(mesh.triangles)
    inertia = properties['inertia']
    mass = properties['mass']
    # TODO - Use estimated physical properties
    #data_yml[name]['mass'] = float(mass)
    #data_yml[name]['inertia'] = str(inertia.tolist())

    progress = progress + progress_inc

  # Write output yaml
  with open(out_yml, 'w') as outfile:
    yaml.dump(data_yml, outfile, default_flow_style=False)

  print("\x1b[2K{:10.4f}".format(100.0) + ' %\n\n' + \
   "Done generating Gazebo models!\n")

if __name__ == "__main__":
  main(sys.argv)
