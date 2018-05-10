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

# Usage
USAGE = 'options: -d <metadata file>\n' +          \
        '         -i <input mesh directory>\n' +   \
        '         -o <output model directory>\n' + \
        '         -t <template directory>\n'

def parseArgs(argv):
    '''
    Parses command-line options.
    '''

    # Parameters
    data_file = 'metadata.csv'
    in_dir = 'COLLADA'
    out_dir = 'output'
    template = 'template'
    
    usage = 'usage:   ' + argv[0] + ' [options]\n' + USAGE

    try:
        opts, args = getopt.getopt(argv[1:],
            "hd:i:o:t:",["data_file=","in_dir=","out_dir=","template="])
    except getopt.GetoptError:
        print (usage)
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print (usage)
            sys.exit()
        elif opt in ("-d", "--data_file"):
            data_file = arg
        elif opt in ("-i", "--in_dir"):
            in_dir = arg
        elif opt in ("-o", "--out_dir"):
            out_dir = arg
        elif opt in ("-t", "--template"):
            template = arg

    print ('Metadata file          ', data_file)
    print ('Input mesh directory   ', in_dir)
    print ('Output model directory ', out_dir)
    print ('Template path          ', template)
    
    return [data_file, in_dir, out_dir, template]

def search_and_replace(filename, search, replace):
    with fileinput.FileInput(filename, inplace=True) as file:
        for line in file:
            print(line.replace(search, replace), end='')

def remove_xml_blocks(filename, tag):
    tag_start = '<'+tag+'>'
    tag_end = '</'+tag+'>'
    in_block = False
    with fileinput.FileInput(filename, inplace=True) as file:
        for line in file:
            if tag_end in line: in_block = False
            if not in_block:
                print(line, end='')
            if tag_start in line: in_block = True


def main(argv):

    # Obtain command-line arguments
    [data_file, in_dir, out_dir, template] = parseArgs(argv)

    # Open metadata file
    in_data = csv.DictReader(open(data_file))

    # Matching tags
    match_tags = set(['bowl'])
    # Blacklist tags
    blacklist = set([''])

    print('\nGenerating Gazebo models:\n')

    # Process each row
    for row in in_data:

        uid = row['fullId'][4:]
        name = row['name'].replace(" ", "_")
        category = row['category']
        unit = row['unit']
        tags = row['tags'].split(",")

        # If object matches monitored tags
        if match_tags.intersection(tags) and not blacklist.intersection(tags):
            
            print(uid + '  ' + name)
            out_name = out_dir + '/' + uid + '/meshes/'
            os.makedirs(os.path.dirname(out_name), exist_ok=1)

            template_model = template + '/model.sdf'
            template_cfg = template + '/model.config'
            out_model = out_dir + '/' + uid + '/model.sdf'
            out_cfg = out_dir + '/' + uid + '/model.config'
            scale = str(unit) + ' ' + str(unit) + ' ' + str(unit)
            in_mesh = in_dir + '/' + uid + '.dae'
            out_mesh = out_dir + '/' + uid + '/meshes/' + uid + '.stl'
            in_textures = in_dir + '/' + uid

            try:
                textures = [f for f in os.listdir(in_textures) \
                    if os.path.isfile(os.path.join(in_textures, f))]
            except FileNotFoundError:
                textures = []

            if textures:
                out_name = out_dir + '/' + uid + '/materials/textures/'
                os.makedirs(os.path.dirname(out_name), exist_ok=1)
                for texture in textures:
                    copyfile(in_textures + '/' + texture, out_name + texture)

            copyfile(template_model, out_model)
            copyfile(template_cfg, out_cfg)
            search_and_replace(out_model, 'TEMPLATE', uid)
            search_and_replace(out_model, 'MESH_EXT', 'stl')
            search_and_replace(out_model, 'SCALE', scale)
            search_and_replace(out_cfg, 'TEMPLATE', uid)
            search_and_replace(out_cfg, 'DESCRIPTION', name)

            try:
                subprocess.call(['meshlabserver -i ' + in_mesh + ' -o ' + out_mesh], shell=1)
                #copyfile(in_mesh, out_mesh)
                # Remove image texture XML block
                # remove_xml_blocks(out_mesh, 'library_materials')
                # search_and_replace(out_mesh, uid, '../materials/textures')

            except:
                # Some models do not have poisson filtered meshes; discard them
                print(in_mesh + ' not found. Removing model.')
                rmtree(out_dir + '/' + uid)

if __name__ == "__main__":
    main(sys.argv)