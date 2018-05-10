#!/usr/bin/python3

# Command line args
import sys, getopt
# OS
import os
# Copyfile
from shutil import copyfile, rmtree
# For file edit
import fileinput

USAGE = 'options: -i <input dataset directory>\n' +                     \
        '         -o <output model directory>\n' +                      \
        '         -t <path to template> [optional, default=./template]\n'

def parseArgs(argv):
    '''
    Parses command-line options.
    '''

    # Parameters
    input_dir = ''
    output_dir = ''
    template = './template'

    usage = 'usage:   ' + argv[0] + ' [options]\n' + USAGE

    try:
        opts, args = getopt.getopt(argv[1:],
            "hi:o:t:",["input_dir=","output_dir=","template="])
    except getopt.GetoptError:
        print (usage)
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print (usage)
            sys.exit()
        elif opt in ("-i", "--input_dir"):
            input_dir = arg
        elif opt in ("-o", "--output_dir"):
            output_dir = arg
        elif opt in ("-t", "--template"):
            template = arg

    if not input_dir or not output_dir:
        print (usage)
        sys.exit(2)

    print ('Input directory     ', input_dir)
    print ('Output directory    ', output_dir)
    print ('Template path       ', template)
    
    return [input_dir, output_dir, template]

def get_immediate_subdirectories(a_dir):
    return [name for name in os.listdir(a_dir)
        if os.path.isdir(os.path.join(a_dir, name))]

def search_and_replace(filename, search, replace):
    with fileinput.FileInput(filename, inplace=True) as file:
        for line in file:
            print(line.replace(search, replace), end='')

def main(argv):
    '''
    Converts ycb dataset to Gazebo models based on a template model
    '''
    
    # Obtain command-line arguments
    [input_dir, output_dir, template] = parseArgs(argv)

    object_names = get_immediate_subdirectories(input_dir)

    for name in object_names:
        os.makedirs(os.path.dirname(output_dir+'/'+name+'/meshes/'), exist_ok=True)
        copyfile(template+'/model.sdf', output_dir+'/'+name+'/model.sdf')
        copyfile(template+'/model.config', output_dir+'/'+name+'/model.config')
        search_and_replace(output_dir+'/'+name+'/model.sdf', 'TEMPLATE', name)
        search_and_replace(output_dir+'/'+name+'/model.sdf', 'MESH_EXT', 'stl')
        search_and_replace(output_dir+'/'+name+'/model.sdf', 'SCALE', '1 1 1')
        search_and_replace(output_dir+'/'+name+'/model.config', 'TEMPLATE', name)
        try:
            copyfile(input_dir+'/'+name+'/poisson/nontextured.stl', output_dir+'/'+name+'/meshes/'+name+'.stl')
        except:
            # Some models do not have poisson filtered meshes; discard them
            print(input_dir+'/'+name+'/poisson/nontextured.stl not found. Removing model.')
            rmtree(output_dir+'/'+name)

if __name__ == "__main__":
    main(sys.argv)
