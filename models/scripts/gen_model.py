#!/usr/bin/python3

# Command line args
import sys, getopt
# XML
import xml.etree.ElementTree as ET

def removePlugins(root, blacklist):
  """Removes incompatible plugins from XML tree

  Removes any plugin instantiation inside the gazebo tags
  matching any of the plugin names in a plugin blacklist

  Parameters
  ----------
  root : :obj:`Element`
    The root of the XML tree
  blacklist : :obj:`list` of :obj:`str`
    List of names of incompatible plugins to be removed

  """
  for elem in root:
    for subelem in elem.findall('plugin'):
      if subelem.attrib['name'] in blacklist:
        elem.remove(subelem)


def main(argv):
  """Main Application"""

  in_file = 'shadowhand_lite.urdf' 
  out_file = 'out.urdf'

  tree = ET.parse(in_file)  
  root = tree.getroot()

  removePlugins(root, ['gazebo_ros_control'])

  tree.write(out_file)

if __name__ == "__main__" :
    main(sys.argv)
