#!/usr/bin/python3

# Command line args
import sys, getopt
# XML
import xml.etree.ElementTree as ET
# GUI
import tkinter as tk

def removeElemAttrMatch(root, element, attr, blacklist):
  """Removes elements matching given attribute values specified in a list

  Removes elements from an Element tree given its element type,
  the name of the attribute and a list of values to match

  Parameters
  ----------
  root : :obj:`Element`
  The root of the XML tree
  element : :obj:`str`
  The type of elements to be considered
  attr : :obj:`str`
  The name of the attribute to filter
  blacklist : :obj:`list` of :obj:`str`
  List of values to match for the given attribute

  """
  for elem in root.findall(element):
    if elem.attrib[attr] in blacklist:
      root.remove(elem)

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
  for elem in root.findall('gazebo'):
    removeElemAttrMatch(elem, 'plugin', 'name', blacklist)

class MainApp(tk.Frame):
  """TODO: Main App class """

  def __init__(self, parent, data=None):
    """ """
    tk.Frame.__init__(self, parent)
    self.parent = parent
    self.parent.title('URDF Model Generator')
    self.parent.minsize(width=300, height=200)
    self.parent.maxsize(width=parent.winfo_screenwidth(), 
      height=parent.winfo_screenheight())

    self.root = data['xml']
    self.links = data['links']
    self.joints = data['joints']

    self.setupWidgets()

  def setupWidgets(self):
    """TODO"""

    joint_var = tk.StringVar(self.parent)
    joint_var.set(self.joints[0])
    w = tk.OptionMenu(self.parent, joint_var, *self.joints)
    w.pack()

def main(argv):
  """TODO: Main Application"""

  in_file = '../shadowhand_lite.urdf' 
  out_file = 'out.urdf'

  # Open input file
  tree = ET.parse(in_file)  
  root = tree.getroot()

  # DEBUG operations
  removePlugins(root, ['gazebo_ros_control'])
  removeElemAttrMatch(root, 'joint', 'name', ['rh_world_joint'])
  removeElemAttrMatch(root, 'link', 'name', ['world'])

  # Obtain name of each link
  links = [elem.attrib['name'] for elem in root.findall('link')]
  # Obtain name of each joint
  joints = [elem.attrib['name'] for elem in root.findall('joint')]
  # Encapsulate data in structure
  data = {'xml': root, 'links': links, 'joints': joints}

  # Launch GUI
  root = tk.Tk()
  app = MainApp(root, data)
  root.mainloop()

if __name__ == "__main__" :
  main(sys.argv)
