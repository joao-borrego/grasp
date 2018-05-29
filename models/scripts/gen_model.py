#!/usr/bin/python3

# Command line args
import sys, getopt
# XML
import xml.etree.ElementTree as ET
# GUI
from tkinter import *
from tkinter import ttk
from tkinter import font
from tkinter.filedialog import asksaveasfile

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

class MainApp(Frame):
  """TODO: Main App class """

  def __init__(self, parent, data=None):
    """ """
    Frame.__init__(self, parent)
    self.parent = parent
    self.parent.title('URDF Model Generator')
    self.parent.minsize(width=300, height=200)
    self.parent.maxsize(width=parent.winfo_screenwidth(), 
      height=parent.winfo_screenheight())

    self.name = data['name']
    self.root = data['xml']
    self.links = data['links']
    self.all_joints = data['joints']

    self.setupWidgets()

  def setupWidgets(self):
    """TODO"""

    # Styling
    m_pad = {'padx': 20, 'pady': 10}
    font_h1 = "Helvetica 18 bold"
    font_h2 = "Helvetica 15"
    style_h1 = {'padx': 20, 'pady': 20}
    style_h2 = {'padx': 10, 'pady': 5}

    # Main window
    parent = self.parent
    frame_btn = Frame(parent)
    frame_tree = Frame(parent)

    self.createTree(frame_tree)

    # Variables
    self.v_name = StringVar()
    self.v_name.set(self.name)
    self.v_prefix = StringVar()
    self.v_prefix.set('virtual_')

    # Entry
    ent_name = Entry(parent, textvariable=self.v_name)
    ent_prefix = Entry(parent, textvariable=self.v_prefix)
    self.ent_gravity = Checkbutton(parent)
    self.btn_add_joint = Button(frame_btn, text="+")
    self.btn_rmv_joint = Button(frame_btn, text="-")
    self.btn_clr_joint = Button(frame_btn, text="Clear")
    self.btn_save = Button(parent, text="Save", command=self.onSave)

    """
    joint_var = StringVar(parent)
    joint_var.set(self.joints[0])
    joint_dropdown = ttk.Combobox(parent, textvariable=joint_var, values=self.joints)
    joint_dropdown.grid(row=2, column=1, **style_h2)
    """

    # Layout

    Label(parent, text="Model Name").grid(row=0, **style_h1)
    ent_name.grid(row=0, column=1, **style_h2)

    Label(parent, text="Virtual Joints").grid(row=1, **style_h1)
    text = 'Virtual joints for unconstrained floating behaviour'
    Label(parent, text=text).grid(row=1, column=1, columnspan=4, **style_h1)

    Label(parent, text="Joint name prefix").grid(row=2, **style_h2)
    ent_prefix.grid(row=2, column=1, **style_h2)
    Label(parent, text="Gravity").grid(row=2, column=2, **style_h2)
    self.ent_gravity.grid(row=2, column=3, **style_h2)

    Label(parent, text="Real Joints").grid(row=3, **style_h1)
    frame_btn.grid(row=3, column=3)
    self.btn_rmv_joint.pack(side="left")
    self.btn_add_joint.pack(side="left")
    self.btn_clr_joint.pack(side="right")

    frame_tree.grid(row=4, column=0, columnspan=4)

    self.btn_save.grid(row=5, column=3, **style_h2)

  def createTree(self, frame):
    """TODO"""
    self.header = ['name','mimic','parent','multiplier']
    self.tree = ttk.Treeview(frame, columns=self.header, show="headings")
    y_scrollbar = ttk.Scrollbar(frame, orient="vertical",
      command=self.tree.yview)
    x_scrollbar = ttk.Scrollbar(frame, orient="horizontal",
      command=self.tree.xview)
    self.tree.configure(yscrollcommand=y_scrollbar.set,
      xscrollcommand=x_scrollbar.set)

    x_scrollbar.pack(side='bottom', fill='x')
    self.tree.pack(side='left')
    y_scrollbar.pack(side='right', fill='y')

    # Allow treeview to be resized
    self.grid_columnconfigure(0, weight=1)
    self.grid_rowconfigure(0, weight=1)

    self.buildTree()

  def buildTree(self):
    """TODO"""
    for col in self.header:
      self.tree.heading(col, text=col.title())
      self.tree.column(col)

    for row in self.all_joints:
      self.tree.insert("","end", values=[row, "", "", ""])

  def onSave(self):
    """TODO"""
    f = asksaveasfile(mode='w', defaultextension=".urdf")

def main(argv):
  """TODO: Main Application"""

  in_file = '../shadowhand_lite.urdf'

  # Open input file
  tree = ET.parse(in_file)  
  root = tree.getroot()

  # DEBUG operations
  removePlugins(root, ['gazebo_ros_control'])
  removeElemAttrMatch(root, 'joint', 'name', ['rh_world_joint'])
  removeElemAttrMatch(root, 'link', 'name', ['world'])

  # Obtain model name
  name = root.attrib['name']
  # Obtain name of each link
  links = [elem.attrib['name'] for elem in root.findall('link')]
  # Obtain name of each joint
  joints = [elem.attrib['name'] for elem in root.findall('joint')]
  # Encapsulate data in structure
  data = {
    'name': name,
    'xml': root,
    'links': links,
    'joints': joints}

  # Launch GUI
  root = Tk()
  root.columnconfigure(0, weight=1)
  root.resizable(width=0, height=0)
  app = MainApp(root, data)
  root.mainloop()

if __name__ == "__main__" :
  main(sys.argv)
