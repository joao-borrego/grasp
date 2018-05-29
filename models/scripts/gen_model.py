#!/usr/bin/python3

# Command line args
import sys, getopt
# XML
import xml.etree.ElementTree as ET
# GUI
from tkinter import *
from tkinter import ttk
from tkinter import font
from tkinter.filedialog import asksaveasfilename
from tkinter import messagebox

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
    self.all_joints = list(data['joints'])
    self.added_joints = list(data['joints'])
    self.removed_joints = []
    # List of existing joints
    self.joints = [None] * len(self.all_joints)
    for [idx, joint] in enumerate(self.all_joints) :
      self.joints[idx] = [joint, "False", "-", "-"]

    # Add joint dialog toplevel window
    self.toplevel = None

    self.setupWidgets()

  def setupWidgets(self):
    """TODO"""

    # Styling
    style_h1 = {'padx': 15, 'pady': 15, 'sticky' : "w"}
    style_h2 = {'padx': 30, 'pady': 5, 'sticky' : "w"}
    text_h1 = {'font': "Helvetica 18 bold"}
    text_h2 = {'font': "Helvetica 15 bold"}
    text_p = {'font': "Helvetica 15"}

    # Main window
    parent = self.parent
    frame_btn = Frame(parent)
    frame_tree = Frame(parent)

    self.createTree(frame_tree)

    # Variables
    self.var_name = StringVar(parent)
    self.var_name.set(self.name)
    self.var_base = StringVar(parent)
    self.var_base.set(self.links[0])
    self.var_prefix = StringVar(parent)
    self.var_prefix.set('virtual')
    self.var_gravity = IntVar(parent)
    self.var_gravity.set(0)

    # Entry
    ent_name = Entry(parent, textvariable=self.var_name)
    base_dropdown = ttk.Combobox(parent, textvariable=self.var_base,
      values=self.links)
    ent_prefix = Entry(parent, textvariable=self.var_prefix)
    self.ent_gravity = Checkbutton(parent, variable=self.var_gravity)
    self.btn_add_joint = Button(frame_btn, text="+",
      command=self.onAddJoint, **text_p)
    self.btn_rmv_joint = Button(frame_btn, text="-",
      command=self.onRemoveJoint, **text_p)
    self.btn_clr_joint = Button(frame_btn, text="Clear",
      command=self.onClear, **text_p)
    self.btn_save = Button(parent, text="Save", command=self.onSave, **text_p)

    # Layout

    Label(parent, text="Model Name", **text_h1).grid(row=0, **style_h1)
    ent_name.grid(row=0, column=1, **style_h2)
    Label(parent, text="Base Link", **text_h1).grid(row=0, column=2, **style_h1)
    base_dropdown.grid(row=0, column=3, **style_h2)

    Label(parent, text="Virtual Joints", **text_h1).grid(row=1, **style_h1)
    text = 'Virtual joints for unconstrained floating behaviour'
    Label(parent, text=text, **text_h2).grid(row=1, column=1, columnspan=4, **style_h1)

    Label(parent, text="Joint name prefix", **text_h2).grid(row=2, **style_h2)
    ent_prefix.grid(row=2, column=1, **style_h2)
    Label(parent, text="Enable gravity", **text_h2).grid(row=2, column=2, **style_h2)
    self.ent_gravity.grid(row=2, column=3, **style_h2)

    Label(parent, text="Real Joints", **text_h1).grid(row=3, **style_h1)
    frame_btn.grid(row=3, column=3, sticky="e", padx=10)
    self.btn_add_joint.pack(side="left")
    self.btn_rmv_joint.pack(side="left")
    self.btn_clr_joint.pack(side="right")

    frame_tree.grid(row=4, column=0, columnspan=4, padx=10, pady=5)

    self.btn_save.grid(row=5, column=3, sticky="e", padx=10, pady=5)

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
    
    self.tree.delete(*self.tree.get_children())
    for col in self.header:
      self.tree.heading(col, text=col.title())
      self.tree.column(col)
    for row in self.joints:
      self.tree.insert("","end", values=row)

  def onSave(self):
    """TODO"""
    name = self.var_name.get()
    base_link = self.var_base.get()
    prefix = self.var_prefix.get()
    gravity = self.var_gravity.get()

    out_file = asksaveasfilename(defaultextension=".urdf")

    template = './template.urdf'
    tree = ET.parse(template)  
    root = tree.getroot()

    virtual_names = ["_px_", "_py_", "_pz_", "_rr_", "_ry_", "_rz_"]
    virtual_joints = [prefix + s + "joint" for s in virtual_names]

    # Virtual Joints
    for elem in root.iter('joint'):
      elem.attrib['name'] = prefix + elem.attrib['name']
      for subelem in elem.iter('parent'):
        if subelem.attrib['link'] != "world":
          subelem.attrib['link'] = prefix + subelem.attrib['link']
      for subelem in elem.iter('child'):
        if subelem.attrib['link'] != "base_link":
          subelem.attrib['link'] = prefix + subelem.attrib['link']
        else:
          subelem.attrib['link'] = base_link

    # Plugin parameters
    tree_gazebo = root.find('gazebo')
    tree_plugin = tree_gazebo.find('plugin')
    tree_model = tree_plugin.find('model')
    tree_model.text = name
    tree_virtual = tree_plugin.find('virtualJoints')
    tree_virtual.text = " ".join(str(x) for x in virtual_joints)
    tree_gravity = tree_plugin.find('gravity')
    tree_gravity.text = str(gravity)

    # Actuated and Mimic Joints
    for actuated in [x for x in self.joints if x[1] == 'False']:
      new_actuated = ET.SubElement(tree_plugin, "actuatedJoint")
      new_actuated.set('name', actuated[0])
      for mimic in [x for x in self.joints if x[2] == actuated[0]]:
        new_mimic = ET.SubElement(new_actuated, "mimicJoint")
        new_mimic.set('name', mimic[0])
        new_mimic.set('multiplier', str(mimic[3]))

    # Input file tree
    tree.write(out_file)

  def onAddJoint(self):
    """TODO"""
    if self.toplevel is None:

      if self.removed_joints:

        m_pad = {'padx': 20, 'pady': 10}

        self.toplevel = Toplevel()
        toplevel = self.toplevel
        toplevel.title("Add Joint")
        toplevel.protocol("WM_DELETE_WINDOW", self.onCloseToplevel)
        toplevel.resizable(width=0, height=0)

        self.var_joint = StringVar(toplevel)
        self.var_joint.set(self.removed_joints[0])
        self.var_mimic = IntVar(toplevel)
        self.var_mimic.set(0)
        self.var_parent = StringVar(toplevel)
        self.var_parent.set("-")
        self.var_mult = DoubleVar(toplevel)
        self.var_mult.set(0.0)
        
        joint_dropdown = ttk.Combobox(toplevel, textvariable=self.var_joint,
          values=self.removed_joints)
        ent_mimic = Checkbutton(toplevel, text="Is mimic joint", 
          variable=self.var_mimic)
        parent_dropdown = ttk.Combobox(toplevel, textvariable=self.var_parent,
          values=self.added_joints)
        ent_mult = Entry(toplevel, textvariable=self.var_mult)
        btn_save = Button(toplevel, text="Add", command=self.addJoint)

        joint_dropdown.grid(row=0, column=0, **m_pad)
        ent_mimic.grid(row=1, column=0, **m_pad)
        parent_dropdown.grid(row=1, column=1, **m_pad)
        ent_mult.grid(row=1, column=2, **m_pad)
        btn_save.grid(row=2, column=1, **m_pad)

      else:
        messagebox.showinfo("Add Joint","Every joint is already instanced!")

  def onClear(self):
    """TODO"""
    self.added_joints = []
    self.joints = []
    self.removed_joints = list(self.all_joints)
    self.buildTree()

  def onCloseToplevel(self):
    self.toplevel.destroy()
    self.toplevel = None

  def addJoint(self):
    """TODO"""
    error = "Incorrect data provided!\n" +\
      "If joint is mimic then a valid parent joint must be specified, " +\
      "and its multiplier (between 0.0 and 1.0) must be given."
    try:
      name = self.var_joint.get()
      mimic = self.var_mimic.get()
      parent = self.var_parent.get()
      multiplier = self.var_mult.get()
    except TclError:
      messagebox.showerror("Error", error)
      return

    if mimic:
      if parent == "-" or ( multiplier < 0 or multiplier > 1):
        messagebox.showerror("Error", error)
        return
    else:
      parent = multiplier = "-"
    
    self.added_joints.append(name)
    self.joints.append ([name,bool(mimic),parent,multiplier])
    self.removed_joints.remove(name)

    self.onCloseToplevel()
    self.buildTree()

  def onRemoveJoint(self):
    """TODO"""
    sel_id = self.tree.selection()
    if sel_id:
      names = [self.tree.item(row_id)["values"][0] for row_id in sel_id]
      self.joints = [x for x in self.joints if x[0] not in names]
      self.added_joints = [x for x in self.added_joints if x not in names]
      for name in names:  
        self.removed_joints.append(name)
      self.buildTree()
    else:
      messagebox.showinfo("Remove Joint","No joint is selected!")

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
