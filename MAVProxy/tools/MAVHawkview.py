#!/usr/bin/env python
'''
log analysis program
based on mavexplorer by Andrew Tridgell 
Samuel Dudley September 2015
'''
import sys, struct, time, os, datetime
import math, re
import Queue
import fnmatch
import threading, multiprocessing
from math import *
from MAVProxy.modules.lib import rline
from MAVProxy.modules.lib import wxconsole
from MAVProxy.modules.lib import grapher
from MAVProxy.modules.lib import mavmemlog
from pymavlink.mavextra import *
from MAVProxy.modules.lib.mp_menu import *
from pymavlink import mavutil
from MAVProxy.modules.lib.mp_settings import MPSettings, MPSetting
from MAVProxy.modules.lib import wxsettings
from lxml import objectify
import pkg_resources

import numpy as np
from MAVProxy.modules.lib import grapher_vispy
from MAVProxy.modules.lib import cam_vispy

 
class MEStatus(object):
    '''status object to conform with mavproxy structure for modules'''
    def __init__(self):
        self.msgs = {}

class MEState(object):
    '''holds state of MAVExplorer'''
    def __init__(self):
        self.message_count = {}
        self.message_field_count = {}
        self.arrays = np.zeros(1,)
        self.cam = cam_vispy.XSyncCamera()
        self.input_queue = Queue.Queue()
        self.rl = None
        self.console = wxconsole.MessageConsole(title='MAVHawkview')
        self.exit = False
        self.status = MEStatus()
        self.settings = MPSettings(
            [ MPSetting('marker', str, '+', 'data marker', tab='Graph'),
              MPSetting('condition', str, None, 'condition'),
              MPSetting('xaxis', str, None, 'xaxis'),
              MPSetting('linestyle', str, None, 'linestyle'),
              MPSetting('flightmode', str, None, 'flightmode', choice=['apm','px4']),
              MPSetting('legend', str, 'upper left', 'legend position'),
              MPSetting('legend2', str, 'upper right', 'legend2 position')
              ]
            )

        self.mlog = None
        self.command_map = command_map
        self.completions = {
            "set"       : ["(SETTING)"],
            "condition" : ["(VARIABLE)"],
            "graph"     : ['(VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE)'],
            "map"       : ['(VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE)']
            }
        self.aliases = {}
        self.graphs = []
        self.flightmode_selections = []

def have_graph(name):
    '''return true if we have a graph of the given name'''
    for g in mestate.graphs:
        if g.name == name:
            return True
    return False

def menu_callback(m):
    '''called on menu selection'''
    if m.returnkey.startswith('# '):
        cmd = m.returnkey[2:]
        if m.handler is not None:
            if m.handler_result is None:
                return
            cmd += m.handler_result
        process_stdin(cmd)
    elif m.returnkey == 'menuSettings':
        wxsettings.WXSettings(mestate.settings)
    elif m.returnkey.startswith("mode-"):
        idx = int(m.returnkey[5:])
        mestate.flightmode_selections[idx] = m.IsChecked()
    else:
        print('Unknown menu selection: %s' % m.returnkey)


def flightmode_menu():
    '''construct flightmode menu'''
    modes = mestate.mlog.flightmode_list()
    ret = []
    idx = 0
    for (mode,t1,t2) in modes:
        modestr = "%s %us" % (mode, (t2-t1))
        ret.append(MPMenuCheckbox(modestr, modestr, 'mode-%u' % idx))
        idx += 1
        mestate.flightmode_selections.append(False)
    return ret

def messages_menu():
    '''construct messages menu'''
    msgs = mestate.message_field_count.keys()
    ret = []
    idx = 0
    for msg in msgs:
        msgstr = "%s" % (msg)
        ret.append(MPMenuCheckbox(msgstr,msgstr, 'mode-%u' % idx))
        idx += 1
    return ret

class graph_tree_state(object):
    def __init__(self, graphs):
        self.prefix = None
        self.graphs = graphs[:]

def graph_menus():
    '''return menu tree for graphs (recursive)'''
    ret = MPMenuSubMenu('Graphs', [])
    for i in range(len(mestate.graphs)):
        g = mestate.graphs[i]
        path = g.name.split('/')
        name = path[-1]
        path = path[:-1]
        ret.add_to_submenu(path, MPMenuItem(name, name, '# graph :%u' % i))
    return ret

def setup_menus():
    '''setup console menus'''
    menu = MPMenuTop([])
    menu.add(MPMenuSubMenu('MAVExplorer',
                           items=[MPMenuItem('Settings', 'Settings', 'menuSettings'),
                                  MPMenuItem('Map', 'Map', '# map')]))

    menu.add(graph_menus())
    menu.add(MPMenuSubMenu('FlightMode', items=flightmode_menu()))
    
    menu.add(MPMenuSubMenu('Messages', items=messages_menu()))

    mestate.console.set_menu(menu, menu_callback)

class GraphDefinition(object):
    '''a pre-defined graph'''
    def __init__(self, name, expression, description):
        self.name = name
        self.expression = expression
        self.description = description

def load_graph_xml(xml):
    '''load a graph from one xml string'''
    try:
        root = objectify.fromstring(xml)
    except Exception:
        return False
    if root.tag != 'graphs':
        return False
    if not hasattr(root, 'graph'):
        return False
    for g in root.graph:
        name = g.attrib['name']
        if have_graph(name):
            continue
        expressions = [e.text for e in g.expression]
        for e in expressions:
            graph_ok = True
            fields = e.split()
            for f in fields:
                try:
                    if f.endswith(':2'):
                        f = f[:-2]
                    if mavutil.evaluate_expression(f, mestate.status.msgs) is None:
                        graph_ok = False                        
                except Exception:
                    graph_ok = False
                    break
            if graph_ok:
                mestate.graphs.append(GraphDefinition(name, e, g.description.text))
                break
    return True

def count_msg_types():
    for msg in mestate.mlog._msgs:
        type = msg.get_type()
        if type not in mestate.message_count.keys():
            mestate.message_count[type] = 0
#             f='GPS.Spd'
#             vars = mlog.messages
#             print vars
#             v = mavutil.evaluate_expression(f, vars)
#             #print v
            #print len(msg.fmt.columns),msg.fmt.columns
            mestate.message_field_count[type] = msg.fmt.columns
        mestate.message_count[type] += 1
    #print mestate.message_count
    #print mestate.message_field_count #this holds a list of the field names for each msg type
    
    t2 = time.time()
    mestate.arrays = {}
    array_idx = {}
    
    ignore_list = ['FMT','MSG','PARM']
    msgs_to_parse = [x for x in mestate.message_field_count.keys() if x not in ignore_list]
    msgs_to_parse_dict = {}
    
    for msg in msgs_to_parse:
        msgs_to_parse_dict[msg] = mestate.message_field_count[msg]

    for msg_type in msgs_to_parse:
        #make data object
        
        mestate.arrays[msg_type] = MEData(msg_type, np.zeros((len(msgs_to_parse_dict[msg_type]),mestate.message_count[msg_type])))
        mestate.arrays[msg_type].array_idx = 0
    
    for msg in mestate.mlog._msgs:
        msg_type = msg.get_type()
        if msg_type in msgs_to_parse:
            for (idx,c) in enumerate(msgs_to_parse_dict[msg_type]):
                #print c
                try:
                    
                    mestate.arrays[msg_type].data[idx][mestate.arrays[msg_type].array_idx]=msg.__getattr__(c)
                    setattr(mestate.arrays[msg_type], c, mestate.arrays[msg_type].data[idx,:])
                except:
                    print type
                
            mestate.arrays[msg_type].array_idx +=1

    t3 = time.time()
    
    

    print 'time', t3-t2
    
class MEData(object):
    def __init__(self, type, data):
        self.type = type
        self.data = data
        
    def get_type(self):
        return self.type
    
    
def load_graphs():
    '''load graphs from mavgraphs.xml'''
    mestate.graphs = []
    gfiles = ['mavgraphs.xml']
    if 'HOME' in os.environ:
        for dirname, dirnames, filenames in os.walk(os.path.join(os.environ['HOME'], ".mavproxy")):
            for filename in filenames:
                if filename.lower().endswith('.xml'):
                    gfiles.append(os.path.join(dirname, filename))
    for file in gfiles:
        if not os.path.exists(file):
            continue
        if load_graph_xml(open(file).read()):
            mestate.console.writeln("Loaded %s" % file)
    # also load the built in graphs
    dlist = pkg_resources.resource_listdir("MAVProxy", "tools/graphs")
    for f in dlist:
        raw = pkg_resources.resource_stream("MAVProxy", "tools/graphs/%s" % f).read()
        if load_graph_xml(raw):
            mestate.console.writeln("Loaded %s" % f)
    mestate.graphs = sorted(mestate.graphs, key=lambda g: g.name)


def graph_process_vispy(fields):
    '''process for a vispy graph'''
    mgv = grapher_vispy.MavGraphVispy()
    mgv.set_cam_link(mestate.cam)
    
    for f in fields:
        mgv.add_field(f)
    
    mgv.set_data(mestate.arrays)
    mgv.process()
    mgv.show()


def graph_process(fields):
    '''process for a graph'''
    mestate.mlog.reduce_by_flightmodes(mestate.flightmode_selections)
    
    mg = grapher.MavGraph()
    mg.set_marker(mestate.settings.marker)
    mg.set_condition(mestate.settings.condition)
    mg.set_xaxis(mestate.settings.xaxis)
    mg.set_linestyle(mestate.settings.linestyle)
    mg.set_flightmode(mestate.settings.flightmode)
    mg.set_legend(mestate.settings.legend)
    mg.add_mav(mestate.mlog)
    for f in fields:
        mg.add_field(f)
    mg.process()
    mg.show()

def cmd_graph(args):
    '''graph command'''
    usage = "usage: graph <FIELD...>"
    if len(args) < 1:
        print(usage)
        return
    if args[0][0] == ':':
        i = int(args[0][1:])
        g = mestate.graphs[i]
        expression = g.expression
        args = expression.split()
        mestate.console.write("Added graph: %s\n" % g.name)
        if g.description:
            mestate.console.write("%s\n" % g.description, fg='blue')
    mestate.console.write("Expression: %s\n" % ' '.join(args))
    child = multiprocessing.Process(target=graph_process_vispy, args=[args])
    child.start()

def map_process(args):
    '''process for a graph'''
    from mavflightview import mavflightview_mav, mavflightview_options
    mestate.mlog.reduce_by_flightmodes(mestate.flightmode_selections)
    
    options = mavflightview_options()
    options.condition = mestate.settings.condition
    if len(args) > 0:
        options.types = ','.join(args)
    mavflightview_mav(mestate.mlog, options)

def cmd_map(args):
    '''map command'''
    child = multiprocessing.Process(target=map_process, args=[args])
    child.start()

def cmd_set(args):
    '''control MAVExporer options'''
    mestate.settings.command(args)

def cmd_condition(args):
    '''control MAVExporer conditions'''
    if len(args) == 0:
        print("condition is: %s" % mestate.settings.condition)
        return
    mestate.settings.condition = ' '.join(args)

def cmd_reload(args):
    '''reload graphs'''
    load_graphs()
    setup_menus()
    mestate.console.write("Loaded %u graphs\n" % len(mestate.graphs))

def cmd_param(args):
    '''show parameters'''
    if len(args) > 0:
        wildcard = args[0]
    else:
        wildcard = '*'
    k = sorted(mestate.mlog.params.keys())
    for p in k:
        if fnmatch.fnmatch(str(p).upper(), wildcard.upper()):
            print("%-16.16s %f" % (str(p), mestate.mlog.params[p]))

def process_stdin(line):
    '''handle commands from user'''
    if line is None:
        sys.exit(0)

    line = line.strip()
    if not line:
        return

    args = line.split()
    cmd = args[0]
    if cmd == 'help':
        k = command_map.keys()
        k.sort()
        for cmd in k:
            (fn, help) = command_map[cmd]
            print("%-15s : %s" % (cmd, help))
        return
    if cmd == 'exit':
        mestate.exit = True
        return

    if not cmd in command_map:
        print("Unknown command '%s'" % line)
        return
    (fn, help) = command_map[cmd]
    try:
        fn(args[1:])
    except Exception as e:
        print("ERROR in command %s: %s" % (args[1:], str(e)))

def input_loop():
    '''wait for user input'''
    while mestate.exit != True:
        try:
            if mestate.exit != True:
                line = raw_input(mestate.rl.prompt)
        except EOFError:
            mestate.exit = True
            sys.exit(1)
        mestate.input_queue.put(line)

def main_loop():
    '''main processing loop, display graphs and maps'''
    while True:
        if mestate is None or mestate.exit:
            return
        while not mestate.input_queue.empty():
            line = mestate.input_queue.get()
            cmds = line.split(';')
            for c in cmds:
                process_stdin(c)
        time.sleep(0.1)

command_map = {
    'graph'      : (cmd_graph,     'display a graph'),
    'set'        : (cmd_set,       'control settings'),
    'reload'     : (cmd_reload,    'reload graphs'),
    'condition'  : (cmd_condition, 'set graph conditions'),
    'param'      : (cmd_param,     'show parameters'),
    'map'        : (cmd_map,       'show map view'),
    }

mestate = MEState()
mestate.rl = rline.rline("MAV> ", mestate)

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("files", metavar="<FILE>", nargs="+")
args = parser.parse_args()

if len(args.files) == 0:
    print("Usage: MAVExplorer FILE")
    sys.exit(1)


def progress_bar(pct):
    if pct % 2 == 0:
        mestate.console.write('#')

mestate.console.write("Loading %s...\n" % args.files[0])
t0 = time.time()
mlog = mavutil.mavlink_connection(args.files[0], notimestamps=False,
                                  zero_time_base=False)
mestate.mlog = mavmemlog.mavmemlog(mlog, progress_bar)
mestate.status.msgs = mlog.messages

t1 = time.time()
mestate.console.write("\ndone (%u messages in %.1fs)\n" % (mestate.mlog._count, t1-t0))
  
count_msg_types()

load_graphs()
setup_menus()

# run main loop as a thread
mestate.thread = threading.Thread(target=main_loop, name='main_loop')
mestate.thread.daemon = True
mestate.thread.start()

# input loop
while True:
    try:
        try:
            line = raw_input(mestate.rl.prompt)
        except EOFError:
            mestate.exit = True
            break
        mestate.input_queue.put(line)
    except KeyboardInterrupt:
        mestate.exit = True
        break
