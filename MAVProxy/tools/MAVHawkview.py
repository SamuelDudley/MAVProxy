#!/usr/bin/env python
'''
log analysis program
hacked from mavexplorer by Andrew Tridgell to open and view (much) larger logs
Samuel Dudley September 2015
'''
import memory_profiler
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
from MAVProxy.modules.lib import mavmemlog_np
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


class Camera_Control(object):
    def __init__(self, rect):
        self.rect = rect
        
class MEStatus(object):
    '''status object to conform with mavproxy structure for modules'''
    def __init__(self):
        self.msgs = {}

class MEState(object):
    '''holds state of MAVExplorer'''
    def __init__(self):
        self.message_count = {}
        self.message_field_count = {}
        self.arrays = dict()
        self.plot_processes = []
        self.send_queues = []
        self.recv_queues = []
        self.master_rect = None
        self.flightmode_list = []
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
              MPSetting('legend2', str, 'upper right', 'legend2 position'),
              MPSetting('grid', str, 'off', 'grid', choice=['on','off'])
              ]
            )

        self.mlog = None
#         self.command_map = command_map
        self.completions = {
            "set"       : ["(SETTING)"],
            "condition" : ["(VARIABLE)"],
            "graph"     : ['(VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE)'],
            "map"       : ['(VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE)']
            }
        self.aliases = {}
        self.graphs = []
        self.flightmode_selections = []
    
    def add_array(self, msg_type):
        self.arrays.update({msg_type : MEData(msg_type, np.fromfile('/tmp/mav/'+msg_type+'.np',dtype = self.mlog.dtypes[msg_type]))})
    
    def get_array(self, msg_type):
        return self.arrays[msg_type].data
    
    def set_data(self, msg_type, data):
        self.arrays[msg_type].data = data
    
    def get_array_names(self):
        return self.arrays.keys()



class graph_tree_state(object):
    def __init__(self, graphs):
        self.prefix = None
        self.graphs = graphs[:]



class GraphDefinition(object):
    '''a pre-defined graph'''
    def __init__(self, name, expression, description):
        self.name = name
        self.expression = expression
        self.description = description





        
class MEFlightmode(object):
    def __init__(self, number, s_global = None , e_global = None, s_local = None, e_local = None):
        self.number = number
        self.s_global = s_global
        self.e_global = e_global
        
        
            
        self.s_local = s_local
        self.e_local = e_local
        self.duration_local = None
        
    def set_duration_global(self, start, end):
        self.duration_global = e_global - s_global
        

class MEData(object):
    def __init__(self, type, data):
        self.type = type
        self.data = data
        
    def get_type(self):
        return self.type
    





    

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

class Hawkview(object):
    def __init__(self, files):
        
        
        self.command_map = {
        'graph'      : (self.cmd_graph,     'display a graph'),
        'set'        : (self.cmd_set,       'control settings'),
        'reload'     : (self.cmd_reload,    'reload graphs'),
        'condition'  : (self.cmd_condition, 'set graph conditions'),
        'param'      : (self.cmd_param,     'show parameters'),
#         'map'        : (cmd_map,       'show map view'),
        }
        
        self.mestate = MEState()
        
        self.mestate.rl = rline.rline("MAV> ", self.mestate)


        self.mestate.console.write("Loading %s...\n" % files[0])
        
        self.mestate.file = args.files[0]
        t0 = time.time()
        mlog = mavutil.mavlink_connection(args.files[0], notimestamps=False,
                                          zero_time_base=False)
        self.mestate.mlog = mavmemlog_np.mavmemlog(mlog, self.progress_bar)
        self.mestate.status.msgs = mlog.messages

        t1 = time.time()
        self.mestate.console.write("\nDone! (%u messages in %.1fs)\n" % (self.mestate.mlog._count, t1-t0))
        #load_np_arrays()
        # t0 = time.time()
        # count_msg_types(progress_bar)
        # t1 = time.time()
        # mestate.console.write("\ndone (%u messages in %.1fs)\n" % (mestate.mlog._count, t1-t0))

        self.load_graphs()
        self.setup_menus()
    
#     def input_loop():
#         '''wait for user input'''
#         while mestate.exit != True:
#             try:
#                 if mestate.exit != True:
#                     line = raw_input(mestate.rl.prompt)
#             except EOFError:
#                 mestate.exit = True
#                 sys.exit(1)
#             mestate.input_queue.put(line)

    def cmd_set(self, args):
        '''control MAVExporer options'''
        self.mestate = mestate
        mestate.settings.command(args)

    def cmd_condition(self, args):
        '''control MAVExporer conditions'''
        self.mestate = mestate
        if len(args) == 0:
            print("condition is: %s" % mestate.settings.condition)
            return
        mestate.settings.condition = ' '.join(args)
    
    def cmd_reload(self, args):
        '''reload graphs'''
        self.mestate = mestate
        load_graphs()
        setup_menus()
        mestate.console.write("Loaded %u graphs\n" % len(mestate.graphs))
    
    def cmd_param(self, args):
        '''show parameters'''
        self.mestate = mestate
        if len(args) > 0:
            wildcard = args[0]
        else:
            wildcard = '*'
        k = sorted(mestate.mlog.params.keys())
        for p in k:
            if fnmatch.fnmatch(str(p).upper(), wildcard.upper()):
                print("%-16.16s %f" % (str(p), mestate.mlog.params[p]))

    def progress_bar(self, pct, end_val=100, bar_length=100):
        percent = float(pct) / end_val
        hashes = '|' * int(round(percent * bar_length))
        spaces = '-' * (bar_length - len(hashes))
        self.mestate.console.write("\r[ {0} ] {1}%".format(hashes + spaces, int(round(percent * 100))))
        
    def load_graphs(self):
        '''load graphs from mavgraphs.xml'''
        self.mestate.graphs = []
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
            if self.load_graph_xml(raw):
                self.mestate.console.writeln("Loaded %s" % f)
        self.mestate.graphs = sorted(self.mestate.graphs, key=lambda g: g.name)
    
    def have_graph(self, name):
        '''return true if we have a graph of the given name'''
        for g in self.mestate.graphs:
            if g.name == name:
                return True
        return False
    
    def graph_menus(self):
        '''return menu tree for graphs (recursive)'''
        ret = MPMenuSubMenu('Graphs', [])
        for i in range(len(self.mestate.graphs)):
            g = self.mestate.graphs[i]
            path = g.name.split('/')
            name = path[-1]
            path = path[:-1]
            ret.add_to_submenu(path, MPMenuItem(name, name, '# graph :%u' % i))
        return ret

    def setup_menus(self):
        '''setup console menus'''
        menu = MPMenuTop([])
        menu.add(MPMenuSubMenu('MAVExplorer',
                               items=[MPMenuItem('Settings', 'Settings', 'menuSettings'),
                                      MPMenuItem('Map', 'Map', '# map')]))
    
        menu.add(self.graph_menus())
        menu.add(MPMenuSubMenu('FlightMode', items=self.flightmode_menu()))
        
        menu.add(MPMenuSubMenu('Messages', items=self.messages_menu()))
    
        self.mestate.console.set_menu(menu, self.menu_callback)
        
    def menu_callback(self, m):
        '''called on menu selection'''
        if m.returnkey.startswith('# '):
            cmd = m.returnkey[2:]
            if m.handler is not None:
                if m.handler_result is None:
                    return
                cmd += m.handler_result
            self.process_stdin(cmd)
        elif m.returnkey == 'menuSettings':
            wxsettings.WXSettings(self.mestate.settings)
        elif m.returnkey.startswith("mode-"):
            idx = int(m.returnkey[5:])
            self.mestate.flightmode_selections[idx] = m.IsChecked()
        else:
            print('Unknown menu selection: %s' % m.returnkey)
    
    def load_graph_xml(self, xml):
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
            if self.have_graph(name):
                continue
            expressions = [e.text for e in g.expression]
            for e in expressions:
                graph_ok = True
                fields = e.split()
                for f in fields:
                    try:
                        if f.endswith(':2'):
                            f = f[:-2]
                        if mavutil.evaluate_expression(f, self.mestate.status.msgs) is None:
                            graph_ok = False                        
                    except Exception:
                        graph_ok = False
                        break
                if graph_ok:
                    self.mestate.graphs.append(GraphDefinition(name, e, g.description.text))
                    break
        return True
    
    def flightmode_menu(self):
        '''construct flightmode menu'''
        modes = self.mestate.mlog.flightmode_list()
        ret = []
        idx = 0
        for (mode,t1,t2) in modes:
            modestr = "%s %us" % (mode, (t2-t1))
            ret.append(MPMenuCheckbox(modestr, modestr, 'mode-%u' % idx))
            idx += 1
            self.mestate.flightmode_selections.append(False)
        return ret
    
    def messages_menu(self):
        '''construct messages menu'''
        msgs = self.mestate.mlog.message_field_count.keys()
        ret = []
        idx = 0
        for msg in msgs:
            msgstr = "%s" % (str(self.mestate.mlog.message_count[msg])+' : '+msg+' '+str(self.mestate.mlog.message_field_count[msg]))
            ret.append(MPMenuCheckbox(msgstr,msgstr, 'mode-%u' % idx))
            idx += 1
        return ret
    
    def main_loop(self):
        '''main processing loop, display graphs and maps'''
        while True:
            if self.mestate is None or self.mestate.exit:
                return
            while not self.mestate.input_queue.empty():
                line = self.mestate.input_queue.get()
                cmds = line.split(';')
                for c in cmds:
                    self.process_stdin(c)
            
            
            for idx,queue in enumerate(self.mestate.recv_queues):
                if idx == 0:
                    obj = None
                    while not queue.empty():
                        obj = queue.get()
                    
                    if obj is not None:
                        self.mestate.master_rect = obj.rect
    
                
                else:
                    slave_rect = None
                    while not queue.empty():
                        obj = queue.get()
                        slave_rect = obj.rect
                        
                    if slave_rect is not None and self.mestate.master_rect is not None:
                        slave_rect.left = self.mestate.master_rect.left
                        slave_rect.right = self.mestate.master_rect.right
                        self.mestate.send_queues[idx].put(Camera_Control(slave_rect))
                        
            time.sleep(0.1)
            
    def reduce_by_flightmodes(self, flightmode_selections):
        '''reduce data using flightmode selections'''
        if len(flightmode_selections) == 0:
            return
        all_false = True
        for s in flightmode_selections:
            if s:
                all_false = False
        if all_false:
            # treat all false as all modes wanted'''
            return
        # otherwise there are specific flight modes we have selected to plot
        times = []
        times = [(mode[1],mode[2]) for (idx, mode) in enumerate(self.mestate.mlog._flightmodes) if self.mestate.flightmode_selections[idx]]
        
        print 'times', times
        for msg_type in self.mestate.arrays.keys():
            mask = None
            for (t0, t1) in times:
                if mask is not None:
                    mask += (self.mestate.arrays[msg_type].timestamp >= t0) & (self.mestate.arrays[msg_type].timestamp <= t1)
                else:
                    mask = (self.mestate.arrays[msg_type].timestamp >= t0) & (self.mestate.arrays[msg_type].timestamp <= t1)
            
#             mask = ~mask #invert the mask
            print mask
            print len(self.mestate.arrays[msg_type].data)
            self.mestate.arrays[msg_type].data = self.mestate.arrays[msg_type].data[mask]
            print len(self.mestate.arrays[msg_type].data)
            


    
    def graph_process_vispy(self, args):
        '''process for a vispy graph'''
        self.reduce_by_flightmodes(self.mestate.flightmode_selections)
        fields = args[0:-2]
        
        send_queue = args[-2]
        recv_queue = args[-1]
        
        mgv = grapher_vispy.MavGraphVispy(send_queue,recv_queue)
        
        
        for f in fields:
            mgv.add_field(f)
        
        mgv.set_grid(self.mestate.settings.grid)#can be used to display a grid
        
        mgv.set_condition(self.mestate.settings.condition)
        mgv.set_xaxis(self.mestate.settings.xaxis)
        mgv.set_flightmode(self.mestate.settings.flightmode)
        mgv.set_data(self.mestate.arrays)
        mgv.set_legend(self.mestate.settings.legend)
        mgv.process()
        
        mgv.show()
        
    def process_stdin(self, line):
        '''handle commands from user'''
        if line is None:
            sys.exit(0)
    
        line = line.strip()
        if not line:
            return
    
        args = line.split()
        cmd = args[0]
        if cmd == 'help':
            k = self.command_map.keys()
            k.sort()
            for cmd in k:
                (fn, help) = self.command_map[cmd]
                print("%-15s : %s" % (cmd, help))
            return
        if cmd == 'exit':
            self.mestate.exit = True
            return
    
        if not cmd in self.command_map:
            print("Unknown command '%s'" % line)
            return
        (fn, help) = self.command_map[cmd]
        try:
            fn(args[1:])
        except Exception as e:
            print("ERROR in command %s: %s" % (args[1:], str(e)))
    
    def load_np_arrays(self, msg_types=None):
        print 'existing', self.mestate.get_array_names()
        if msg_types is not None:
            msg_types = [x for x in msg_types if ((x in self.mestate.mlog.dtypes.keys()) and (x not in self.mestate.arrays.keys()))]
            
        else:
            msg_types = [x for x in self.mestate.mlog.dtypes.keys() if x not in self.mestate.arrays.keys()]
        print 'loading', msg_types
        for msg_type in msg_types:
            
            self.mestate.add_array(msg_type)
            
#             print self.mestate.get_array(msg_type)
            # we have loaded the array, but we cant do operations on it simply as the datatypes are set.
            # recast to a float64 array
            fmt = '<'+'d'*len(self.mestate.mlog.message_field_count[msg_type])
            fmt_list = [fmt[0]+x for x in fmt[1:]]
            double_type = np.dtype(zip(self.mestate.mlog.message_field_count[msg_type],fmt_list))
    #         print double_type
            self.mestate.set_data(msg_type, self.mestate.get_array(msg_type).astype(dtype=double_type, casting='safe', subok=False, copy=False))
            
            
            #we have built the array.... now apply the atts.
            
            for col_name in self.mestate.mlog.message_field_count[msg_type]:
                setattr(self.mestate.arrays[msg_type], col_name, self.mestate.get_array(msg_type)[:][col_name])
                col_multi = self.mestate.mlog.msg_mults[msg_type][col_name]
    #             print col_name, col_multi
                if col_multi is not None:
                    self.mestate.get_array(msg_type)[:][col_name]*= float(col_multi)
                    
                    
                
            setattr(self.mestate.arrays[msg_type], 'min_timestamp', np.min(self.mestate.arrays[msg_type].timestamp))
            setattr(self.mestate.arrays[msg_type], 'max_timestamp', np.max(self.mestate.arrays[msg_type].timestamp))
                
    #         print mestate.arrays[msg_type].data
            print msg_type, (self.mestate.arrays[msg_type].data.nbytes)*10**-6, 'MiB'
    
    def cmd_graph(self, args):
        '''graph command'''
        mestate = self.mestate
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
        
        send_queue = multiprocessing.Queue()
        recv_queue = multiprocessing.Queue() 
        args.append(send_queue)
        args.append(recv_queue)
        
        
        fields = args[0:-2]
        fields_to_load = set([x.split('.')[0] for x in fields])
        #check to see if we have already loaded the fields...
        fields_to_load = [x for x in fields_to_load if (x in mestate.mlog.dtypes.keys() and x not in mestate.arrays.keys())]
        if len(fields_to_load) == 0:
            pass
        else:
            self.load_np_arrays(fields_to_load)
        
        mestate.plot_processes.append(multiprocessing.Process(target=self.graph_process_vispy, args=[args]))
        mestate.send_queues.append(send_queue)
        mestate.recv_queues.append(recv_queue)
        mestate.plot_processes[-1].start()
        
if __name__ == "__main__":
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("files", metavar="<FILE>", nargs="+")
    args = parser.parse_args()

    if len(args.files) == 0:
        print("Usage: MAVHawkview FILE")
        sys.exit(1)
    
        
    hawk = Hawkview(args.files)
    
    # run main loop as a thread
    hawk.mestate.thread = threading.Thread(target=hawk.main_loop, name='main_loop')
    hawk.mestate.thread.daemon = True
    hawk.mestate.thread.start()
    
    # input loop
    while True:
        try:
            try:
                line = raw_input(hawk.mestate.rl.prompt)
            except EOFError:
                hawk.mestate.exit = True
                break
            hawk.mestate.input_queue.put(line)
        except KeyboardInterrupt:
            hawk.mestate.exit = True
            break
