#!/usr/bin/env python

'''
 core library for graphing in mavexplorer
'''
import numpy as np
import vispy.plot as vp
from vispy.color import get_colormap



import sys, struct, time, os, datetime
import math, re
from math import *

from pymavlink.mavextra import *
from pymavlink import mavutil
from numpy import * #needed for sqrt on arrays

colourmap = {
    'apm' : {
        'MANUAL'    : (1.0,   0,   0),
        'AUTO'      : (  0, 1.0,   0),
        'LOITER'    : (  0,   0, 1.0),
        'FBWA'      : (1.0, 0.5,   0),
        'RTL'       : (  1,   0, 0.5),
        'STABILIZE' : (0.5, 1.0,   0),
        'LAND'      : (  0, 1.0, 0.5),
        'STEERING'  : (0.5,   0, 1.0),
        'HOLD'      : (  0, 0.5, 1.0),
        'ALT_HOLD'  : (1.0, 0.5, 0.5),
        'CIRCLE'    : (0.5, 1.0, 0.5),
        'POSITION'  : (1.0, 0.0, 1.0),
        'GUIDED'    : (0.5, 0.5, 1.0),
        'ACRO'      : (1.0, 1.0,   0),
        'CRUISE'    : (  0, 1.0, 1.0)
        },
    'px4' : {
        'MANUAL'    : (1.0,   0,   0),
        'SEATBELT'  : (  0.5, 0.5,   0),
        'EASY'      : (  0, 1.0,   0),
        'AUTO'    : (  0,   0, 1.0),
        'UNKNOWN'    : (  1.0,   1.0, 1.0)
        }
    }

class MavGraphVispy(object):
    def __init__(self):
        # create figure with plot
        self.fig = vp.Fig(bgcolor='w', size=(800, 600), show=True)
        self.plt = self.fig[0, 0]
        self.plt._configure_2d()
        self.plt.title.text = ''
        self.plt.ylabel.text = ''
        self.plt.xlabel.text = ''
        self.selected = None
        self.fig.connect(self.on_mouse_press)
        self.fig.connect(self.on_mouse_move)
        self.fig.connect(self.on_mouse_wheel)
        self.linked = False
        self.fields = []
        self.lables = True
        
    def add_field(self, field):
        '''add another field to plot'''
        self.fields.append(field)
        
    def set_cam_link(self, cam):
        self.plt.camera.link(cam)
    
    def get_cam(self):
        return self.plt.camera
        
    def process(self, block=True):
        '''process and display graph'''
        self.msg_types = set()
        self.multiplier = []
        self.field_types = []

        # work out msg types we are interested in
        self.x = []
        self.y = []
        self.modes = []
        self.axes = []
        self.first_only = []
        re_caps = re.compile('[A-Z_][A-Z0-9_]+')
        for f in self.fields:
            caps = set(re.findall(re_caps, f))
            self.msg_types = self.msg_types.union(caps)
            self.field_types.append(caps)
        print self.field_types
#             self.y.append([])
#             self.x.append([])
#             self.axes.append(1)
#             self.first_only.append(False)
# 
#         if self.labels is not None:
#             labels = self.labels.split(',')
#             if len(labels) != len(fields)*len(self.mav_list):
#                 print("Number of labels (%u) must match number of fields (%u)" % (
#                     len(labels), len(fields)*len(self.mav_list)))
#                 return
#         else:
#             labels = None
#             
#         timeshift = self.timeshift
# 
#         for fi in range(0, len(self.mav_list)):
#             mlog = self.mav_list[fi]
#             self.process_mav(mlog, timeshift)
#             timeshift = 0
#             for i in range(0, len(self.x)):
#                 if self.first_only[i] and fi != 0:
#                     self.x[i] = []
#                     self.y[i] = []
#             if labels:
#                 lab = labels[fi*len(self.fields):(fi+1)*len(self.fields)]
#             else:
#                 lab = self.fields[:]
#             if self.multi:
#                 col = colors[:]
#             else:
#                 col = colors[fi*len(self.fields):]
#             self.plotit(self.x, self.y, lab, colors=col)
#             for i in range(0, len(self.x)):
#                 self.x[i] = []
#                 self.y[i] = []
#         pylab.draw()

    
    def set_data(self, vars):
    
    
        cmap = get_colormap('hsl', value=0.5)
        colors = cmap.map(np.linspace(0.1, 0.9, len(self.fields)))
        
        for i in range(0, len(self.fields)):
            f = self.fields[i]
            v = evaluate_expression(f, vars)
            x = range(len(v))
            line = self.plt.plot((x, v), color=colors[i])
            line.interactive = True
            line.unfreeze()  # make it so we can add a new property to the instance
            line.data_index = i
            line.x = x
            line.y = v
            line.label=f
            line.freeze()
            
        
        
        # Build visuals used for cursor
        self.cursor_text = vp.Text("", pos=(0, 0), anchor_x='left', anchor_y='center',
                              font_size=8, parent=self.plt.view.scene)
        self.cursor_line = vp.Line(parent=self.plt.view.scene)
        self.cursor_symbol = vp.Markers(pos=np.array([[0, 0]]), parent=self.plt.view.scene)
        self.cursor_line.visible = False
        self.cursor_symbol.visible = False
        self.cursor_line.order = 10
        self.cursor_symbol.order = 11
        self.cursor_text.order = 10
        
    def on_mouse_press(self, event):
        if event.handled or event.button != 1:
            return
        if self.selected is not None:
            self.selected.set_data(width=1)
        self.selected = None
        for v in self.fig.visuals_at(event.pos):
            if isinstance(v, vp.LinePlot):
                self.selected = v
                break
        if self.selected is not None:
            self.selected.set_data(width=3)
            self.update_cursor(event.pos)
    
    def on_mouse_wheel(self, event):
        self.update_cursor(event.pos)
    
    def on_mouse_move(self, event):
        self.update_cursor(event.pos)
    
    
    def update_cursor(self, pos):
        if self.selected is None:
            self.cursor_text.visible = False
            self.cursor_line.visible = False
            self.cursor_symbol.visible = False
        else:

            # map the mouse position to data coordinates
            tr = self.fig.scene.node_transform(self.selected)
            pos = tr.map(pos)

            # get interpolated y coordinate
            x_cur = pos[0]
            
          
            i = find_nearest(self.selected.x,x_cur)
            
            x = self.selected.x[i]
            y = self.selected.y[i]
            
            # update cursor
            if self.lables:
                self.cursor_text.text = "%s: x=%0.2f, y=%0.2f" % (self.selected.label,x, y)
            else:
                self.cursor_text.text =  "x=%0.2f, y=%0.2f" % (x, y)
            offset = np.diff(tr.map([[0, 0], [10, 10]]), axis=0)[0, 0]
            self.cursor_text.pos = x + offset, y
            rect = self.plt.view.camera.rect
            self.cursor_line.set_data(np.array([[x, rect.bottom], [x, rect.top]]))
            self.cursor_symbol.set_data(pos=np.array([[x, y]]), symbol='+',
                                   face_color='b')
            self.cursor_text.visible = True
            self.cursor_line.visible = True
            self.cursor_symbol.visible = True
    
    def show(self):
        self.fig.app.run(allow_interactive=True)
        
        

        
        

def find_nearest(array,value):
    idx = (np.abs(array-value)).argmin()
    return idx

def evaluate_expression(expression, vars): #nicked from mavutil
    '''evaluation an expression'''
    try:
        print expression
        print vars
        v =eval(expression, globals(), vars)
        print sqrt(v)
        print ""
        
    except NameError:
        return None
    except ZeroDivisionError:
        return None
    return v

