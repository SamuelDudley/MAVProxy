"""
  MAVProxy console

  uses lib/console.py for display
"""

import time, math, sys, cmd
from threading import Thread

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import wxconsole
import multiprocessing, threading
from MAVProxy.modules.lib.wxconsole_util import Value, Text
from MAVProxy.modules.lib.wxconsole import MessageConsole
from MAVProxy.modules.mavproxy_map import mp_elevation
from MAVProxy.modules.mavproxy_console import ConsoleModule
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import wxsettings
from MAVProxy.modules.lib import textconsole
from MAVProxy.modules.lib.mp_menu import *
from wx import Size


class MessageConsolePlus(MessageConsole):
    '''
    a message console that is always on top and can be positioned on screen
    '''
    def __init__(self,out_fname,w=-1,h=-1,x=-1,y=-1):
        self.w=w
        self.h=h
        self.x=x
        self.y=y
        MessageConsole.__init__(self)


    def child_task(self):
        '''child process - this holds all the GUI elements'''
        self.parent_pipe_send.close()
        self.parent_pipe_recv.close()
         
        import MAVProxy.modules.lib.wx_processguard
        from MAVProxy.modules.lib.wx_loader import wx
        from MAVProxy.modules.lib.wxconsole_ui import ConsoleFrame
        app = wx.App(False)
        app.frame = ConsoleFrame(state=self, title="Console++")
        app.frame.SetDoubleBuffered(True)
        app.frame.SetFocus()
        # set the window size
        #print "Window size "+str(self.w)+" x "+str(self.h)
        if self.w != -1 and self.h != -1 :
            app.frame.SetSize(Size(self.w,self.h))
        if self.x != -1 and self.y != -1 :
            app.frame.SetPosition((self.x,self.y))
        app.frame.SetWindowStyle(wx.STAY_ON_TOP)
        app.MainLoop()

class ConsolePlusModule(ConsoleModule):
    def __init__(self, mpstate):
        mp_module.MPModule.__init__(self,mpstate, "Console++", "Console++", public=True)
        self.set_console()

    def set_console(self) :
        self.in_air = False
        self.start_time = 0.0
        self.total_time = 0.0
        self.speed = 0
        self.max_link_num = 0
        self.last_sys_status_health = 0
        time_string=time.strftime("%Y%m%d-%H%M%S")
        self.mpstate.console=MessageConsolePlus(self,self.mpstate.console_w,self.mpstate.console_h,\
                                                self.mpstate.console_x,self.mpstate.console_y)
        self.mpstate.console.set_status('Mode', 'UNKNOWN', row=0, fg='blue')
        self.mpstate.console.set_status('ARM', 'ARM', fg='grey', row=0)
        self.mpstate.console.set_status('GPS', 'GPS: --', fg='red', row=0)
        self.mpstate.console.set_status('Vcc', 'Vcc: --', fg='red', row=0)
        self.mpstate.console.set_status('Radio', 'Radio: --', row=0)
        self.mpstate.console.set_status('INS', 'INS', fg='grey', row=0)
        self.mpstate.console.set_status('MAG', 'MAG', fg='grey', row=0)
        self.mpstate.console.set_status('AS', 'AS', fg='grey', row=0)
        self.mpstate.console.set_status('RNG', 'RNG', fg='grey', row=0)
        self.mpstate.console.set_status('AHRS', 'AHRS', fg='grey', row=0)
        self.mpstate.console.set_status('EKF', 'EKF', fg='grey', row=0)
        self.mpstate.console.set_status('Heading', 'Hdg ---/---', row=2)
        self.mpstate.console.set_status('Alt', 'Alt ---', row=2)
        self.mpstate.console.set_status('AGL', 'AGL ---/---', row=2)
        self.mpstate.console.set_status('AirSpeed', 'AirSpeed --', row=2)
        self.mpstate.console.set_status('GPSSpeed', 'GPSSpeed --', row=2)
        self.mpstate.console.set_status('Thr', 'Thr ---', row=2)
        self.mpstate.console.set_status('Roll', 'Roll ---', row=2)
        self.mpstate.console.set_status('Pitch', 'Pitch ---', row=2)
        self.mpstate.console.set_status('Wind', 'Wind ---/---', row=2)
        self.mpstate.console.set_status('WP', 'WP --', row=3)
        self.mpstate.console.set_status('WPDist', 'Distance ---', row=3)
        self.mpstate.console.set_status('WPBearing', 'Bearing ---', row=3)
        self.mpstate.console.set_status('AltError', 'AltError --', row=3)
        self.mpstate.console.set_status('AspdError', 'AspdError --', row=3)
        self.mpstate.console.set_status('FlightTime', 'FlightTime --', row=3)
        self.mpstate.console.set_status('ETR', 'ETR --', row=3)
        self.mpstate.console.ElevationMap = mp_elevation.ElevationModel()
        
        # create the main menu
        if mp_util.has_wxpython:
            self.menu = MPMenuTop([])
            self.add_menu(MPMenuSubMenu('MAVProxy',
                                        items=[MPMenuItem('Settings', 'Settings', 'menuSettings'),
                                               MPMenuItem('Map', 'Load Map', '# module load map')]))
def init(mpstate):
    '''initialise module'''
    return ConsolePlusModule(mpstate)
