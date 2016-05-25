"""
  MAVProxy map++

"""

import functools
import math
import os, sys, cmd
import time


from MAVProxy.modules.mavproxy_map import MapModule
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_map.mp_slipmap_util import *
from MAVProxy.modules.mavproxy_map.mp_slipmap import *
#from threading import Thread
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import wxconsole
import multiprocessing, threading
from MAVProxy.modules.lib.wxconsole_util import Value, Text
from MAVProxy.modules.mavproxy_map import mp_elevation
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import wxsettings
from MAVProxy.modules.lib.mp_menu import *
from wx import Size



class MPSlipMapPlus(MPSlipMap):
    '''
    an enhanced map viewer widget for use in mavproxy
    '''
    def __init__(self,
                 title='SlipMap',
                 lat=-35.362938,
                 lon=149.165085,
                 width=800,
                 height=600,
                 ground_width=1000,
                 tile_delay=0.3,
                 service="MicrosoftSat",
                 max_zoom=19,
                 debug=False,
                 brightness=1.0,
                 elevation=False,
                 download=True,
                 w=-1,
                 h=-1,
                 x=-1,
                 y=-1):
        
        # Window frame size and position
        self.frame_w=w
        self.frame_h=h
        self.frame_x=x
        self.frame_y=y
        
        MPSlipMap.__init__(self,
                 title,
                 lat,
                 lon,
                 width,
                 height,
                 ground_width,
                 tile_delay,
                 service,
                 max_zoom,
                 debug,
                 brightness,
                 elevation,
                 download)


    def child_task(self):
        '''child process - this holds all the GUI elements'''
        mp_util.child_close_fds()

        import MAVProxy.modules.lib.wx_processguard
        from MAVProxy.modules.lib.wx_loader import wx
        from MAVProxy.modules.mavproxy_map.mp_slipmap_ui import MPSlipMapFrame
        
        state = self

        self.mt = mp_tile.MPTile(download=self.download,
                                 service=self.service,
                                 tile_delay=self.tile_delay,
                                 debug=self.debug,
                                 max_zoom=self.max_zoom)
        state.layers = {}
        state.info = {}
        state.need_redraw = True

        self.app = wx.App(False)
        self.app.SetExitOnFrameDelete(True)
        self.app.frame = MPSlipMapFrame(state=self)
        # set the window size
        #print "Window size "+str(self.frame_w)+" x "+str(self.frame_h)
        if self.frame_w != -1 and self.frame_h != -1 :
            self.app.frame.SetSize(Size(self.frame_w,self.frame_h))
        if self.frame_x != -1 and self.frame_y != -1 :
            self.app.frame.SetPosition((self.frame_x,self.frame_y))
        self.app.frame.SetWindowStyle(wx.STAY_ON_TOP)
        self.app.frame.Show()
        self.app.MainLoop()


class MapPlusModule(MapModule):
    def __init__(self, mpstate):
        super(MapModule, self).__init__(mpstate, "map++", "map++ display", public = True)
        self.lat = None
        self.lon = None
        self.heading = 0
        self.wp_change_time = 0
        self.fence_change_time = 0
        self.rally_change_time = 0
        self.have_simstate = False
        self.have_vehicle = {}
        self.move_wp = -1
        self.moving_wp = None
        self.moving_fencepoint = None
        self.moving_rally = None
        self.mission_list = None
        self.icon_counter = 0
        self.click_position = None
        self.click_time = 0
        self.draw_line = None
        self.draw_callback = None
        self.have_global_position = False
        self.vehicle_type_name = 'plane'
        self.ElevationMap = mp_elevation.ElevationModel()
        self.last_unload_check_time = time.time()
        self.unload_check_interval = 0.1 # seconds
        self.showLandingZone = 0
        self.map_settings = mp_settings.MPSettings(
            [ ('showgpspos', int, 0),
              ('showgps2pos', int, 1),
              ('showsimpos', int, 0),
              ('showahrs2pos', int, 0),
              ('showahrs3pos', int, 0),
              ('brightness', float, 1),
              ('rallycircle', bool, False),
              ('loitercircle',bool, False)])
        service='OviHybrid'
        if 'MAP_SERVICE' in os.environ:
            service = os.environ['MAP_SERVICE']
        import platform
        from MAVProxy.modules.mavproxy_map import mp_slipmap
        #print "map mpstate.map_w "+str(mpstate.map_w)
        mpstate.map = MPSlipMapPlus(service=service,elevation=True,title='Map++',w=mpstate.map_w,h=mpstate.map_h,x=mpstate.map_x,y=mpstate.map_y)
        #mpstate.map = mp_slipmap.MPSlipMap(service=service, elevation=True, title='Map')
        mpstate.map_functions = { 'draw_lines' : self.draw_lines }
    
        mpstate.map.add_callback(functools.partial(self.map_callback))
        self.add_command('map', self.cmd_map, "map control", ['icon',
                                      'set (MAPSETTING)'])
        self.add_completion_function('(MAPSETTING)', self.map_settings.completion)

        self.default_popup = MPMenuSubMenu('Popup', items=[])
        self.add_menu(MPMenuItem('Fly To', 'Fly To', '# guided ',
                                 handler=MPMenuCallTextDialog(title='Altitude (m)', default=100)))
        self.add_menu(MPMenuItem('Set Home', 'Set Home', '# map sethome '))
        self.add_menu(MPMenuItem('Terrain Check', 'Terrain Check', '# terrain check'))
        self.add_menu(MPMenuItem('Show Position', 'Show Position', 'showPosition'))
        self.add_menu(MPMenuCheckbox('Show Landing Zone', 'Show Landing Zone', 'showLandingZone'))
        


def init(mpstate):
    '''initialise module'''
    return MapPlusModule(mpstate)
