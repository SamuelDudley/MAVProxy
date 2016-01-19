'''
Cesium viewer module
Samuel Dudley
Jan 2016
'''

import time, math
from math import *
import socket

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_menu import *  # popup menus
from pymavlink import mavutil

class CesiumModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(CesiumModule, self).__init__(mpstate, "cesium", "Cesium viewer module")
        self.add_command('cesium', self.cmd_cesium, [""])
        self.connected = False
        
        while not self.connected:
            try:
                self.cesium_connection = mavutil.mavtcp("127.0.0.1:14555")
                self.connected = True
            except:
                time.sleep(0.1)
                
        
        self.cesium_link= mavutil.mavlink.MAVLink(self.cesium_connection)
        

    def cmd_cesium(self, args):
        '''cland command parser'''
        usage = "usage: cesium <status|mission|fence> <set> (CESIUMSETTING)"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "status":
            print ''
#         elif args[0] == "set":
#             self.cland_settings.command(args[1:])
        elif args[0] == "mission":
            self.send_mission()
        elif args[0] == "fence":
            self.send_fence()
        else:
            print(usage)
            

    def send_fence(self):
        '''load and draw the fence in cesium'''
        self.fence_points_to_send = self.mpstate.public_modules['fence'].fenceloader.points
        for point in self.fence_points_to_send:
            self.cesium_connection.write(point.get_msgbuf())
            
    def send_mission(self):
        '''load and draw the mission in cesium'''
        self.mission_points_to_send = self.mpstate.public_modules['wp'].wploader.wpoints
        for point in self.mission_points_to_send:
            self.cesium_connection.write(point.get_msgbuf())
    
   
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            self.cesium_connection.write(m.get_msgbuf())
        if m.get_type() == 'ATTITUDE':
            self.cesium_connection.write(m.get_msgbuf())
            
        
   
    def idle_task(self):
        '''called on idle'''
        pass


def init(mpstate):
    '''initialise module'''
    return CesiumModule(mpstate)