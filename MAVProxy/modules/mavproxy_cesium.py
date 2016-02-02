'''
Cesium viewer module
Samuel Dudley
Jan 2016
'''

import time, math
from math import *
import socket

import json
import eventlet
from flask_socketio import SocketIO
import redis
import threading

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_menu import *  # popup menus
from pymavlink import mavutil

class FlaskIO():
    def __init__(self):
        self.thread = threading
        pass

class CesiumModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(CesiumModule, self).__init__(mpstate, "cesium", "Cesium viewer module")
        self.add_command('cesium', self.cmd_cesium, [""])
        
        self.aircraft = {'lat':None, 'lon':None, 'alt_wgs84':None,
                         'roll':None, 'pitch':None, 'yaw':None}
        

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
            #fence[fields['idx']] = {"lat":fields['lat'], "lon":fields['lng']}
            pass
            
    def send_mission(self):
        '''load and draw the mission in cesium'''
        self.mission_points_to_send = self.mpstate.public_modules['wp'].wploader.wpoints
        for point in self.mission_points_to_send:
            pass
    
   
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
             
            msg_dict = m.to_dict()
            self.aircraft['lat']= msg_dict['lat']
            self.aircraft['lon'] = msg_dict['lon']
            self.aircraft['alt_wgs84'] = msg_dict['alt']
             
            if None not in self.aircraft.values():
                eventlet.monkey_patch()
                socketio = SocketIO(message_queue='redis://')
                socketio.emit('aircraft_data', json.dumps(self.aircraft), namespace='/test')
             
        if m.get_type() == 'ATTITUDE':
 
            msg_dict = m.to_dict()
            self.aircraft['roll']= msg_dict['roll']
            self.aircraft['pitch'] = msg_dict['pitch']
            self.aircraft['yaw'] = msg_dict['yaw']
             
            if None not in self.aircraft.values():
                eventlet.monkey_patch()
                socketio = SocketIO(message_queue='redis://')
                socketio.emit('aircraft_data', json.dumps(self.aircraft), namespace='/test')
                
    def idle_task(self):
        '''called on idle'''
        pass


def init(mpstate):
    '''initialise module'''
    return CesiumModule(mpstate)