'''
Cesium viewer module
Samuel Dudley
Jan 2016
'''

import time, math
from math import *
import socket


import threading
import Queue

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_menu import *  # popup menus
from pymavlink import mavutil

class FlaskIO():
    def __init__(self, data_queue):
        self.alive = True
        self.queue = data_queue
        self.thread = threading.Thread(target=self.main)
        self.thread.daemon = True
        self.thread.start()
        
        
    def is_alive(self):
        return self.alive
        
    def main(self):
        import json
        import eventlet
        #eventlet.monkey_patch()
        from flask_socketio import SocketIO
        import redis
        socketio = SocketIO(message_queue='redis://')
        while self.is_alive():
            data = self.queue.get(block = True, timeout = None) #block waiting for data
            socketio.emit(data.keys()[0], json.dumps(data.values()[0]), namespace='/test')
        
        

class CesiumModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(CesiumModule, self).__init__(mpstate, "cesium", "Cesium viewer module")
        self.add_command('cesium', self.cmd_cesium, [""])
        
        self.aircraft = {'lat':None, 'lon':None, 'alt_wgs84':None,
                         'roll':None, 'pitch':None, 'yaw':None}
        
        self.fence = {}
        self.mission = {}
        
        self.queue = Queue.Queue()
        self.websocket = FlaskIO(self.queue)
        

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
        self.fence = {}
        self.fence_points_to_send = self.mpstate.public_modules['fence'].fenceloader.points
        for point in self.fence_points_to_send:
            point_dict = point.to_dict()
            if point_dict['idx'] != 0: # dont include the return location
                self.fence[point_dict['idx']] = {"lat":point_dict['lat'], "lon":point_dict['lng']}
        self.queue.put_nowait({"fence_data":self.fence})
            
    def send_mission(self):
        '''load and draw the mission in cesium'''
        self.mission = {}
        self.mission_points_to_send = self.mpstate.public_modules['wp'].wploader.wpoints
        for point in self.mission_points_to_send:
            point_dict = point.to_dict()
            self.mission[point_dict['seq']] = {"x":point_dict['x'], "y":point_dict['y'], "z":point_dict['z']}
        self.queue.put_nowait({"mission_data":self.mission})
    
   
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
             
            msg_dict = m.to_dict()
            self.aircraft['lat']= msg_dict['lat']
            self.aircraft['lon'] = msg_dict['lon']
            self.aircraft['alt_wgs84'] = msg_dict['alt']
             
            if None not in self.aircraft.values():
                self.queue.put_nowait({"aircraft_data":self.aircraft})
               
             
        if m.get_type() == 'ATTITUDE':
 
            msg_dict = m.to_dict()
            self.aircraft['roll']= msg_dict['roll']
            self.aircraft['pitch'] = msg_dict['pitch']
            self.aircraft['yaw'] = msg_dict['yaw']
             
            if None not in self.aircraft.values():
                self.queue.put_nowait({"aircraft_data":self.aircraft})
                
    def idle_task(self):
        '''called on idle'''
        pass


def init(mpstate):
    '''initialise module'''
    return CesiumModule(mpstate)