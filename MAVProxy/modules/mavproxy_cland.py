'''
Support for C-LAND Trials
Samuel Dudley
Jan 2016
'''

import time, math
from math import *

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_menu import *  # popup menus
from pymavlink import mavutil


class Estimator(object):
    '''a generic estimator'''

    def __init__(self, id, state):
        self.id = id
        self.state = state
        self.extra = {}
        self.vehicle_colour = 'green'  
        self.vehicle_type = 'plane' # use plane icon for now
        self.icon = self.vehicle_colour + self.vehicle_type + '.png'
        self.state_update_time = time.time()
        self.extra_update_time = None
        self.distance = 0
        self.bearing = 0
        self.is_active = True

    def update_state(self, state):
        '''update the estimator state'''
        self.state = state
        self.state_update_time = time.time()
        
    def update_extra(self, extra):
        '''update the estimator extra info'''
        self.extra = extra
        self.extra_update_time = time.time()


class CLANDModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(CLANDModule, self).__init__(mpstate, "cland", "C-LAND trials support")
        self.estimators = {}
        self.auto_submode = None
        self.cland_settings = mp_settings.MPSettings([("rlim", float, 20),  # +-roll limits     [degrees]
                                                      ("plim", float, 15),  # +-pitch limits    [degrees]
                                                      ("amax", float, 10000),  # max altitude    [wgs84? meters]
                                                      ("amin", float, 0),  # min altitude    [wgs84? meters]
                                                      ("tlim", float, 5),  # EST_TELE timeout limit   [seconds]
                                                      ("elim", float, 100),  # pos error limit [meters]
                                                      ("line", bool, True)]) # show the line that links the est to the true pos
        
        self.add_command('cland', self.cmd_cland, ["cland control",
                                                 "<status>",
                                                 "<reset>",
                                                 "set (CLANDSETTING)"])
        
        

    def cmd_cland(self, args):
        '''cland command parser'''
        usage = "usage: cland <start|stop|reset|status|fence> <set> (CLANDSETTING)"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "status":
            print ''
            for id in self.estimators:
                print ''
                print id
                print 'state:', self.estimators[id].state
                print 'extra:', self.estimators[id].extra
                print 'distance:', self.estimators[id].distance, 'bearing:', self.estimators[id].bearing
                print ''
        elif args[0] == "set":
            self.cland_settings.command(args[1:])
        elif args[0] == "reset":
            self.send_reset()
        elif args[0] == "start":
            self.send_cland_start()
        elif args[0] == "stop":
            self.send_cland_stop()
        elif args[0] == "fence":
            self.load_fence()
            
        else:
            print(usage)
            
    def send_reset(self):
        '''send a reset command to the AP in order to reset the estimator'''
        print("Sent DO_RESET_EST")
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_RESET_EST, # command
            0, # confirmation
            0, # param1
            0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7

    def send_cland_start(self):
        '''send a command to start the CLAND mode'''
        print("Sent DO_START_CLAND1")
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_START_CLAND1, # command
            0, # confirmation
            self.cland_settings.rlim, # Roll limits (degrees)
            self.cland_settings.plim, # Pitch limits (degrees)
            self.cland_settings.amin, # Min Alt (m)
            self.cland_settings.amax, # Max Alt (m)
            self.cland_settings.elim, # Max EST error (m)
            self.cland_settings.tlim, # EST_TELE timeout limit (seconds)
            0) # empty
        
    
    def load_fence(self):
        for (lat, lon) in self.fence_return():
            print lat, lon
            self.mpstate.public_modules['fence'].points = []
            self.mpstate.public_modules['fence'].fenceloader.add_latlon(lat, lon)
        
        self.mpstate.public_modules['fence'].send_fence()
        self.mpstate.public_modules['fence'].list_fence('fence.txt')
        
        self.mpstate.map.add_object(mp_slipmap.SlipPolygon('fence_kill', self.fence_kill(), layer=3, linewidth=2, colour=(255, 0, 0)))
        
    
    
        
    def send_cland_stop(self):
        '''send a command to stop the CLAND mode'''
        print("Sent DO_STOP_CLAND1")
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_STOP_CLAND1, # command
            0, # confirmation
            0, # param1
            0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7


    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'EST_TELE': #EST_EXTRA_TELE WP_SET CONTROL_SET
            id = 'cland'
            if id not in self.estimators.keys():  # check to see if the estimator is in the dict
                # if not then add it
                self.estimators[id] = Estimator(id=id, state=m.to_dict())
                if self.mpstate.map:  # if the map is loaded...
                    icon = self.mpstate.map.icon(self.estimators[id].icon)
                    # draw the vehicle on the map
                    self.mpstate.map.add_object(mp_slipmap.SlipIcon(id, (m.lat * 1e-7, m.lon * 1e-7),
                                                                    icon, layer=3, rotation=mavutil.wrap_180(m.yaw), follow=False,
                                                                    trail=mp_slipmap.SlipTrail(colour=(0, 255, 255))))
                    
            else:  # the estimator is in the dict
                # update the dict entry
                self.estimators[id].update_state(m.to_dict())
                if self.mpstate.map:  # if the map is loaded...
                    # update the map
                    self.mpstate.map.set_position(id, (m.lat * 1e-7, m.lon * 1e-7), rotation=mavutil.wrap_180(m.yaw))
        
        if m.get_type() == 'EST_EXTRA_TELE':
            id = 'cland'
            if id in self.estimators.keys():  # check to see if the estimator is in the dict
                self.estimators[id].update_extra(m.to_dict())
        
        if m.get_type() == 'FC_TELE':
            pass
        
        if m.get_type() == 'WP_SET':
            pass
        
        if m.get_type() == 'CONTROL_SET':
            pass 
        
        if m.get_type() == "GLOBAL_POSITION_INT":
            self.lat = m.lat
            self.lon = m.lon
            self.alt = m.alt
            self.last_position_update_time = time.time()
            for id in self.estimators:
                self.estimators[id].state
                (self.estimators[id].distance, self.estimators[id].bearing) = self.distance_bearing_two_points(
                                                                                                      self.estimators[id].state['lat']* 1e-7, self.estimators[id].state['lon']* 1e-7,
                                                                                                      m.lat* 1e-7, m.lon* 1e-7 
                                                                                                      )
                
                p = [(self.estimators[id].state['lat']* 1e-7, self.estimators[id].state['lon']* 1e-7), (m.lat* 1e-7, m.lon* 1e-7)]
                self.estimators[id].map_line = mp_slipmap.SlipPolygon(id+'line', p, layer=3,linewidth=1, colour=(0, 255, 255))
                self.estimators[id].map_line.set_hidden(not self.cland_settings.line)
                if self.mpstate.map:  # if the map is loaded...
                    self.mpstate.map.add_object(self.estimators[id].map_line)
            
        if m.get_type() == 'AUTO_SUBMODE':
            msg_dict = m.to_dict()
            if True in msg_dict.values():
                for key in msg_dict:
                    if msg_dict[key] == True and self.master.flightmode == 'AUTO':
                        self.auto_submode = key
                        
                        self.mpstate.rl.set_prompt(self.master.flightmode+' ['+self.auto_submode.upper()+']> ')
            else:
                self.auto_submode = None
                self.mpstate.rl.set_prompt(self.mpstate.status.flightmode+'> ')
            

    def idle_task(self):
        '''called on idle'''
        pass
    
    def distance_bearing_two_points(self, lat1, lon1, lat2, lon2):
        '''get the horizontal distance between two points'''
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
    
        dLat = lat2 - lat1
        dLon = lon2 - lon1
    
        # math as per mavextra.distance_two()
        a = math.sin(0.5 * dLat)**2 + math.sin(0.5 * dLon)**2 * math.cos(lat1) * math.cos(lat2)
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        
        b = math.atan2( math.sin(dLon)* math.cos(lat2),
                        math.cos(lat1)* math.sin(lat2)- math.sin(lat1)* math.cos(lat2)* math.cos(dLon))
        b = math.degrees(b)
        b = (b + 360) % 360
        
        return (6371. * 1000. * c, b)
    
    
    def fence_kill(self):
        points = [
                  (-30.923611, 136.524444),
                  (-30.8975, 136.524444),
                  (-30.8947123581, 136.53439135),
                  (-30.5180366333, 136.393053765),
                  (-30.4957131215, 136.47230589),
                  (-30.9116531829, 136.629078844),
                  (-30.9237442549, 136.585888539),
                  (-30.925277, 136.585833),
                  (-30.9377738859, 136.569449314),
                  (-30.937778, 136.544722),
                  (-30.923611, 136.524444)
                 ]
        
        return points
        
    def fence_return(self):
        points = [
                  (-30.923611, 136.524444),
                  (-30.8975, 136.524444),
                  (-30.8905294594, 136.549311294),
                  (-30.5267268708, 136.412790802),
                  (-30.5127803466, 136.462332028),
                  (-30.9029957688, 136.609393997),
                  (-30.9094563814, 136.586326039),
                  (-30.925277, 136.585833),
                  (-30.9377738859, 136.569449314),
                  (-30.937778, 136.544722),
                  (-30.923611, 136.524444)
                 ]
        return points
        


def init(mpstate):
    '''initialise module'''
    return CLANDModule(mpstate)