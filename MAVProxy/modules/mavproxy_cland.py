'''
Support for C-LAND Trials
Samuel Dudley
Jan 2016
'''

import time
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

        self.add_command('cland', self.cmd_cland, ["cland control",
                                                 "<status>",
                                                 "<reset>",
                                                 "set (CLANDSETTING)"])

#         self.threat_detection_timer = mavutil.periodic_event(2)
#         self.threat_timeout_timer = mavutil.periodic_event(2)
        self.lat = None
        self.lon = None
        self.alt = None

    def cmd_cland(self, args):
        '''cland command parser'''
        usage = "usage: cland <reset> <status>" #no <set> at this point in time...
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
                print ''
#         elif args[0] == "set":
#             self.cland_settings.command(args[1:])
        elif args[0] == "reset":
            self.send_reset()
        else:
            print(usage)
            
    def send_reset(self):
        '''send a reset msg to the estimator'''
        time_since_last_position_update  = time.time() - self.last_position_update_time
        # TODO: check how old the last update was is prior to reseting
        #block until we get a msg...?
        msg = self.mpstate.master().recv_match(type = 'GLOBAL_POSITION_INT', blocking=True, timeout=5) #wait for a msg
        if msg is not None:
            self.master.mav.reset_est_send(msg.lat, msg.lon, msg.alt)
        else:
            print("Error: Failed to recv current location for reset")

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
            
        if m.get_type() == 'AUTO_SUBMODE':
            msg_dict = m.to_dict()
            if 1 in msg_dict.values():
                for key in msg_dict:
                    if msg_dict[key] == 1 and self.master.flightmode == 'AUTO':
                        self.auto_submode = key
                        
                        self.mpstate.rl.set_prompt('AUTO ['+self.auto_submode.upper()+']> ')
            else:
                self.auto_submode = ''
                self.mpstate.rl.set_prompt(self.mpstate.status.flightmode+'> ')
            

    def idle_task(self):
        '''called on idle'''
        pass


def init(mpstate):
    '''initialise module'''
    return CLANDModule(mpstate)

#RESET_EST reset the EST to a specified lat, lng, alt (wgs84 10.**3)
#TARGET_ICE sets a target for the FC (ICE) #cland 2
