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
        self.vehicle_colour = 'blue'  
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
        self.experimental_mode= {0:"none",1:"cland1",2:"cland2",3:"cland3",4:"systemid",5:"ofdrift"}
        self.auto_submode = self.experimental_mode[0]
        self.row_1 = 4
        self.mpstate.console.set_status('cland', '', fg='grey', row=self.row_1)
        self.row_2 = self.row_1 + 1
        self.mpstate.console.set_status('submode', 'NONE', fg='grey', row=self.row_2)
        self.row_3 = self.row_2 + 1
        self.mpstate.console.set_status('cland', '', fg='grey', row=self.row_3)
        self.row_4 = self.row_3 + 1
        self.mpstate.console.set_status('cland', '', fg='grey', row=self.row_4)
        
        self.cland_settings = mp_settings.MPSettings([("tlim", int, 500),  # EST_TELE timeout limit   [seconds]
                                                      ("elim", int, 1000),  # pos error limit [meters]
                                                      ("mode", int, 0),  # C-LAND mode (CLAND1 = 0, CLAND2_WP_SET = 1, CLAND2_CONTROL_SET = 2)
                                                      ("dmax", int, 1000),  # Maximum duration to remain in C-LAND submode [seconds]
                                                      ("line", bool, True)]) # show the line that links the est pos to the true pos
        
        self.add_command('cland', self.cmd_cland, ["cland control",
                                                 "<status>",
                                                 "<reset>",
                                                 "set (CLANDSETTING)"])
        
        mpstate.public_modules['map'].map_settings.loitercircle = True # Show loiter radius on map 
        mpstate.public_modules['map'].map_settings.showgps2pos = 0 # Hide the second GPS pos from the map
        

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
        print("Sent DO_START_CLAND")
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_START_CLAND, # command
            0, # confirmation
            self.cland_settings.elim, # Max EST error (m)
            self.cland_settings.tlim, # EST_TELE timeout limit (seconds)
            self.cland_settings.mode, # C-LAND mode (CLAND1 = 0, CLAND2_WP_SET = 1, CLAND2_CONTROL_SET = 2)
            self.cland_settings.dmax, # Maximum duration to remain in C-LAND submode (seconds)
            0, # empty
            0, # empty
            0) # empty
        
    def send_cland_stop(self):
        '''send a command to stop the CLAND mode'''
        print("Sent DO_STOP_CLAND")
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_STOP_CLAND, # command
            0, # confirmation
            0, # param1
            0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7

    def load_fence(self):
        '''load and draw the CLAND fences'''
        for (lat, lon) in self.fence_return():
            #clear the fence points (if any loaded)
            self.mpstate.public_modules['fence'].points = []
            #add the cland fence
            self.mpstate.public_modules['fence'].fenceloader.add_latlon(lat, lon)
        #send the fence to the ap
        self.mpstate.public_modules['fence'].send_fence()
        #save the fence and draw it on the map
        self.mpstate.public_modules['fence'].list_fence('fence.txt')
        #draw the second fence 
        self.mpstate.map.add_object(mp_slipmap.SlipPolygon('fence_kill', self.fence_kill(), layer=3, linewidth=2, colour=(255, 0, 0)))
        

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'EST_TELE':
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
                
                # update the console
                self.mpstate.console.set_status(id+'hdg', 'Hdg: %u/%u' % (int(m.yaw), m.gcourse*1.0e-2),
                                                 fg='black', row=self.row_3)
                
                self.mpstate.console.set_status(id+'alt_wgs84', 'AMSL: %u' % (int(m.alt_wgs84*1.0e-3)),
                                                 fg='black', row=self.row_3)
                
                self.mpstate.console.set_status(id+'alt_agl', 'AGL: %u' % (int(m.alt_agl*1.0e-3)),
                                                 fg='black', row=self.row_3)
                
                self.mpstate.console.set_status(id+'gspeed', 'GNDSpeed: %u' % (int(m.gspeed*1.0e-2)),
                                                 fg='black', row=self.row_3)
                
                self.mpstate.console.set_status(id+'vspeed', 'VSpeed: %u' % (int(m.vspeed*1.0e-2)),
                                                 fg='black', row=self.row_3)
                
                self.mpstate.console.set_status(id+'roll', 'Roll: %u' % (int(m.roll)),
                                                 fg='black', row=self.row_3)
                
                self.mpstate.console.set_status(id+'pitch', 'Pitch: %u' % (int(m.pitch)),
                                                 fg='black', row=self.row_3)
                
                if m.hacc <= 500:
                    self.mpstate.console.set_status(id+'acc', 'Acc: %u/%u' % (int(m.hacc), int(m.vacc)),
                                                 fg='green', row=self.row_2)
                else:
                    self.mpstate.console.set_status(id+'acc', 'Acc: %u/%u' % (int(m.hacc), int(m.vacc)),
                                                 fg='red', row=self.row_2)
                
                if m.filter_status <= 128:
                    self.mpstate.console.set_status(id+'filter_status', 'Status: {0:0>8b}'.format(m.filter_status),
                                                 fg='red', row=self.row_2)
                else:
                    self.mpstate.console.set_status(id+'filter_status', 'Status: {0:0>8b}'.format(m.filter_status),
                                                 fg='green', row=self.row_2)
                    
                if m.reset_init <= 0:
                    self.mpstate.console.set_status(id+'counter', 'Reset: %u' % (m.reset_counter),
                                                 fg='green', row=self.row_2)
                else:
                    self.mpstate.console.set_status(id+'counter', 'Reset: %u' % (m.reset_counter),
                                                 fg='red', row=self.row_2)
                
            
        if m.get_type() == 'EST_EXTRA_TELE':
            id = 'cland'
            if id in self.estimators.keys():  # check to see if the estimator is in the dict
                self.estimators[id].update_extra(m.to_dict())
            
                # update the console
                # TODO: set limits to make the text green or red...
                self.mpstate.console.set_status(id+'nadir', 'Nadir: %u' % (m.num_features_nadir),
                                                 fg='black', row=self.row_4)
                
                self.mpstate.console.set_status(id+'left', 'Left: %u' % (m.num_features_left),
                                                 fg='black', row=self.row_4)
                
                self.mpstate.console.set_status(id+'right', 'Right: %u' % (m.num_features_right),
                                                 fg='black', row=self.row_4)
                
                self.mpstate.console.set_status(id+'forward', 'Forward: %u' % (m.num_features_forward),
                                                 fg='black', row=self.row_4)
                
                
            
            
        if m.get_type() == 'FC_TELE':
            pass
        
        if m.get_type() == 'WP_SET':
            pass
        
        if m.get_type() == 'CONTROL_SET':
            pass 
        
        if m.get_type() == "GLOBAL_POSITION_INT":
            '''draw a line from the estimated pos to the true pos'''
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
            '''tell the operator what the submode is via a prompt change'''
            if self.master.flightmode == 'AUTO': #make sure we are in AUTO
                submodes = [key for key in m.to_dict().keys() if (key != 'mavpackettype' and m.to_dict()[key] > 0)]
                # ^ gets all the non-zero submodes from the msg and puts them into a list
                
                if len(submodes) == 0:
                    # we are not in a submode
                    self.auto_submode = "NONE"
                    self.console.set_status('submode', '%s' % (self.auto_submode.upper()), fg='grey')
                    self.mpstate.rl.set_prompt('%s>' % self.master.flightmode) #set the prompt to default
                    
                elif len(submodes) == 1:
                    # we are in a submode
                    self.auto_submode = submodes[0] #grab the submode name
                    if self.auto_submode == 'experimental_mode': # check to see if its an experimental mode
                        # if so get the experimental mode name from its number
                        self.auto_submode = self.experimental_mode[m.to_dict()[self.auto_submode]]
                        
                        if self.auto_submode in ['cland1', 'cland2', 'cland3']: #check to see if we are in a cland mode
                            self.console.set_status('submode', '%s' % (self.auto_submode.upper()), fg='green')
                    else:
                        #were in a submode but not a cland mode...
                        self.console.set_status('submode', '%s' % (self.auto_submode.upper()), fg='blue')
                    
                    # change the prompt to include the submode    
                    self.mpstate.rl.set_prompt('%s [%s]>' % (self.master.flightmode, self.auto_submode.upper())) 
                    
                else:
                    # we are in more than one submode... this is bad
                    self.auto_submode = 'error'
                    self.console.set_status('submode', '%s' % (self.auto_submode.upper()), fg='red')
                    self.mpstate.rl.set_prompt('%s [%s]>' % (self.master.flightmode, self.auto_submode.upper()))

                
        
        if m.get_type() == "NAV_CONTROLLER_OUTPUT":
            if (self.master.flightmode == "AUTO" and self.auto_submode in ['cland1', 'cland2', 'cland3']):
                
                self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('Trajectory')) #this will hide the default trajectory line
                
                for id in self.estimators: #this will show the cland est trajectory
                    trajectory = [ (self.estimators[id].state['lat']* 1e-7, self.estimators[id].state['lon']* 1e-7),
                                   mp_util.gps_newpos(self.estimators[id].state['lat']* 1e-7, self.estimators[id].state['lon']* 1e-7,
                                                      m.target_bearing, m.wp_dist) ]
                    self.mpstate.map.add_object(mp_slipmap.SlipPolygon('trajectory', trajectory, layer='Trajectory',
                                                                       linewidth=2, colour=(180,0,255)))


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