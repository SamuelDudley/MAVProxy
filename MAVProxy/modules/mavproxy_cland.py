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
        self.error_display = None
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
        
        self.master_cland_link = '/dev/uhard' # tcp:127.0.0.1:5760' # '/dev/uhard'
        
        
        self.estimators = {}
        self.experimental_mode= {0:"none",1:"cland1",2:"cland2",3:"cland3",4:"systemid",5:"ofdrift"} # as per mavlink msg defn.
        self.experimental_timer = None
        self.auto_submode = self.experimental_mode[0] # set the initial submode to 'none'
        self.row_1 = 4 #the first row of cland console text. this needs to be the first non-populated row
        
        #populate the remaining rows with temp items so that they can be updated with msg values at run time
        self.mpstate.console.set_status('cland', '', fg='grey', row=self.row_1)
        self.row_2 = self.row_1 + 1
        self.mpstate.console.set_status('submode', 'NONE', fg='grey', row=self.row_2)
        self.mpstate.console.set_status('experimental_timer', 'Sec: %s' % (str(self.experimental_timer).upper()),
                                                 fg='grey', row=self.row_2)
        self.row_3 = self.row_2 + 1
        self.mpstate.console.set_status('cland', '', fg='grey', row=self.row_3)
        self.row_4 = self.row_3 + 1
        self.mpstate.console.set_status('cland', '', fg='grey', row=self.row_4)
        
        self.cland_settings = mp_settings.MPSettings([("est_tele_time_lim", int, 5),  # EST_TELE timeout limit   [seconds]
                                                      ("pos_error_lim", int, 1000),  # pos error limit [meters]
                                                      ("mode", int, 0),  # C-LAND mode (CLAND1 = 0, CLAND2_WP_SET = 1, CLAND2_CONTROL_SET = 2)
                                                      ("cland_duration_sec", int, 1000),  # Maximum duration to remain in C-LAND submode [seconds]
                                                      ("show_error_circle", bool, True),  # Show error circle
                                                      ("error_circle_multi", float, 3.0),  # multiplier for error circle
                                                      # error_circle_multi set to 3 because the error is 1 sigma and we want to show 3 sigma
                                                      ("show_line", bool, True)]) # show the line that links the est pos to the true pos
        
        self.add_command('cland', self.cmd_cland, ["cland control",
                                                 "<status>",
                                                 "<reset>",
                                                 "set (CLANDSETTING)"])
        
        # set some map options we would otherwise have to type / script
        mpstate.public_modules['map'].map_settings.loitercircle = True # Show loiter radius on map (useful!)
        mpstate.public_modules['map'].map_settings.showgps2pos = 0 # Hide the second GPS pos from the map
        # the canbus GPS reports the wrong (fixed) heading and can cause confusion.
        # GPS2 status is still displayed in the console for health check purposes.
        mpstate.map.add_object(mp_slipmap.SlipBrightness(0.7)) # this makes the drawn icons / lines much easier to see...
        mpstate.map.add_object(mp_slipmap.SlipCenter((-30.931436, 136.544790))) # load the map at WTR...

    def cmd_cland(self, args):
        '''cland command parser'''
        usage = "usage: cland <start|stop|reset|status|fence|master> <set> (CLANDSETTING)"
        
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
        
        elif args[0] == "notam":
            self.load_notam()
            
        elif args[0] == "master":
            self.set_master()
        
            
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
            mavutil.mavlink.MAV_CMD_DO_DST_EXP_START, # command
            0, 
            1, # CLAND1 = mode 1
            0,
            0,
            0,
            0, # empty
            0, # empty
            0) # empty


    def send_cland_stop(self):
        '''send a command to stop the CLAND mode'''
        print("Sent DO_STOP_CLAND")
        
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_DST_EXP_STOP, # command
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
        # set some params to make sure the aircraft is using a fence and the correct
        # action will be taken when / if the fence is crossed
        self.param_set('FENCE_AUTOENABLE', 1,3)
        self.param_set('FENCE_ACTION', 0,3)
        self.param_set('FENCE_TOTAL', 0, 3) # clear the fence from the AP
        
        # clear the fence points (if any loaded)
        self.mpstate.public_modules['fence'].points = []
        self.mpstate.public_modules['fence'].fenceloader.clear()
        for (lat, lon) in self.fence_return():
            #add the cland fence
            self.mpstate.public_modules['fence'].fenceloader.add_latlon(lat, lon)
            
        # send the fence to the ap
        self.mpstate.public_modules['fence'].send_fence()
        # save the fence and draw it on the map
        self.mpstate.public_modules['fence'].list_fence('cland_fence.txt')
        # draw the second fence (not used by AP logic but enforced by IDP)
        self.mpstate.map.add_object(mp_slipmap.SlipPolygon('fence_kill', self.fence_kill(), layer=3, linewidth=2, colour=(255, 0, 0)))
        self.param_set('FENCE_ACTION', 1,3)
    
    def load_notam(self, all = True):
        try:
            from MAVProxy.modules import notam
            if all:
                notam_dict = notam.WTR()
                for airspace in notam_dict.keys():
                    self.mpstate.map.add_object(mp_slipmap.SlipPolygon(airspace, notam_dict[airspace], layer=3, linewidth=2, colour=(0, 0, 255)))
                
#                 self.mpstate.map.add_object(mp_slipmap.SlipPolygon('airspace', self.fence_huge(), layer=3, linewidth=2, colour=(128, 128, 255)))
                    
            else:
                self.mpstate.map.add_object(mp_slipmap.SlipPolygon('notam_222B', notam.R222B(), layer=3, linewidth=2, colour=(0, 0, 255)))
                self.mpstate.map.add_object(mp_slipmap.SlipPolygon('notam_222C', notam.R222C(), layer=3, linewidth=2, colour=(0, 0, 255)))
        except:
            # could not load the notam(s)
            pass
        
    def set_master(self):
        '''add a master link to a slave terminal in the case of a master terminal crash'''
        self.mpstate.public_modules['link'].link_add(self.master_cland_link) # add a master link
        
        self.mpstate.public_modules['link'].cmd_link_list() # show the current links
        self.mpstate.public_modules['output'].cmd_output_list() # show the current outputs

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'EST_TELE':
            id = 'cland' #hard coded for now, but could support a number of esitmators with unique id's
            if id not in self.estimators.keys():  # check to see if the estimator is in the dict
                # if not then add it
                self.estimators[id] = Estimator(id=id, state=m.to_dict())
                if self.mpstate.map:  # if the map is loaded...
                    icon = self.mpstate.map.icon(self.estimators[id].icon)
                    # draw the vehicle on the map
                    self.mpstate.map.add_object(mp_slipmap.SlipIcon(id, (m.lat * 1e-7, m.lon * 1e-7),
                                                                    icon, layer=3, rotation=mavutil.wrap_180(m.yaw), follow=False,
                                                                    trail=mp_slipmap.SlipTrail(colour=(0, 255, 255))))
                    
                    self.estimators[id].error_display = mp_slipmap.SlipCircle(id+"error_circle", 3,
                                                      (m.lat * 1e-7, m.lon * 1e-7),
                                                      self.cland_settings.error_circle_multi*m.hacc,
                                                      (0, 255, 255), linewidth=2)
                    # show the error display?
                    self.estimators[id].error_display.set_hidden(not self.cland_settings.show_error_circle)
                    self.mpstate.map.add_object(self.estimators[id].error_display)
                    
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
                
                
                
                if m.hacc < (250./sqrt(2)): # TODO what value makes sense here... ask AFRL
                    self.mpstate.console.set_status(id+'acc', 'Acc: %u/%u' % (int(m.hacc), int(m.vacc)),
                                                 fg='green', row=self.row_2)
                    if self.mpstate.map:  # if the map is loaded...
                        self.estimators[id].error_display = mp_slipmap.SlipCircle(id+"error_circle", 3,
                                                      (m.lat * 1e-7, m.lon * 1e-7),
                                                      self.cland_settings.error_circle_multi*m.hacc,
                                                      (0, 255, 255), linewidth=2) #(0, 255, 255), linewidth=2)
                    
                elif (m.hacc >= (250./sqrt(2)) and m.hacc < 250.):
                    self.mpstate.console.set_status(id+'acc', 'Acc: %u/%u' % (int(m.hacc), int(m.vacc)),
                                                 fg='orange', row=self.row_2)
                    if self.mpstate.map:  # if the map is loaded...
                        self.estimators[id].error_display = mp_slipmap.SlipCircle(id+"error_circle", 3,
                                                      (m.lat * 1e-7, m.lon * 1e-7),
                                                      self.cland_settings.error_circle_multi*m.hacc,
                                                      (255, 255, 0), linewidth=2)
                
                else:
                    self.mpstate.console.set_status(id+'acc', 'Acc: %u/%u' % (int(m.hacc), int(m.vacc)),
                                                 fg='red', row=self.row_2)
                    if self.mpstate.map:  # if the map is loaded...
                        self.estimators[id].error_display = mp_slipmap.SlipCircle(id+"error_circle", 3,
                                                      (m.lat * 1e-7, m.lon * 1e-7),
                                                      self.cland_settings.error_circle_multi*m.hacc,
                                                      (255, 0, 0), linewidth=2)
                    
                
                if m.filter_status <= 128: # TODO what value makes sense here... ask AFRL
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
                
                # show the error display?
                self.estimators[id].error_display.set_hidden(not self.cland_settings.show_error_circle)
                self.mpstate.map.add_object(self.estimators[id].error_display)
                   
        if m.get_type() == 'EST_EXTRA_TELE':
            # note this msg might not even get sent...
            id = 'cland'
            if id in self.estimators.keys():  # check to see if the estimator is in the dict
                # NOTE: we dont add the estimator to the dict here if it does not exist as we have no
                #       state to pass the init
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
            # needed for cland > 1
            pass
        
        if m.get_type() == 'WP_SET':
            # needed for cland > 1
            pass
        
        if m.get_type() == 'CONTROL_SET':
            # needed for cland > 1
            pass 
        
        if m.get_type() == "GLOBAL_POSITION_INT":
            '''draw a line from the estimated pos to the true pos'''
            # GLOBAL_POSITION_INT always reports the "safe" location of the aircraft
            for id in self.estimators:
                self.estimators[id].state
                (self.estimators[id].distance, self.estimators[id].bearing) = self.distance_bearing_two_points(
                                                                                                      self.estimators[id].state['lat']* 1e-7, self.estimators[id].state['lon']* 1e-7,
                                                                                                      m.lat* 1e-7, m.lon* 1e-7 
                                                                                                      )
                
                p = [(self.estimators[id].state['lat']* 1e-7, self.estimators[id].state['lon']* 1e-7), (m.lat* 1e-7, m.lon* 1e-7)]
                self.estimators[id].map_line = mp_slipmap.SlipPolygon(id+'line', p, layer=3,linewidth=1, colour=(0, 255, 255))
                self.estimators[id].map_line.set_hidden(not self.cland_settings.show_line) # show the line?
                if self.mpstate.map:  # if the map is loaded...
                    self.mpstate.map.add_object(self.estimators[id].map_line)
        
        if m.get_type() == 'AUTO_SUBMODE':
            '''tell the operator what the submode is via a prompt change and console update'''
#             if self.master.flightmode == 'AUTO': #make sure we are in AUTO
            submodes = [key for key in m.to_dict().keys() if (key not in ['mavpackettype', 'experimental_timer'] and m.to_dict()[key] > 0)]
            # ^ gets all the non-zero submodes from the msg and puts them into a list
            
            if len(submodes) == 0:
                # we are not in a submode
                self.auto_submode = "NONE"
                
                self.console.set_status('submode', '%s' % (self.auto_submode.upper()), fg='grey')
                self.mpstate.rl.set_prompt('%s>' % self.master.flightmode) #set the prompt to default
                
                self.experimental_timer = None
                self.mpstate.console.set_status('experimental_timer', 'Sec: %s' % (str(self.experimental_timer).upper()),
                                             fg='grey', row=self.row_2)
                
                
            elif len(submodes) == 1:
                # we are in a submode
                self.auto_submode = submodes[0] #grab the submode name from the first (and only) list entry
                if self.auto_submode == 'experimental_mode': # check to see if its an experimental mode
                    # if so get the experimental mode name from its value (see mavlink msg defn for notes)
                    experimental_value = m.to_dict()[self.auto_submode]
                    # re-define the auto_submode from the experimental_value
                    self.auto_submode = self.experimental_mode[experimental_value]
                    
                    # get the experimental timer
                    self.experimental_timer = m.to_dict()['experimental_timer']
                    
                    # add the timer to the console
                    self.mpstate.console.set_status('experimental_timer', 'Sec: %s' % (str(self.experimental_timer).upper()),
                                             fg='green', row=self.row_2)
                    
                    if self.auto_submode in ['cland1', 'cland2', 'cland3']: # check to see if we are in a cland mode
                        # if so make the console submode text green
                        self.console.set_status('submode', '%s' % (self.auto_submode.upper()), fg='green')
                    
                    else:
                        # we are not in a cland mode but we are in an experimental mode... e.g. SYSTEMID, OFDRIFT, etc...
                        self.console.set_status('submode', '%s' % (self.auto_submode.upper()), fg='blue')
                else:
                    # we are in a submode, but not a experimental mode... e.g. TAKEOFF, LAND, etc...
                    # make the console submode text blue
                    self.console.set_status('submode', '%s' % (self.auto_submode.upper()), fg='blue')
                    
                    self.experimental_timer = None
                    self.mpstate.console.set_status('experimental_timer', 'Sec: %s' % (str(self.experimental_timer).upper()),
                                             fg='grey', row=self.row_2)
                
                # change the prompt to include the submode which has been defined in the logic above  
                self.mpstate.rl.set_prompt('%s [%s]>' % (self.master.flightmode, self.auto_submode.upper()))
                
            else:
                # we are in more than one submode... this is bad
                self.auto_submode = 'error'
                # make the console submode text red and display 'ERROR'
                self.console.set_status('submode', '%s' % (self.auto_submode.upper()), fg='red')
                
                # change the prompt to include the submode which has been defined in the logic above  
                self.mpstate.rl.set_prompt('%s [%s]>' % (self.master.flightmode, self.auto_submode.upper()))
            
#             else:
#                 # we are not in auto...
#                 pass
                
        
        if m.get_type() == "NAV_CONTROLLER_OUTPUT":
            '''show the est trajectory while in cland mode'''
            # if we are in a cland mode
            if (self.master.flightmode == "AUTO" and self.auto_submode in ['cland1', 'cland2', 'cland3']):
                # hide the safe trajectory line
                self.mpstate.map.hide_object('trajectory')

                for id in self.estimators: #this will show the est trajectory
                    trajectory = [ (self.estimators[id].state['lat']* 1e-7, self.estimators[id].state['lon']* 1e-7),
                                   mp_util.gps_newpos(self.estimators[id].state['lat']* 1e-7, self.estimators[id].state['lon']* 1e-7,
                                                      m.target_bearing, m.wp_dist) ]
                    
                    self.mpstate.map.add_object(mp_slipmap.SlipPolygon(id+'trajectory', trajectory, layer='Trajectory',
                                                                       linewidth=2, colour=(180,0,255)))
                    # show the est trajectory
                    self.mpstate.map.hide_object(id+'trajectory', hide = False)
            else:
                # show the safe trajectory line
                self.mpstate.map.hide_object('trajectory', hide = False)
                
                for id in self.estimators:
                    # hide the est trajectory
                    self.mpstate.map.hide_object(id+'trajectory')
                
        if m.get_type() == 'DATALINK_STATUS':
            '''show the datalink rssi in the console'''
            self.datalink_rssi = m.to_dict()['rssi']
            error_values = [-1, -2] # -1 & -2 are error values.
            if self.datalink_rssi < -80 or self.datalink_rssi in error_values:
                self.mpstate.console.set_status('rssi', 'RSSI: %u' % (int(self.datalink_rssi)),
                                             fg='red', row=1)
            else:
                self.mpstate.console.set_status('rssi', 'RSSI: %u' % (int(self.datalink_rssi)),
                                             fg='green', row=1)
            


    '''the remainder of the code is helper functions'''


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
        '''return the fence points which when crossed will 'kill' the vehicle'''
        points = [
#                   (-30.923611, 136.524444),
#                   (-30.8975, 136.524444),
#                   (-30.8947123581, 136.53439135),
#                   (-30.5180366333, 136.393053765),
#                   (-30.4957131215, 136.47230589),
#                   (-30.9116531829, 136.629078844),
#                   (-30.9237442549, 136.585888539),
#                   (-30.925277, 136.585833),
#                   (-30.9377738859, 136.569449314),
#                   (-30.937778, 136.544722),
#                   (-30.923611, 136.524444)
                    
#                     (-30.883469 ,   136.518036),
#                     (-30.500778 ,   136.373077),
#                     (-30.500801  ,  136.499222),
#                     (-30.855524  ,  136.620621),
#                     (-30.917898  ,  136.620636),
#                     (-30.930731   , 136.578369),
#                     (-30.942970  ,  136.562332),
#                     (-30.942974  ,  136.542526),
#                     (-30.916700 ,   136.518066),
#                     (-30.883469  ,  136.518036)


                    (-30.88353333,136.518),
                    (-30.88048333,136.5289667),
                    (-30.50966667,136.3900833),
                    (-30.50978333,136.4774833),
                    (-30.86036667,136.6098667),
                    (-30.91711667,136.6098167),
                    (-30.92375,136.5858833),
                    (-30.92528333,136.5858333),
                    (-30.94296667,136.5623333),
                    (-30.94301667,136.5425),
                    (-30.9168,136.5180833),
                    (-30.88353333,136.518)
                 ]
        
        return points
        
        
    def fence_return(self):
        '''return the fence points which when crossed will cause the vehicle to enter RTL / Guided'''
        points = [
#                   (-30.910362, 136.556900),

#                   (-30.897497, 136.524445),
#                   (-30.890532, 136.549316),
#                   (-30.526728, 136.412796),
#                   (-30.512781, 136.462326),
#                   (-30.902996, 136.609390),
#                   (-30.909456, 136.586319),
#                   (-30.925278, 136.585831),
#                   (-30.937777, 136.569443),
#                   (-30.937780, 136.544739),
#                   (-30.923611, 136.524445),
#                   (-30.897497, 136.524445)

                    (-30.910362, 136.556900),
                    
                    (-30.88353333,136.518),
                    (-30.87658333,136.5440333),
                    (-30.52256667,136.4113833),
                    (-30.52255,136.4661),
                    (-30.86031667,136.59335),
                    (-30.90756667,136.5930333),
                    (-30.90945,136.5863333),
                    (-30.92528333,136.5858333),
                    (-30.94296667,136.5623333),
                    (-30.94301667,136.5425),
                    (-30.938851,136.538621),
                    (-30.938163,136.53984),
                    (-30.937668,136.539321),
                    (-30.938343,136.538117),
                    (-30.9168,136.5180833),
                    (-30.88353333,136.518)
                 ]
        
        return points
    
    def fence_huge(self):
        points = [
                    (-30.855270, 136.619113),#-30.930683, 136.621367
                    (-30.679289, 136.559957), # -30.626309, 136.541874
                    (-29.970729, 136.844298),
                    (-29.299318, 136.587105),
                    (-29.117657, 136.329931),# 
                    (-29.125321, 135.601567), #-29.127854, 135.340424
                    (-29.730517, 135.599256),# -29.513965, 135.343153
                    (-29.734899, 136.379325), #-29.797068, 136.396813 
                    (-30.319242, 136.580495),
                    (-30.503488, 136.501256),
                    ( -30.503673, 136.378492),
                    (-30.852734, 136.510357)
                    
                  ]
                #   
        return points
def init(mpstate):
    '''initialise CLAND module'''
    return CLANDModule(mpstate)