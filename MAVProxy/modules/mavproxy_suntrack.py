#!/usr/bin/ python
'''
Sun tracking inteface for UniPol compass trial.
Kent Rosser.
April 2015
'''
import time
import math
import random
import subprocess
import datetime 
import os

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_map import mp_elevation
from MAVProxy.modules.mavproxy_map import mp_settings
from pymavlink import mavutil

SCALE_LATLON = 1e-7 #ardu to DSTO
SCALE_ALT_GSPD_GCOR = 1e-2 #ardu to DSTO
SCALE_ALT = 1e-3 #ardu to DSTO


START_TIME = time.time()


class SunTrackModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SunTrackModule, self).__init__(mpstate, "suntrack")
        self.add_command("suntrack", self.cmd_suntrack, "Sun Track Inteface for UniPol")
        # Time lastcall to the SPA module.
        self.spa_time = time.time()

        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.year = 0
        self.month = 0
        self.day = 0
        self.hour = 0
        self.minute = 0
        self.second = 0
        self.path_spa = "/opt/git/ardupilot/libraries/AP_Polarisation/sunpos/"
        self.az = 0
        self.zen = 0

        # Define any settable settings.
        self.suntrack_settings = mp_settings.MPSettings(
            [('suntrack_active', bool, True),
             ('suntrack_interval', int, 10),
             ('suntrack_console', bool, True),
             ('suntrack_timezone', float, 10.5),  #Timezone 10.5 is daylight savings.
            ])

        self.update_next_suntrack_time()

        print "SunTrack Module loaded OK"
        
    def update_next_suntrack_time(self):
         t = time.time()
         self.spa_time = time.time() + (self.suntrack_settings.suntrack_interval)
#         print t
#         print self.spa_time
    
    def cmd_suntrack(self, args):
        '''Suntrack settings management.'''
        usage = "usage: suntrack <set|status>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "status":
            print ""
            print "SPA Time: ",self.spa_time
            print "Lon: ", self.lon
            print "Lat: ", self.lat
            print "Alt: ", self.alt
            print "Date: %i:%i:%i" % (self.year,self.month,self.day)
            print "Time: %i:%i:%i" % (self.hour,self.minute,self.second)
            print ""
        elif args[0] == "set":
            self.suntrack_settings.command(args[1:])
        else:
            print(usage)

    def run_command(self,command):
        # print command
        res_str = os.popen( command ).read()  # Fixme - this is deprecated.
        return res_str

    def update_suntrack(self):
        # Get the current time into year/month.day......
        dt = datetime.datetime.fromtimestamp(time.time())
        # print dt
        self.year = dt.year
        self.month = dt.month
        self.day  = dt.day
        self.hour = dt.hour
        self.minute = dt.minute
        self.second = dt.second
        self.timezone = self.suntrack_settings.suntrack_timezone

        cmd = "%sspa_command %.6f %.6f %.3f %i %i %i %i %i %i %.1f" % (self.path_spa, self.lat, self.lon, self.alt, self.year, self.month, self.day, self.hour, self.minute, self.second, self.timezone )

#        print cmd
        result_str = self.run_command( cmd )
#        print result_str
        azzen = result_str.split()
#        print azzen
        self.az = float(azzen[0]) 
        self.zen = float(azzen[1])
#        print (self.az,self.zen)

    def update_send_suntrack_params(self):
        self.update_suntrack( )
        self.param_set( 'POL_AZIMUTH', self.az )
        self.param_set( 'POL_ZENITH', self.zen )
        if( self.suntrack_settings.suntrack_console ):
            self.console.writeln( "Suntrack: Az %f, Z %f" % (self.az, self.zen))
        
         
    def idle_task(self):
        '''called when idle'''
        state = self
        
        if self.suntrack_settings.suntrack_active:
            if time.time() >= self.spa_time:
                # update the suntrack parameters.
                self.update_send_suntrack_params()
                self.update_next_suntrack_time()
                
            
    def mavlink_packet(self, m):
        '''handle a incoming mavlink packet'''
        state = self
        
        #parse msg is from ardupilot or sitl
        if m.get_type() == 'GLOBAL_POSITION_INT':
            self.lat = m.lat*SCALE_LATLON
            self.lon = m.lon*SCALE_LATLON
            self.alt = m.alt*SCALE_ALT
            

    
def init(mpstate):
    '''initialise module'''
    return SunTrackModule(mpstate)
