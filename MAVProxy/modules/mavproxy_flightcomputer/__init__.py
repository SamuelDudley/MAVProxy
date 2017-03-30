#!/usr/bin/env python
'''
Flight Computer Module
Samuel Dudley, March 2017

This module is used to communicate with and show the status of a flight computer / its output.
'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time
import json, os

from MAVProxy.modules.mavproxy_map import mp_slipmap

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class flightcomputer(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(flightcomputer, self).__init__(mpstate, "flightcomputer", "")

        self.flightcomputer_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('flightcomputer', self.cmd_flightcomputer, "flightcomputer module", ['status','set (LOGSETTING)'])
        self.transmitters = self.load_transmitter_config('CMAC_Transmitters.json')
        self.draw_transmitters()
        self.system_whitelist = []
        
    def usage(self):
        '''show help on command line options'''
        return "Usage: flightcomputer <status|set|draw>"

    def cmd_flightcomputer(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "status":
            print self.status()
        elif args[0] == "set":
            self.flightcomputer_settings.command(args[1:])
        elif args[0] == "draw":
            self.draw_transmitters()
        else:
            print self.usage()

    def status(self):
        '''returns flight computer status'''
        pass

    def idle_task(self):
        '''called rapidly by mavproxy'''
        pass

    def mavlink_packet(self, m):
        '''filter and handle mavlink packets from ONBOARD_CONTROLLER'''
        if m.get_type() == 'HEARTBEAT':
            if m.type != mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER:
                self.system_whitelist.append(m.get_srcSystem())
        
        if m.get_srcSystem() not in self.system_whitelist:
            return
        
        # everything pass this point is from a ONBOARD_CONTROLLER
    

    def load_transmitter_config(self, file_name):
        file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)
        with open(file_path) as fid:    
            return json.load(fid)
    
    def draw_transmitters(self):
        # draw the transmitter(s) on the map
        if self.mpstate.map:
            self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('Transmitters'))
            for (idx,transmitter) in enumerate(self.transmitters):
                radius = 10
                circle = mp_slipmap.SlipCircle('Transmitter %u' % (idx+1), 'Transmitters',
                                                      (transmitter['latitude'], transmitter['longitude']),
                                                      radius,
                                                      (255,215,0), linewidth=2)
                self.mpstate.map.add_object(circle)
            

def init(mpstate):
    '''initialise module'''
    return flightcomputer(mpstate)
