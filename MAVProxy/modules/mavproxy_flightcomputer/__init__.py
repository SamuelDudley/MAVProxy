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
        self.add_command('fc', self.cmd_flightcomputer, "flightcomputer module", ['status','set (LOGSETTING)'])
        self.transmitters = self.load_transmitter_config('CMAC_Transmitters.json')
        self.draw_transmitters()
        self.system_whitelist = [11]
        
    def usage(self):
        '''show help on command line options'''
        return "Usage: fc <status|set|draw|bootstrap|reset>" # TODO: write and handle commands on the flight computer

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
        elif args[0] == "param":
            if args[1].lower() == 'set':
                self.send_param(args[2].upper(), float(args[3]))
        elif args[0] == "bootstrap":
            self.send_bootstrap_msg(float(args[1]))
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
        
        if m.get_type() == 'SYSTEM_STATUS':
            pass
        
        if m.get_type() == 'RF_DATA':
            pass
        
        if m.get_type() == 'MSG_RATE':
            pass
        
        if m.get_type() == 'BEACON_POSITION_AND_RANGE':
            if self.mpstate.map:
                for (idx,transmitter) in enumerate(self.transmitters):
                    if idx == m.id:
                        radius = m.distance
                        circle = mp_slipmap.SlipCircle('Transmitter Radius %u' % (idx+1), 'TransmitterRanges',
                                                              (transmitter['latitude'], transmitter['longitude']),
                                                              radius,
                                                              (255,115,0), linewidth=1)
                        self.mpstate.map.add_object(circle)
        
    def send_param(self, param, val):
        for system in self.system_whitelist:
            self.master.mav.param_set_send(
                target_system = system,
                target_component = 220,
                param_id = param,
                param_value = val,
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_UINT32 # TODO: handle this
                )

    def load_transmitter_config(self, file_name):
        file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)
        with open(file_path) as fid:    
            return json.load(fid)
    
    def draw_transmitters(self):
        # draw the transmitter(s) on the map
        if self.mpstate.map:
            self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('TransmitterRanges'))
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
