"""
    MAVProxy trials support for System Identification module.
"""
import os, time
from pymavlink import mavwp, mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_menu import *
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_map import mp_slipmap

class SysIdModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SysIdModule, self).__init__(mpstate, "sysid", "SysId controls", public = True)
        self.present = False
        self.enabled = False
        self.healthy = True
        self.add_command('sysid', self.cmd_sysid,
                         "sysid",
                         ["roll/pitch/rud"])

        self.console.writeln( "Sysid Operations Loaded." )
        

    def cmd_sysid(self, args):
        '''sysid commands'''
        print "cmd"
        if len(args) < 1:
            self.print_usage()
            return

        print "Entering"

        mav = self.master

        print "1"

        if args[0] == "roll":
            mav.mav.command_long_send(mav.target_system, 
                                      mav.target_component,
                                      mavutil.mavlink.MAV_CMD_DO_DST_EXP_START, 
                                      0,
                                      4,  # DST command 4 is SYSID.
                                      0,  # axis 0 = roll 
                                      0, 
                                      0, 
                                      0, 
                                      0, 
                                      0)
            tempstr =  "SYSID ROLL ON"
            self.console.writeln( tempstr )
            print tempstr
        elif args[0] == "pitch":
            mav.mav.command_long_send(mav.target_system, 
                                      mav.target_component,
                                      mavutil.mavlink.MAV_CMD_DO_DST_EXP_START, 
                                      0,
                                      4,  # DST command 4 is SYSID.
                                      1,  # axis 1 = pitch 
                                      0, 
                                      0, 
                                      0, 
                                      0, 
                                      0)
            tempstr =  "SYSID PITCH ON"
            self.console.writeln( tempstr )
            print tempstr
        elif args[0] == "yaw":
            mav.mav.command_long_send(mav.target_system, 
                                      mav.target_component,
                                      mavutil.mavlink.MAV_CMD_DO_DST_EXP_START, 
                                      0,
                                      4,  # DST command 4 is SYSID.
                                      2,  # axis 0 = rudder 
                                      0, 
                                      0, 
                                      0, 
                                      0, 
                                      0)
            tempstr =  "SYSID RUD ON"
            self.console.writeln( tempstr )
            print tempstr
 
        else:
            tempstr = "Unknown Sysid %s" % args[0]
            print tempstr
            self.console.writeln( tempstr )



    def print_usage(self):
        print("usage: sysid <roll/pitch/yaw>")

def init(mpstate):
    '''initialise module'''
    return SysIdModule(mpstate)
