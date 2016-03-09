"""
    MAVProxy trials support for OF Drift containment module.
"""
import os, time
from pymavlink import mavwp, mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_menu import *
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_map import mp_slipmap

class OfDriftModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(OfDriftModule, self).__init__(mpstate, "ofdrift", "OfDrift controls", public = True)
        self.present = False
        self.enabled = False
        self.healthy = True
        self.add_command('ofdrift', self.cmd_ofdrift,
                         "ofdrift",
                         ["<heading>"])

        self.console.writeln( "Ofdrift Operations Loaded." )
        

    def cmd_ofdrift(self, args):
        '''ofdrift commands'''
        print "cmd"
        if len(args) < 1:
            self.print_usage()
            return

        print "Entering"

        mav = self.master

        print "1"

        i = 0
        if args[i] == "heading":
            mav.mav.command_long_send(
                mav.target_system, 
                mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_DST_EXP_START, 
                0, # confirm
                5, # OFDRIFT is 5.
                int(args[i+1]),  # Heading Angle 
                0, # param 3
                0, #param 4
                0, #param 5
                0, #parm 6
                0) #parm 7
            tempstr =  "OFDRIFT heading %s" % args[i]
            self.console.writeln( tempstr )
            print tempstr
            i += 1

 
        else:
            tempstr = "Unknown Ofdrift %s" % args[i]
            print tempstr
            self.console.writeln( tempstr )



    def print_usage(self):
        print("usage: ofdrift heading <val>")

def init(mpstate):
    '''initialise module'''
    return OfDriftModule(mpstate)
