"""
    MAVProxy trials support module -- Cheat shortcuts to avoid typing.
"""
import os, time
from pymavlink import mavwp, mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_menu import *

class CheatModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(CheatModule, self).__init__(mpstate, "cheat", "Cheatsheet operations from menus", public = True)
        self.present = False
        self.enabled = False
        self.healthy = True
        self.add_command('cheat', self.cmd_cheat,
                         "cheat",
                         ["[mode X] [Thr X] [SPD X] [alt X] [WP X] [L1 Period] [escrst]"])

        self.throttle_max = 100  # Default setting.
        self.reset_throttle_underway = False
        self.sequence_preflight = False
        self.preflight_state = 0
        self.menu_added_console = False
        self.menu_added_map = False
        self.cmd_submenu=MPMenuSubMenu('CMD',items=[MPMenuItem('AUTO', 'AUTO', '# mode auto'),
                                         MPMenuItem('RTL', 'RTL', '# mode rtl'),
                                         MPMenuItem('FEN OFF', 'FEN OFF', '# fence disable'),
                                         MPMenuItem('FEN ON', 'FEN ON', '# fence enable')])
        
        self.sim_submenu=MPMenuSubMenu('SIM',items=[MPMenuItem('WINDSPD0', 'WINDSPD0', '# param set SIM_WIND_SPD 0'),
                                         MPMenuItem('WINDSPD10', 'WINDSPD10', '# param set SIM_WIND_SPD 10'),
                                         MPMenuItem('WINDSPD20', 'WINDSPD20', '# param set SIM_WIND_SPD 20')])
        
        self.tweak_submenu=MPMenuSubMenu('TWEAK',items=[MPMenuItem('ROLL_P UP', 'ROLL_P UP', '# cheat tweak RLL2SRV_P 1.25'),
                                         MPMenuItem('ROLL_P DN', 'ROLL_P DN', '# cheat tweak RLL2SRV_P 0.8'),
                                         MPMenuItem('PITCH_P UP', 'PITCH_P UP', '# cheat tweak PTCH2SRV_P 1.25'),
                                         MPMenuItem('PITCH_P DN', 'PITCH_P DN', '# cheat tweak PTCH2SRV_P 0.8')])
        
        self.thr_submenu=MPMenuSubMenu('Thr',items=[MPMenuItem('ESCRST',  'ESCRST',  '# cheat escrst'),
                                         MPMenuItem('ThrM100',  'Thr100',  '# cheat thr 100'),
                                         MPMenuItem('Thr0',  'Thr0',  '# cheat thr 0')])        
        
        self.spd_submenu=MPMenuSubMenu('SPD',items=[MPMenuItem('SPD10',  'SPD10', '# cheat speed 10'),
                                         MPMenuItem('SPD15',  'SPD15', '# cheat speed 15'),
                                         MPMenuItem('SPD20',  'SPD20', '# cheat speed 20'),
                                         MPMenuItem('SPD25',  'SPD25', '# cheat speed 25'),
                                         MPMenuItem('SPD30',  'SPD30', '# cheat speed 30'),
                                         MPMenuItem('SPD35',  'SPD35', '# cheat speed 35')])   
        
        self.l1_submenu=MPMenuSubMenu('L1',items=[MPMenuItem('PERIOD UP',  'PERIOD UP', '# cheat tweak NAVL1_PERIOD 1.25'),
                                         MPMenuItem('PERIOD DN',  'PERIOD DN', '# cheat tweak NAVL1_PERIOD 0.8'),
                                         MPMenuItem('BRG_PERIOD UP',  'PERIOD UP', '# cheat tweak NAVL1_BRG_PERIOD 1.25'),
                                         MPMenuItem('BRG_PERIOD DN',  'PERIOD DN', '# cheat tweak NAVL1_BRG_PERIOD 0.8')]) 
        
        self.wp_loiter_rad_submenu=MPMenuSubMenu('WP_LOITER_RAD',items=[MPMenuItem('RADIUS 80',  'RADIUS 80', '# param set WP_LOITER_RAD 80' ),
                                         MPMenuItem('RADIUS 160',  'RADIUS 160', '# param set WP_LOITER_RAD 160' ),
                                         MPMenuItem('RADIUS 320',  'RADIUS 320', '# param set WP_LOITER_RAD 320' )]) 

        self.wp_submenu=MPMenuSubMenu('WP',items=[MPMenuItem('WP1',  'WP1', '# cheat wp 1'),
                                                  MPMenuItem('WP2',  'WP2', '# cheat wp 2'),
                                                  MPMenuItem('WP77',  'WP77', '# cheat wp 77')]) 
        
        self.graph_submenu=MPMenuSubMenu('GRAPH',items=[MPMenuItem('ATTITUDE',  'ATTITUDE', '# graph ylimits -40 40 ; graph degrees(ATTITUDE.roll) NAV_CONTROLLER_OUTPUT.nav_roll ; graph ylimits -30 30 ; graph degrees(ATTITUDE.pitch) NAV_CONTROLLER_OUTPUT.nav_pitch ; graph ylimits False'),
                                         MPMenuItem('NAV_ERR',  'NAV_ERR', '# graph NAV_CONTROLLER_OUTPUT.alt_error ; graph NAV_CONTROLLER_OUTPUT.aspd_error'),
                                         MPMenuItem('RNGDNR',  'RNGFNDR', '# graph RANGEFINDER.distance'), 
                                         MPMenuItem('SPEED',  'SPEED', '# graph VFR_HUD.airspeed ; graph GPS_RAW_INT.vel'),
                                         MPMenuItem('ASPEED',  'ASPEED', '# graph ylimits 25 50 ; graph VFR_HUD.airspeed NAV_CONTROLLER_OUTPUT.aspd_error/100+VFR_HUD.airspeed ; graph ylimits False'),
                                         MPMenuItem('FLOW',  'FLOW', '# graph OPTICAL_FLOW.flow_x OPTICAL_FLOW.flow_y'), 
                                         MPMenuItem('DBGV',  'DBGV',   '# graph DEBUG_VECT.x DEBUG_VECT.y DEBUG_VECT.z'),
                                         MPMenuItem('POL',  'POL', '# graph POL_COMPASS.polCompass ; graph POL_COMPASS.heading POL_COMPASS.refheading'),
                                         MPMenuItem('POS_ERR',  'POS_ERR', '#  graph ylimits 0 500 ; graph EST_TELE.hacc ; graph ylimits False')]) 
        
        self.pol_submenu=MPMenuSubMenu('POL',items=[ MPMenuItem('REVERSE',  'REVERSE', '# param set POL_REVERSE 1')]) 
        
        self.ofd_submenu=MPMenuSubMenu('OFD',items=[MPMenuItem('WEST',  'WEST', '# ofdrift heading 27000'),
                                         MPMenuItem('EAST',  'EAST', '# ofdrift heading 9000' )]) 
        
        self.cland_submenu=MPMenuSubMenu('CLAND',items=[MPMenuItem('START',  'START', '# cland start'),
                                         MPMenuItem('STOP',  'STOP', '# cland stop'),
                                         MPMenuItem('ALT ENABLE',  'ALT_ENABLE', '# param set CLND_ALT_ENABLE 1'),
                                         MPMenuItem('ALT DISABLE',  'ALT_DISABLE', '# param set CLND_ALT_ENABLE 0')]) 
        
        self.sid_submenu=MPMenuSubMenu('SID',items=[MPMenuItem('CHIRP',  'CHIRP', '# param set SI_TYPE 2 ; param set SI_TPULSE 0.3 ; param set SI_FGRAD 0.25'),
                                         MPMenuItem('SINE',  'SINE', '# param set SI_TYPE 3 ; param set SI_TPULSE 1.0' ),
                                         MPMenuItem('PULSE', 'PULSE', '# param set SI_TYPE 1 ; param set SI_TPULSE 1.0' ),
                                         MPMenuItem('DBLET', 'DBLET', '# param set SI_TYPE 4 ; param set SI_TPULSE 1.0' ),
                                         MPMenuItem('CMD_ROLL', 'CMD_ROLL', '# sysid roll' ),
                                         MPMenuItem('CMD_PITCH', 'CMD_PITCH', '# sysid pitch' ),
                                         MPMenuItem('CMD_RUD', 'CMD_RUD', '# sysid rud' )])
        
        self.load_submenu=MPMenuSubMenu('LOAD',items=[MPMenuItem('Mission Editor',  'Mission Editor', '# module unload misseditor; module load misseditor'),
                                         MPMenuItem('submode',  'submode', '# module unload submode; module load mavproxy_submode'),
                                         MPMenuItem('sysid',  'sysid', '# module unload sysid; module load sysid'),
                                         MPMenuItem('Optical Flow Drift',  'Optical Flow Drift', '# module unload ofdrift; module load ofdrift'),
                                         MPMenuItem('Beacon',  'Beacon', '# module unload beacon; module load mavproxy_beacon'),
                                         MPMenuItem('Overlay',  'Overlay', '# module unload overlay; module load mavproxy_overlay'),
                                         MPMenuItem('Sequencer',  'Sequencer', '# module unload sequencer; module load mavproxy_sequencer'),
                                         MPMenuItem('Graph',  'Graph', '# module unload graph; module load graph'),
                                         MPMenuItem('SunTrack',  'SunTrack', '# module unload suntrack; module load suntrack')])
        
        self.update_submenu=MPMenuSubMenu('UPDATE',items=[MPMenuItem('CHEAT', 'CHEAT', '# module unload cheat; module load cheatsheet') ])
        
        self.menu = MPMenuSubMenu('Cheat',items=[])

        self.menu.add([self.cmd_submenu,self.sim_submenu, self.tweak_submenu, self.thr_submenu, self.spd_submenu, self.l1_submenu,
                       self.wp_loiter_rad_submenu,self.wp_submenu,self.graph_submenu,self.pol_submenu,self.ofd_submenu,self.sid_submenu,
                       self.cland_submenu,self.load_submenu,self.update_submenu])
        self.console.writeln( "Cheat Operations Loaded." )
        
    def unload(self):
        '''unload module'''
        if self.menu_added_map == True:
            #TODO: remove from map
            pass
        if self.menu_added_console == True:
            #TODO: remove from console
            pass
        pass

    def idle_task(self):
        '''called on idle'''
        if not self.menu_added_console and self.module('console') is not None:
            self.menu_added_console = True
            self.module('console').add_menu(self.menu)
        if not self.menu_added_map and self.module('map') is not None:
            self.menu_added_map = True
            self.module('map').add_menu(self.menu)

        # Action management.
        if self.reset_throttle_underway:
            if self.last_throttle == 0:  # Wait for the throttle to go to off.
              self.set_throttle_enable( True )
              self.reset_throttle_underway = False
              print "CHEAT ESCRST Completed"

#        self.console.writeln( "seq %i", self.preflight_state )
        
        if self.sequence_preflight:
            self.run_preflight_seq()
              

    def mavlink_packet(self, m):
        '''handle and incoming mavlink packet'''
        if m.get_type() == "VFR_HUD":
            self.last_throttle = m.throttle
        elif m.get_type() in ['SYS_STATUS']:
            bits = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE
        elif m.get_type() == "HEARTBEAT":
            current_mode = m.custom_mode

    def run_preflight_seq():
        self.console.writeln( "seq %i", preflight_state )
        if( preflight_state == 0 ):
            #wait for flight mode not initialising
            if current_mode != INITIALISING:
                preflight_state = 1
                tempstr = "%s %i %s" % ( seq_name, preflight_state, "INITIALISED" )
                self.console.writeln( tempstr )
                preflight_state = 1
        elif( preflight_state == 1 ):
            # Download wps.
            tempstr = "%s %i %s" % ( seq_name, preflight_state, "BLAH" )
            self.console.writeln( tempstr )
            preflight_state = 2
        elif( preflight_state == 2 ):
            # Download Fence.
            tempstr = "%s %i %s" % ( seq_name, preflight_state, "BLAH" )
            self.console.writeln( tempstr )
            preflight_state = 3
        elif( preflight_state == 3 ):
            # Download rally.
            tempstr = "%s %i %s" % ( seq_name, preflight_state, "BLAH" )
            self.console.writeln( tempstr )
            preflight_state = 4
        elif( preflight_state == 4 ):
            # Download wps.
            tempstr = "%s %i %s" % ( seq_name, preflight_state, "BLAH" )
            self.console.writeln( tempstr )
            preflight_state = 5
        elif( preflight_state == 5 ):
            # Download wps.
            tempstr = "%s %i %s" % ( seq_name, preflight_state, "BLAH" )
            self.console.writeln( tempstr )
            preflight_state = 6
        else:
            tempstr = "%s %i %s" % ( seq_name, preflight_state, "COMPLETE" )
            self.console.writeln( tempstr )
            preflight_state = 1
            self.sequence_preflight = False


        

    ## Waypoint variation functions.
    def set_tgt_wp(self, idx):
        '''Set the target WP'''
        self.master.waypoint_set_current_send(int(idx))
        tempstr = "WP set to target - %i" % idx
        self.console.writeln( tempstr )
        print tempstr

    # Mode control functions
    def execute_mode( self, name ):
        mode_mapping = self.master.mode_mapping()
        if mode_mapping is None:
            print('No mode mapping available')
            return
        mode = name
        if mode not in mode_mapping:
            print('Unknown mode %s: ' % mode)
            return
        self.master.set_mode(mode_mapping[mode])
        tempstr = "CHEAT - %s" % mode
        self.console.writeln( tempstr )

    # Mode control functions
    def tweak_parm( self, name, scalar ):
       val = self.get_mav_param(name )    # Get the parameter.
       if( not( val == None )):
           new_val = scalar*val
           self.param_set( name, new_val ) 
           tempstr = "CHEAT - %s from %f to %f" % ( name, val, new_val )
       else:
           tempstr = "CHEAT - couldn't find param %s" % name
       self.console.writeln( tempstr )


    # Throttle Management function
    def set_throttle_max(self, throttle ):
        '''set the parametrs in throttle sytsem'''
        self.throttle_max = throttle
        self.param_set('THR_MAX', self.throttle_max )    # Set THRMAX
        self.param_set('THR_MIN', 0 )                    # make sure min is 0.
        tempstr = "THR_MAX = %i, THR_MIN = %i\n" % (self.throttle_max, 0 )
        print tempstr
        self.console.writeln( tempstr )
 
    def set_throttle_enable(self, enable ):
        if( enable ):
          self.param_set('THR_MAX', self.throttle_max )    # Set THRMAX
          self.console.writeln( "ENABLE THR" )
        else:
          self.param_set('THR_MAX', 0 )                    # THRMAX to zero.
          self.console.writeln( "DISABLE THR" )
       
    # Speed Management function
    def set_airspeed(self, speed ):
        '''set the parametrs in throttle sytsem'''
        self.speed_tgt_cm = speed * 100
        self.param_set('TRIM_ARSPD_CM', self.speed_tgt_cm )    # Set ARSPD_CM
        tempstr = "Airspeed CD = %i (%i cm/s)\n" % (speed, self.speed_tgt_cm )
        print tempstr
        self.console.writeln( tempstr )
         
    # L1 Management function
    def set_L1_period(self, period ):
        '''set the parametrs in throttle sytsem'''
        self.l1_period = period
        self.param_set('NAVL1_PERIOD', self.l1_period )    # Set L1 Period
        tempstr = "L1 PEriod = %i seconds\n" % (self.l1_period )
        print tempstr
        self.console.writeln( tempstr )
                
    # Altitude Management function
    def set_altitude(self, altitude ):
        '''set the parameters in altitude sytsem'''
        self.altitude_tgt = altitude
        tempstr = "YET TO WORKOUT HOW TO DO THIS - PERHAPS WP UPDATE?\n"
        print tempstr
        self.console.writeln( tempstr )
         

    def cmd_cheat(self, args):
        '''cheat commands'''
        if len(args) < 1:
            self.print_usage()
            return

        if args[0] == "alt":
            altitude = float( args[1])
            self.set_altitude( altitude )
            print "CHEAT altitude - %s" % args[1]
        elif args[0] == "speed":
            airspeed = float( args[1])
            self.set_airspeed( airspeed )
            print "CHEAT airspeed - %s" % args[1]
        elif args[0] == "wp":
            idx = float( args[1])
            self.set_tgt_wp( idx )
            print "CHEAT WP - %s" % args[1]
        elif args[0] == "thr":
            thr_val = float( args[1])
            self.set_throttle_max( thr_val )
            print "CHEAT THR - %s" % args[1]
        elif args[0] == "l1period":
            l1period = float( args[1])
            self.set_L1_period( l1period )
            print "CHEAT L1 Period - %s" % args[1]
        elif args[0] == "escrst":
            self.set_throttle_enable( False )
            self.reset_throttle_underway = True
            print "CHEAT ESCRST Started"
        elif args[0] == "mode":
            mode_req = args[1]
            self.execute_mode( mode_req )
            tempstr =  "CHEAT Mode - %s" % args[1]
            self.console.writeln( tempstr )
            print tempstr
        elif args[0] == "tweak":
            param_name = args[1]
            scalar = float( args[2] )
            self.tweak_parm( param_name, scalar )
        elif args[0] == "seq":
            if( args[1] == "preflight"):
                self.sequence_preflight = True
                tempstr =  "SEQ Preflight enabled"
            else:
                tempstr =  "Unknown sequence - %s", args[1]
            self.console.writeln( tempstr )
        else:
            tempstr = "Unknown Cheat %s" % args[0]
            print tempstr
            self.console.writeln( tempstr )



    def print_usage(self):
        print("usage: cheat [mode X] [wp x] [escrst] [thr x] [alt x] [l1 period] [speed X]")

def init(mpstate):
    '''initialise module'''
    return CheatModule(mpstate)
