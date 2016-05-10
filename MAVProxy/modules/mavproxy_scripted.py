#!/usr/bin/env python
'''auto testing suite'''

import time, math, sys, cmd
from threading import Thread

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import wxconsole
import multiprocessing, threading
from MAVProxy.modules.lib.wxconsole_util import Value, Text
from MAVProxy.modules.lib.wxconsole import MessageConsole
from MAVProxy.modules.mavproxy_map import mp_elevation
from MAVProxy.modules.mavproxy_console import ConsoleModule
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import wxsettings
from MAVProxy.modules.lib import textconsole
from MAVProxy.modules.lib.mp_menu import *


class TestingMessageConsole(MessageConsole):
    '''
    a message console for Testing that duplicates output to file
    '''
    def __init__(self,out_fname):
        MessageConsole.__init__(self)
        self.out_file=None
        try:
            self.out_file = open(out_fname, 'w')
        except Exception :
            print "Couldn't open output file "+out_fname

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        self.parent_pipe_send.close()
        self.parent_pipe_recv.close()
         
        import MAVProxy.modules.lib.wx_processguard
        from MAVProxy.modules.lib.wx_loader import wx
        from MAVProxy.modules.lib.wxconsole_ui import ConsoleFrame
        app = wx.App(False)
        app.frame = ConsoleFrame(state=self, title=self.title)
        app.frame.SetDoubleBuffered(True)
        app.frame.SetFocus()
        app.frame.SetWindowStyle(wx.STAY_ON_TOP)
        app.MainLoop()

    def write(self, text, fg='black', bg='white'):
        '''write to the console'''
        try:
            self.parent_pipe_send.send(Text(text, fg, bg))
            if self.out_file != None :
                print text
                if isinstance(text, str):
                    self.out_file.write(text)
                else:
                    self.out_file.write(str(text))
                self.out_file.flush()
        except Exception:
            pass

    def close(self):
        '''close the console'''
        MessageConsole.close(self)
        if self.out_file != None :
            self.out_file.close()
        

class TestCommand:

    def __init__(self,cmd,delay,description,use_wall_clock=False):
        self.cmd=cmd
        self.delay=delay
        self.use_wall_clock=use_wall_clock
        self.description=description

class TestCase:        
    def __init__(self,testCommands):
        self.testCommands=testCommands
    
class TestThread(Thread):
 
    def __init__(self,mp):
        ''' Constructor. '''
        Thread.__init__(self)
        self.mp=mp
        self.test_case=None
        self.sim_speed=0
 
    def load_test_case(self,test_case,sim_speed):
        self.test_case=test_case
        self.sim_speed=sim_speed
 
    def run(self):
        print "Running test"
        if self.test_case is None:
            print "No test case loaded."
            return
        i=0
        for c in self.test_case.testCommands :
            output= "### SCRIPTED COMMAND > "+c.cmd+" ("+c.description +")"
            self.mp.say(output)
            self.mp.mpstate.functions.process_stdin(c.cmd)
            sim_sleep_t=c.delay/self.sim_speed
            if c.use_wall_clock :
                print "sleeping "+str(c.delay)+" seconds"
                time.sleep(c.delay)
            else :
                print "sleeping "+str(c.delay)+" seconds ("+str(sim_sleep_t)+"s sim time)"
                time.sleep(sim_sleep_t)
            i+=1



class ScriptedModule(ConsoleModule):
    def __init__(self, mpstate):
        mp_module.MPModule.__init__(self,mpstate, "testing console", "Testing GUI console", public=True)
        self.set_console()
        self.temp_mission_fname="/tmp/test_mission.txt"
        self.landing_commence_wp=100
        self.current_wp=0
        self.current_test_cmd=0
        self.last_clock_time_us=0.0
        self.last_wall_time_s=0.0
        self.clock_inited=False
        self.sim_speed=0
        self.add_command('scripted', self.cmd_scripted,
                         "scripted",
                         [])     
        
    def set_console(self) :
        self.in_air = False
        self.start_time = 0.0
        self.total_time = 0.0
        self.speed = 0
        self.max_link_num = 0
        self.last_sys_status_health = 0
        #self.mpstate.console = wxconsole.MessageConsole(title='Testing Console')
        time_string=time.strftime("%Y%m%d-%H%M%S")
        self.mpstate.console=TestingMessageConsole("/tmp/testing_"+time_string+".txt")
        self.mpstate.console.set_status('Mode', 'UNKNOWN', row=0, fg='blue')
        self.mpstate.console.set_status('ARM', 'ARM', fg='grey', row=0)
        self.mpstate.console.set_status('GPS', 'GPS: --', fg='red', row=0)
        self.mpstate.console.set_status('Vcc', 'Vcc: --', fg='red', row=0)
        self.mpstate.console.set_status('Radio', 'Radio: --', row=0)
        self.mpstate.console.set_status('INS', 'INS', fg='grey', row=0)
        self.mpstate.console.set_status('MAG', 'MAG', fg='grey', row=0)
        self.mpstate.console.set_status('AS', 'AS', fg='grey', row=0)
        self.mpstate.console.set_status('RNG', 'RNG', fg='grey', row=0)
        self.mpstate.console.set_status('AHRS', 'AHRS', fg='grey', row=0)
        self.mpstate.console.set_status('EKF', 'EKF', fg='grey', row=0)
        self.mpstate.console.set_status('Heading', 'Hdg ---/---', row=2)
        self.mpstate.console.set_status('Alt', 'Alt ---', row=2)
        self.mpstate.console.set_status('AGL', 'AGL ---/---', row=2)
        self.mpstate.console.set_status('AirSpeed', 'AirSpeed --', row=2)
        self.mpstate.console.set_status('GPSSpeed', 'GPSSpeed --', row=2)
        self.mpstate.console.set_status('Thr', 'Thr ---', row=2)
        self.mpstate.console.set_status('Roll', 'Roll ---', row=2)
        self.mpstate.console.set_status('Pitch', 'Pitch ---', row=2)
        self.mpstate.console.set_status('Wind', 'Wind ---/---', row=2)
        self.mpstate.console.set_status('WP', 'WP --', row=3)
        self.mpstate.console.set_status('WPDist', 'Distance ---', row=3)
        self.mpstate.console.set_status('WPBearing', 'Bearing ---', row=3)
        self.mpstate.console.set_status('AltError', 'AltError --', row=3)
        self.mpstate.console.set_status('AspdError', 'AspdError --', row=3)
        self.mpstate.console.set_status('FlightTime', 'FlightTime --', row=3)
        self.mpstate.console.set_status('ETR', 'ETR --', row=3)
        self.mpstate.console.ElevationMap = mp_elevation.ElevationModel()
        
        # create the main menu
        if mp_util.has_wxpython:
            self.menu = MPMenuTop([])
            self.add_menu(MPMenuSubMenu('MAVProxy',
                                        items=[MPMenuItem('Settings', 'Settings', 'menuSettings'),
                                               MPMenuItem('Map', 'Load Map', '# module load map')]))
    
    def run_cmd_thread(self,test_case): 
        cmd_thread = TestThread(self)
        cmd_thread.load_test_case(test_case,self.sim_speed)
        cmd_thread.start()
        
                          
    def cmd_scripted(self, args):
        '''scripted command'''
        if len(args) < 1:
            self.print_usage()
            return
        
        id=int(args[0])
        
        if id>4 or id < 1:
            print "Invalid test case ("+args[0]+")!"
            return;
        
        testCommands=[]
        
        self.write_test_mission()
        
        if id==1 :
            self.say("#############################################################################################")
            self.say("Testing car rooftop launch with Takeoff Release switch disabled (i.e. plane NOT RELEASED)...")
            self.say("#############################################################################################")
            time.sleep(1)
            testCommands.append(TestCommand("wp load "+self.temp_mission_fname,5,"Load test mission.",True))
            testCommands.append(TestCommand('manual',1,"Set to Manual mode"))
            testCommands.append(TestCommand('wp set 1',2,"Set to takeoff"))
            testCommands.append(TestCommand('arm throttle',2,"Arm the throttle"))
            testCommands.append(TestCommand('rc 3 1500',10,"Hit med RC throttle input for 10 seconds"))
            testCommands.append(TestCommand('auto',1,"Set to AUTO mode"))
        
            
        elif id==2 :
            self.say("#####################################################################################")
            self.say("Testing car rooftop launch with Takeoff release switch enabled (i.e. plane RELEASED)")
            self.say("#####################################################################################")
            time.sleep(1)
            testCommands.append(TestCommand("wp load "+self.temp_mission_fname,5,"Load test mission.",True))
            testCommands.append(TestCommand('fbwa',1,"Set to FBWA mode"))
            testCommands.append(TestCommand('wp set 1',2,"Set to takeoff"))
            testCommands.append(TestCommand('arm throttle',2,"Arm the throttle"))
            testCommands.append(TestCommand('rc 3 2200',12,"Hit high RC throttle input for 12 seconds"))
            testCommands.append(TestCommand('auto',0,"Set to AUTO mode"))
            #testCommands.append(TestCommand('rc 3 0',1,"Remove RC throttle input"))
            testCommands.append(TestCommand('rc 7 1800',0,"Enable the takeoff release switch via RC"))
            #testCommands.append(TestCommand('rc 7 0',1,"Zero the takeoff release RC input"))           
        
        elif id==3 :
            self.say("############################################")
            self.say("Testing MANUAL throttle kill on landing...")
            self.say("############################################")
            time.sleep(1)
            self.current_test_cmd=3
            testCommands.append(TestCommand("param set THR_KILL_AUTO 0",0,"Disable AUTO throttle kill"))
            testCommands.append(TestCommand("wp load "+self.temp_mission_fname,5,"Load test mission.",True))
            testCommands.append(TestCommand('wp set 1',1,"Set to takeoff"))
            testCommands.append(TestCommand('arm throttle',1,"Arm the throttle"))
            testCommands.append(TestCommand('rc 7 1800',1,"Enable the takeoff release switch via RC"))
            testCommands.append(TestCommand('auto',1,"Set to AUTO mode"))
            
        elif id==4 :
            self.say("#########################################")
            self.say("Testing AUTO throttle kill on landing...")
            self.say("#########################################")
            time.sleep(5)
            self.current_test_cmd=4
              
        else :
            self.say("invalid test")
        
        testCase=TestCase(testCommands)
        self.run_cmd_thread(testCase)

#   
    def do_landing_test(self):
        testCommands=[]
        if self.current_test_cmd==3:
            # Test manual throttle kill
            testCommands.append(TestCommand('',6,'Wait for 6 seconds'))
            testCommands.append(TestCommand('fbwb',1,"Set to FBWB mode"))
            testCommands.append(TestCommand('rc 6 1800',0,"Kill the throttle"))
            
        else :
            return
        
        testCase=TestCase(testCommands)
        self.run_cmd_thread(testCase)
            
      
    def print_usage(self):
        print("usage: scripted <test-case-ID>")
        
    def write_test_mission(self):
        print "Opening temp mission file..."
        self.landing_commence_wp=4
        f = open(self.temp_mission_fname, 'w')
        f.write("QGC WPL 110\n")
        f.write("0    0    0    16    0.000000    0.000000    0.000000    0.000000    -30.931446    136.544815    138.380005    1\n")
        f.write("1    0    3    22    0.000000    0.000000    0.000000    0.000000    -30.926538    136.536591    102.139999    1\n")
        #f.write("2    0    3    18    1.000000    0.000000    0.000000    0.000000    -30.921862    136.547165    250.000000    1\n")
        f.write("3    0    3    16    0.000000    0.000000    0.000000    0.000000    -30.913450    136.552414    250.000000    1\n")
        f.write("4    0    3    16    0.000000    0.000000    0.000000    0.000000    -30.913668    136.544159    150.070007    1\n")
        f.write("5    0    3    21    0.000000    0.000000    0.000000    0.000000    -30.930006    136.544678    0.000000    1\n")
        f.close()

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        super(ScriptedModule,self).mavlink_packet(m)
        mtype = m.get_type()
        if mtype in ["WAYPOINT_CURRENT", "MISSION_CURRENT"]:
            if m.seq == self.landing_commence_wp and self.current_wp != m.seq:
                self.current_wp=m.seq
                print "COMMENCING LANDING..."
                self.say("Landing commencing %u" % m.seq,priority='message')
                self.do_landing_test()
        if mtype == 'GPS_RAW_INT':
            current_clock_time_us=m.time_usec
            current_wall_time_s=time.time()
            if  self.clock_inited==False :
                if current_clock_time_us>0.0 :
                    self.last_clock_time_us=current_clock_time_us
                    self.last_wall_time_s=current_wall_time_s
                    #print "got initial times."
                    #print "clock time "+str(current_clock_time_us)
                    #print "wall time "+str(current_wall_time_s)
                    self.clock_inited=True
            else:
                clock_time_diff_us=current_clock_time_us-self.last_clock_time_us
                wall_time_diff_us=10.0**6*(current_wall_time_s-self.last_wall_time_s)
                current_sim_speed=clock_time_diff_us/wall_time_diff_us
                # calculate a moving average of sim speed over last 10 samples...
                if self.sim_speed==0.0 :
                    self.sim_speed=current_sim_speed
                else:
                    average_sim_speed=self.sim_speed-self.sim_speed/10.0
                    average_sim_speed=average_sim_speed+current_sim_speed/10.0
                    self.sim_speed=average_sim_speed
                #print "clock time diff us "+str(clock_time_diff_us)
                #print "wall time diff us "+str(wall_time_diff_us)
                #print "sim speed = "+str(self.sim_speed)
                self.last_clock_time_us=current_clock_time_us
                self.last_wall_time_s=current_wall_time_s
                


def init(mpstate):
    '''initialise module'''
    return ScriptedModule(mpstate)