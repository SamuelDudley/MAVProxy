'''
Support for switching datalink antennas
Samuel Dudley
Feb 2016
'''

import time
from math import *

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules import mavproxy_link
from pymavlink import mavutil
import threading
import Queue

import serial


class RFSwitchStatus(object):
    def __init__(self, dev, queue):
        
        self.queue = queue
        self.dev = dev
        self.alive = True
        
        self.status = {} #starts empty...
        
        self.thread = threading.Thread(target=self.main)
        self.thread.daemon = True
        self.thread.start()
    
    def get_usec(self):
        '''time since 1970 in microseconds'''
        return int(time.time() * 1.0e6)
    
    def get_rf_switch_status(self):
        pass
    
    def set_rf_switch_status(self):
        pass
    
    def is_alive(self):
        return self.alive
            
    def main(self):
        while self.is_alive():
            while not self.queue.empty():
                try:
                    with serial.Serial(self.dev, 1200, timeout=3,
                                  dsrdtr=False, rtscts=False, xonxoff=False) as ser:
                        # we rather strangely set the baudrate initially to 1200, then change to the desired
                        # baudrate. This works around a kernel bug on some Linux kernels where the baudrate
                        # is not set correctly
                        ser.setBaudRate(19200)
                        status = self.queue.get_nowait()
                        ser.write(status['rf_output'])
                        resp = ser.readline()
                        print "swich port", resp
#                         self.queue.put_nowait({'rf_output':resp})
                except Exception as err:
                    print err
                    
            time.sleep(0.1)
                

class RFSwitchModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(RFSwitchModule, self).__init__(mpstate, "switch", "rf switch support")
        self.add_command('switch', self.cmd_rf_switch, ["rf switch control",
                                                 "<status>",
                                                 "set (DATALINKSETTING)"])

        self.switch_settings = mp_settings.MPSettings([("query_hz", float, 2.0),
                                                       ("port", int, 1)])
        self.queue = Queue.Queue()
        self.switches = {}
        
        
        self.switches['microhard_switch'] = RFSwitchStatus(dev ='/dev/rf_switch', queue = self.queue)
        
        
        #this connection talks back to the master of the mavproxy instance
        self.mavproxy_connection = mavutil.mavudp("127.0.0.1:14500", input=False)
        self.mavproxy_link= mavutil.mavlink.MAVLink(self.mavproxy_connection)
        self.query_timer = mavutil.periodic_event(self.switch_settings.query_hz)
        
    def cmd_rf_switch(self, args):
        '''switch command parser'''
        usage = "usage: switch <set>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "status":
            for switch_id in self.switches.keys():
                print switch_id, self.switches[switch_id].dev
                print self.switches[switch_id].status
                print ""
                
                
        elif args[0] == "set":
            self.switch_settings.command(args[1:])

            self.update_query_hz()
            self.update_port()
        else:
            print(usage)
            
    def update_query_hz(self):
        if self.switch_settings.query_hz <= 0:
            self.switch_settings.query_hz = 0.01
        self.query_timer = mavutil.periodic_event(self.switch_settings.query_hz)
    
    def update_port(self):
        self.queue.put_nowait({'rf_output':str(self.switch_settings.port)})
    
    def idle_task(self):
        '''called on idle'''
        if self.query_timer.trigger():
            for switch_id in self.switches.keys():
                pass
#                 while not self.switches[switch_id].queue.empty():
#                     status = self.switches[switch_id].queue.get_nowait()
#                     print status
#                     port_number = int(status['rf_output'])
#                     self.switch_settings.port = port_number
#                     self.mpstate.console.set_status('switch', 'ANT: %u' % (int(self.switch_settings.port)),
#                                              fg='green', row=1)
#                     
                        
#                     pass

def init(mpstate):
    '''initialise module'''
    return RFSwitchModule(mpstate)
