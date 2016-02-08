'''
Support for C-LAND datalinks
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
import pysnmp



class MicrohardStatus(object):
    def __init__(self, queue, ip):
        
        self.queue = queue
        self.ip = ip
        self.alive = True
        
        self.status = {} #starts empty...
        
        from pysnmp.entity.rfc3413.oneliner import cmdgen
        from pysnmp.proto import rfc1902
        
        self.cmdGen = cmdgen.CommandGenerator()
 
        
        
        self.serial_number = '1.3.6.1.4.1.21703.100.7.2.1.0'
        self.rssi = '1.3.6.1.4.1.21703.100.7.2.5.0'
        self.tx_bytes = '1.3.6.1.4.1.21703.100.7.2.16.0'
        self.rx_bytes = '1.3.6.1.4.1.21703.100.7.2.8.0'
        self.voltage = '1.3.6.1.4.1.21703.100.7.2.4.0'
        self.temp = '1.3.6.1.4.1.21703.100.7.2.3.0'

        #Transmit bytes: '1.3.6.1.4.1.21703.100.7.2.16.0'
        #Transmit packets: 1.3.6.1.4.1.21703.100.7.2.17.0
        #serial number: 1.3.6.1.4.1.21703.100.7.2.1.0
        #Version: '1.3.6.1.4.1.21703.100.7.2.2.0'
        #temp deg C: 1.3.6.1.4.1.21703.100.7.2.3.0
        #voltage V:1.3.6.1.4.1.21703.100.7.2.4.0
        #rssi: '1.3.6.1.4.1.21703.100.7.2.5.0
        #Receive bytes:  1.3.6.1.4.1.21703.100.7.2.8.0
        
        #ethernet rec bytes: 1.3.6.1.4.1.21703.100.7.1.1.0
        
        #prim freq table first entry '1.3.6.1.4.1.21703.100.3.22.1.0'
        #prim freq table last entry '1.3.6.1.4.1.21703.100.3.22.50.0'
        
        self.thread = threading.Thread(target=self.main)
        self.thread.daemon = True
        self.thread.start()
        
    def get_snmp_value(self, ip, target):
        from pysnmp.entity.rfc3413.oneliner import cmdgen
        
        
        errorIndication, errorStatus, errorIndex, varBinds = self.cmdGen.getCmd(
            cmdgen.CommunityData('public'),
            cmdgen.UdpTransportTarget((ip,161),timeout=1,retries=0),
            target)

        # Check for errors and print out results
        
        if errorStatus:
            print('%s at %s' % (
                errorStatus.prettyPrint(),
                errorIndex and varBinds[int(errorIndex)-1][0] or '?'
                )
            )
            output = -1
            return output
        else:
            for name, val in varBinds:
                output = float(val.prettyPrint())
                return output
    
    def get_usec(self):
        '''time since 1970 in microseconds'''
        return int(time.time() * 1.0e6)
    
    def is_alive(self):
        return self.alive
            
    def main(self):
        while self.is_alive():
            
            self.status['gcs_time'] = self.get_usec()
            
            self.status['rssi'] = self.get_snmp_value(self.ip, self.rssi)
            if self.status['rssi'] == None:
                self.status['rssi'] = -1
                
            self.status['tx_bytes'] = self.get_snmp_value(self.ip, self.tx_bytes)
            if self.status['tx_bytes'] == None:
                self.status['tx_bytes'] = -1
                
            self.status['rx_bytes'] = self.get_snmp_value(self.ip, self.rx_bytes)
            if self.status['rx_bytes'] == None:
                self.status['rx_bytes'] = -1
            
            self.queue.put_nowait(status)
                
class Datalink(object):
    '''a generic datalink object'''

    def __init__(self, ip):
        self.queue = Queue.Queue()
        self.ip = ip
        self.state = {}
        # state holds a dict of the values useful to this datalink...
        # RSSI, voltage, temp, etc...

        self.update_time = time.time()

    def update(self, state):
        '''update the datalink state'''
        self.state = state
        self.update_time = time.time()


class DatalinkModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(DatalinkModule, self).__init__(mpstate, "datalink", "datalink support")
        self.add_command('datalink', self.cmd_datalink, ["datalink control",
                                                 "<status>",
                                                 "set (DATALINKSETTING)"])

        self.datalink_settings = mp_settings.MPSettings([("query_hz", float, 2.0)])
        self.datalinks = {}
        
        
        self.datalinks['microhard1320T'] = Datalink(ip ='100.100.100.82')
        
        
        #this connection talks back to the master of the mavproxy instance
        self.mavproxy_connection = mavutil.mavudp("127.0.0.1:14500", input=False)
        self.mavproxy_link= mavutil.mavlink.MAVLink(self.mavproxy_connection)
        self.query_timer = mavutil.periodic_event(self.datalink_settings.query_hz)
        
    def cmd_datalink(self, args):
        '''datalink command parser'''
        usage = "usage: datalink <set>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "status":
            for datalink_id in self.datalinks.keys():
                print datalink_id, self.datalinks[datalink_id].ip
                print self.datalinks[datalink_id].status
                print ""
                
                
        elif args[0] == "set":
            self.datalink_settings.command(args[1:])
            self.update_query_hz()
        else:
            print(usage)
            
    def update_query_hz(self):
        self.query_timer = mavutil.periodic_event(self.datalink_settings.query_hz)
        
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        pass
    
    def idle_task(self):
        '''called on idle'''
        if self.query_timer.trigger():
            for datalink_id in self.datalinks.keys():
                #print datalink_id, self.datalinks[datalink_id].ip
                while not self.datalinks[datalink_id].queue.empty():
                    status = self.datalinks[datalink_id].queue.get_nowait()
                    print status
                    self.mavproxy_link.d100byte_summary_send(0,0,0,0,0,0,0,0,0)

def init(mpstate):
    '''initialise module'''
    return DatalinkModule(mpstate)
