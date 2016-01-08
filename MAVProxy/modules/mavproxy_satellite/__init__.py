#!/usr/bin/env python
'''
satellite c2 module for C-LAND
Samuel Dudley
July 2015
'''
import time


from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting
from MAVProxy.modules.lib import mp_settings
import GatewayWrapper as wrapper
import SkywaveIDP as IDP
import threading


class SatelliteState(object):
    def __init__(self):
        self.base_mode = 1
        self.targetIDPNumber = 1
        self.lat = 0
        self.lng = 0
        self.alt = 0
        
        self.create_gateway()
        self.start_gateway()
        self.start_rtd()
        #self.start_core() #currently buggy...
    
    def create_gateway(self):
        self.gateway = IDP.Skywave_IDP_Gateway()
        self.lock = threading.Lock()
        self.submitMsgList = wrapper.Thread_List()
        self.returnMsgRTDList = wrapper.Thread_List()
        self.returnMsgIDPList = wrapper.Thread_List()
    
    def start_gateway(self):
        self.gatewayThread = wrapper.Gateway_Thread(self.lock, self.gateway, self.submitMsgList, self.returnMsgRTDList, self.returnMsgIDPList)
    def start_rtd(self):
        self.RTDThread = wrapper.RTD_Thread(self.lock, self.gateway, self.submitMsgList, self.returnMsgRTDList, self.targetIDPNumber)
    def start_core(self):
        self.coreThread = wrapper.Core_Thread(self.lock, self.gateway, self.submitMsgList, self.returnMsgRTDList, self.targetIDPNumber)

class SatelliteCommunicationModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SatelliteCommunicationModule, self).__init__(mpstate, "Satellite", "Satellite commands")
        self.add_command("sat", self.cmd_satellite, "Satellite Communication Module")
        self.state = SatelliteState()
        self.__RegisterConnections()
        
        
        self.satellite_settings = mp_settings.MPSettings(
            [('gateway', bool, True), #set to False to kill the gateway, can be used as a reset...
             ('gateway_thread', bool, True),#set to False to stop gateway thread
             ('rtd_thread', bool, True),#set to False to stop RTD thread
             ('core_thread', bool, False),
             ('heartbeat_rate', float, 1.0), #rate to send module hearbeat in Hz
            ])
        
        self.heartbeat_period = mavutil.periodic_event(self.satellite_settings.heartbeat_rate) #send a heatbeat every 1Hz
        
    def cmd_satellite(self, args):
        '''Satellite Communication Module Commands'''
        usage = "usage: sat <set|status>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "status":
            print ""
        elif args[0] == "set":
            self.satellite_settings.command(args[1:])
        else:
            print(usage)
        
    def __RegisterConnections(self):
        '''register connections'''
        #this connection talks back to the master of the mavproxy instance
        self.mavproxy_connection = mavutil.mavudp("127.0.0.1:14500", input=False)
        self.mavproxy_link= mavutil.mavlink.MAVLink(self.mavproxy_connection)
        
        #this connection talks to the gateway via mavlink (this can be a stand in UDP app rather than the actual skywave gateway)
        self.gateway_connection = mavutil.mavudp("127.0.0.1:14600", input=True)
        self.gateway_link= mavutil.mavlink.MAVLink(self.gateway_connection)
        
    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        mtype = m.get_type()
        #print mtype
        
    def send_heartbeat(self):
        self.mavproxy_link.heartbeat_send(1,1,self.state.base_mode,0,4)

    def send_position(self):
        #time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg
        self.mavproxy_link.global_position_int_send(0, self.state.lat, self.state.lng,self.state.alt,0,0,0,0,0)
    
    def handle_mavproxy_msgs(self):
        now = time.time()
        if self.heartbeat_period.trigger():
            self.send_heartbeat()
        msg = self.mavproxy_connection.recv_msg()
        
        if msg is not None:
            #there is an incoming msg from mavproxy...
            msg_type = msg.get_type()
            if msg_type == 'PARAM_REQUEST_LIST':
                #we throw it away..
                pass
            elif msg_type == 'HEARTBEAT':
                #the gcs is still alive
                pass
            
            elif msg_type == 'SET_MODE':
                #the gcs wants to set the AP mode (e.g. RTL)
                #push the msg to the gateway
                buf = msg.get_msgbuf()
                self.gateway_connection.write(buf)
                #print msg_type, buf
            else:
                #print msg_type
                pass
                
            
    def handle_gateway_msgs(self):
        msg = self.gateway_connection.recv_msg()
        
        if msg is not None:
            #there is an incoming msg from the gateway...
            msg_type = msg.get_type()
            print msg_type, msg.to_dict()
            if msg_type == 'D100BYTE_SUMMARY':
                #we make bits of 'standard' messages from the 100byte msg and send them to mavproxy
                '''
                mode                      : Mode manual / auto etc (uint8_t)
                submode                   : Submode (uint8_t)
                wp                        : WP index (uint8_t)
                safe_lat                  : lat (int32_t)
                safe_lng                  : lng (int32_t)
                safe_alt                  : Alt (int32_t)
                '''
                msg_dict = msg.to_dict()
                
                #send a heartbeat with the newest mode...
                try:
                    self.state.base_mode = msg_dict['mode']
                    self.state.lat = msg_dict['safe_lat']
                    self.state.lng = msg_dict['safe_lng']
                except:
                    print 'error updating mode from 100 byte msg'
                self.send_heartbeat()
            
                self.send_position()
                
            if msg_type == 'HEARTBEAT':
                #the gateway is still alive
                pass
    
    def check_thread_health(self):
        
        try:
            skywave_status = self.state.gateway.is_alive()
            if not skywave_status:
                #the connection to the gateway has gone down...
                #try to recover...
                self.state.RTDThread.alive = False
                self.state.gatewayThread.alive = False
                self.state.coreThread.alive = False
                
                if self.satellite_settings.gateway:
                    self.state.create_gateway()
        
        except:
            print 'error restarting gateway...'
            
        self.state.gateway.alive = self.satellite_settings.gateway
        
        
        if self.satellite_settings.gateway:
            #dont try to restore the threads if the object is to be reset...
            try:
                RTD_status = self.state.RTDThread.is_alive()
                if not RTD_status:
                    #the RTD thread has fallen over...
                    #try to recover...
                    if self.satellite_settings.rtd_thread:
                        self.state.start_rtd()
                        
                self.state.RTDThread.alive = self.satellite_settings.rtd_thread
            
            except:
                if self.satellite_settings.rtd_thread:
                    print 'error checking rtd thread status... restarting...'
                    self.state.start_rtd()
                else:
                    print 'error checking rtd thread status.. not attempting restart'
                
            try:
                gateway_status = self.state.gatewayThread.is_alive()
                if not gateway_status:
                    #the gateway thread has fallen over...
                    #try to recover...
                    if self.satellite_settings.gateway_thread:
                        self.state.start_gateway()
                        
                self.state.gatewayThread.alive = self.satellite_settings.gateway_thread
            
            except:
                if self.satellite_settings.gateway_thread:
                    print 'error checking gateway thread status... restarting...'
                    self.state.start_gateway()
                else:
                    print 'error checking gateway thread status... not attempting restart'
                    
                
            try:
                core_status = self.state.coreThread.is_alive()
                if not core_status:
                    #the core thread has fallen over...
                    #try to recover...
                    if self.satellite_settings.core_thread:
                        self.state.start_core()
                        
                self.state.coreThread.alive = self.satellite_settings.core_thread
            
            except:
                if self.satellite_settings.core_thread:
                    print 'error checking core thread status... restarting...'
                    self.state.start_core()
                else:
                    pass
                    #print 'error checking core thread status.. not attempting restart'
            
            
    
       
    def idle_task(self):
        self.handle_mavproxy_msgs()
        self.handle_gateway_msgs()
        self.check_thread_health()
            
    def unload(self):
        '''unload module'''
        pass
        
def init(mpstate):
    '''initialise module'''
    return SatelliteCommunicationModule(mpstate)