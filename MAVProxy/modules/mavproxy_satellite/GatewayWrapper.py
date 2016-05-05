#needs three threads...
#main thread calls and manages the gateway instance
#second thread handles messages coming in from serial source (UDP)
#third thread handles the mavlink interface of the idp hardware directly
import SkywaveIDP
import threading
from pymavlink import mavutil
import time
import base64
import math


 

class Gateway_Thread(threading.Thread):
    def __init__(self, lock, gateway, submitMsgList, returnMsgRTDList, returnMsgIDPList):
        threading.Thread.__init__(self)
        self.daemon = True
        self.alive = True
        
        self.gateway = gateway #create local shared gateway instance
        self.lock = lock
        
        self.submitMsgList = submitMsgList
        self.returnMsgRTDList = returnMsgRTDList
        self.returnMsgIDPList = returnMsgIDPList

        self.timeDelta = None#900000#
        
        self.start()
         
    def is_alive(self):
        return self.alive
        
    def run(self):
        while self.alive:
            with self.lock: 
                pass
                self.submitMsgList._list = self.gateway.Submit_Forward_Message_To_Gateway(self.submitMsgList._list)
            #check to see if any messages have not been submitted... if any submit them NOW.
            try:
                self.fid1.write(str(time.time())+" "+str(len(submitMsgList)))
                #print submitMsgList._list[-1].History[-1])
            except:
                pass
            time.sleep(0.5)
            returnMessageList = self.gateway.Get_Return_Message_List(self.timeDelta) #no args, so set time delta to start of program
            
            if returnMessageList:
                #if the returnMessageList did not throw an error and return false... then...
            
                with self.lock:
                    self.gateway.Query_List_Of_Forward_Message_Status(self.submitMsgList._list)
                CoreMessages, RTDMessages = self.Sort_Return_Messages(returnMessageList)
                
                if len(RTDMessages) !=0:
                    with self.lock:
                        for msg in RTDMessages:
                            self.returnMsgRTDList._list.append(msg)
                            
                if len(CoreMessages) !=0:
                    with self.lock:
                        for msg in CoreMessages:
                            self.returnMsgIDPList._list.append(msg)
    
    
                if self.gateway.Return_Message_Filter_High_Tide != None:
                    self.timeDelta = self.gateway.Get_Time_Difference_To_Now(self.gateway.Return_Message_Filter_High_Tide, serverTime = True)
                
            print "Gateway high tide: ",self.timeDelta
                
                
            print "_____________________________________________________________"

    
    def Sort_Return_Messages(self,messageList):
        #check SINS to sort types
        CoreMessages = [x for x in messageList if x.SIN != 28]
        RTDMessages = [x for x in messageList if x.SIN == 28]

        return CoreMessages, RTDMessages
    
    
    
class RTD_Thread(threading.Thread):
    def __init__(self, lock, gateway, submitMsgList, returnMsgRTDList, targetIDPNumber):
        threading.Thread.__init__(self)
        self.daemon = True
        self.alive = True
        
        self.gateway = gateway #create local shared gateway instance
        self.lock = lock
        self.submitMsgList = submitMsgList
        self.returnMsgRTDList = returnMsgRTDList

        self.connectionRTD = mavutil.mavudp("127.0.0.1:14600", input=False) #create connection for RTD binary data
        self.mav = mavutil.mavlink.MAVLink(self.connectionRTD)
        self.RTDServiceStatus = {"timeSinceRTDData": time.time(), "localUpdateTime": time.time(), "channelStatus": None, "DTEConnected": None, "reportTime": None, "bytesGatewayToRS232": None, "bytesRS232ToGateway": None,
                                  "tOverrun": None, "reportCause": None, "serviceStatus": None, "rOverrun": None}
        self.buf = ''
        self.log = str(int(time.time()))+"_log.txt"
        self.sendFlag = True #not sending at this stage
        self.targetIDPNumber = targetIDPNumber
        self.start()
        
    def is_alive(self):
        return self.alive
    
    def run(self):
        #wait for time slot in gateway thread
        while self.alive:
#             self.Receive_Bytes()
            try:
                self.Parse_RTD_Messages()
            except:
                print 'error parsing RTD messages...'
#             #print 'self.buf:', self.buf
            if self.sendFlag == True:
                #self.Request_Keep_Alive()
                self.Receive_Bytes()
                payload = self.Get_Buffer()
                if payload != '':
                    print payload
                    message = self.gateway.Generate_RTD_Message(payload,self.targetIDPNumber) #create message
                    with self.lock:
                        self.submitMsgList._list.append(message)
#                     #self.sendFlag = False
            time.sleep(0.1)
            
    def Parse_RTD_Messages(self):
        with self.lock:
            messageList = self.returnMsgRTDList._list
            self.returnMsgRTDList._list = []
        
        for msg in messageList:
            msgDict ={}
            for Field in msg.Payload.Fields[0]:
                try:
                    msgDict[Field._Name]=Field._Value
                except:
                    pass
            try:
                if msg.Payload._Name == "RS232data":
                    if 'payload' in msgDict:
                        bytesToDecode = msgDict['payload']
                        decodedBytes = self.Decode_Binary_Payload(bytesToDecode)
                        print "GOT "+str(len(decodedBytes))+" BYTES OF RTD DATA...", 
#                         for byte in decodedBytes:
#                             print ord(byte)
#                             
#                         print ""
                        print "time since last RTD data: ", time.time() - self.RTDServiceStatus["timeSinceRTDData"]
                        
                        write_log = open(self.log, 'a')
                        write_log.write(str(time.time())+" "+str(time.time() - self.RTDServiceStatus["timeSinceRTDData"])+" "+str(len(decodedBytes))+"\n")
                        write_log.close()
                        
                        self.RTDServiceStatus["timeSinceRTDData"] = time.time()
                         
                        self.Send_Bytes(decodedBytes)
                
                elif msg.Payload._Name == "statusReportRS232":
                    self.Update_RTD_Service_Status(msgDict)
                
                else:
                    print msg.Payload._Name
                    print msgDict
                
            except:
                print "Error decoding RTD Message"

    def Update_RTD_Service_Status(self, msgDict):
        for key in msgDict:
            try:
                self.RTDServiceStatus[key]=msgDict[key]
                self.RTDServiceStatus['localUpdateTime'] = time.time()
            except:
                print 'Error updating RTD Service Status: '+str(key)
        print self.RTDServiceStatus
#         
#    '''this is an old DSTOlink fn... replace'''
#     def Request_Keep_Alive(self):
#         """sends a keep alive / heart beat out... If there is a GCS to respond one will be sent back"""
#         m = self.mav.keep_alive_encode(0)
#         self.Send_Message(m)
        
    def Send_Bytes(self, buf):
        self.connectionRTD.write(buf)
        
    def Send_Message(self, message): #message must be encoded prior to sending via DSTOlink
        self.mav.send(message)
            
        
    def Encode_Binary_Payload(self, payload):
        encodedData = base64.b64encode(payload)
        return encodedData
    
    def Decode_Binary_Payload(self, encodedData):
        payload = base64.b64decode(encodedData)
        return payload
        
    def Get_Buffer(self):
        _buffer = self.buf
        with self.lock:
            self.buf = ''
        return _buffer
    
    def Receive_Bytes(self):
        msg = self.connectionRTD.recv_msg()
        if msg is not None:
            msg_type = msg.get_type()
<<<<<<< HEAD
            #if msg_type == 'SET_MODE':
                #the gcs wants to set the AP mode (e.g. RTL)
                #push the msg to the gateway
        
            self.buf = msg.get_msgbuf()
=======
            if msg_type == 'SET_MODE':
                #the gcs wants to set the AP mode (e.g. RTL)
                #push the msg to the gateway
                self.buf = msg.get_msgbuf()
>>>>>>> branch 'cland1' of /home/uas/Dropbox/Shared_Git/MAVProxy.git
            
#         n = 16*1024#None#16*1024
#         bufLength = len(self.buf)
#         while True:
#             try:
#                 self.buf += self.connectionRTD.recv(n)
#             except Exception:
#                 return
#             if len(self.buf) == bufLength:
#                 return
#             else:
#                 bufLength = len(self.buf)
    

class Core_Thread(threading.Thread):
    def __init__(self, lock, gateway, submitMsgList, returnMsgIDPList, targetIDPNumber):
        threading.Thread.__init__(self)
        self.daemon = True
        self.targetIDPNumber = targetIDPNumber
        self.gateway = gateway #create local shared gateway instance
        self.lock = lock
        
        self.submitMsgList = submitMsgList
        self.returnMsgIDPList = returnMsgIDPList
        
        self.IDPState = {'localUpdateTime': None, 'base_mode':1, 'custom_mode':16,
                         'altitude':None, 'heading':None, 'speed':None, 'latitude':None, 'longitude':None, 'geofence1Status':None, 'geofence2Status':None}
        self.startTime = time.time()
        self.alive = True
        self.start()

    def is_alive(self):
        return self.alive
    
    def run(self):
        
        
        while self.alive:
            
            systemMessages, IDPStatusMessages, shellMessages = self.Sort_Core_Messages()
            self.Parse_System_Messages(systemMessages)
            #self.Parse_Shell_Messages(shellMessages)
            msgDict = self.Parse_IDP_Status_Messages(IDPStatusMessages)
            
           
            
            
            
            if msgDict: #is not none and therefore the msgDict contains updates...
                try:
                    timeSinceIDPUpdate = int(time.time() - self.IDPState['localUpdateTime'])
                except:
                    timeSinceIDPUpdate = 'NO INFO'
                
                self.Update_IDP_Report_Values(msgDict)
    

            time.sleep(0.1) 
            
    def Calculate_Distance(self, lat1, lon1, lat2, lon2):
        f = 1.0 / 298.257223563        # WGS84
        a = 6378137.0 

        phi1 = lat1
        lembda1 = lon1

        phi2 = lat2
        lembda2 = lon2
        
        """ 
        Returns the distance between two geographic points on the ellipsoid
        and the forward and reverse azimuths between these points.
        lats, longs and azimuths are in decimal degrees, distance in metres 

        Returns ( s, alpha12,  alpha21 ) as a tuple
        
        
        phi1 = math.degrees(phi1)
        phi2 = math.degrees(phi2)
        lembda1 = math.degrees(lembda1)
        lembda2 = math.degrees(lembda2)
        """
        
        if (abs( phi2 - phi1 ) < 1e-8) and ( abs( lembda2 - lembda1) < 1e-8 ) :
                return 0.0, 0.0, 0.0

        piD4   = math.atan( 1.0 )
        two_pi = piD4 * 8.0

        phi1    = phi1 * piD4 / 45.0
        lembda1 = lembda1 * piD4 / 45.0        # unfortunately lambda is a key word!
        phi2    = phi2 * piD4 / 45.0
        lembda2 = lembda2 * piD4 / 45.0

        b = a * (1.0 - f)

        TanU1 = (1-f) * math.tan( phi1 )
        TanU2 = (1-f) * math.tan( phi2 )

        U1 = math.atan(TanU1)
        U2 = math.atan(TanU2)

        lembda = lembda2 - lembda1
        last_lembda = -4000000.0        # an impossibe value
        omega = lembda

        # Iterate the following equations, 
        #  until there is no significant change in lembda 

        while ( last_lembda < -3000000.0 or lembda != 0 and abs( (last_lembda - lembda)/lembda) > 1.0e-9 ) :

                sqr_sin_sigma = pow( math.cos(U2) * math.sin(lembda), 2) + \
                        pow( (math.cos(U1) * math.sin(U2) - \
                        math.sin(U1) *  math.cos(U2) * math.cos(lembda) ), 2 )

                Sin_sigma = math.sqrt( sqr_sin_sigma )

                Cos_sigma = math.sin(U1) * math.sin(U2) + math.cos(U1) * math.cos(U2) * math.cos(lembda)
        
                sigma = math.atan2( Sin_sigma, Cos_sigma )

                Sin_alpha = math.cos(U1) * math.cos(U2) * math.sin(lembda) / math.sin(sigma)
                alpha = math.asin( Sin_alpha )

                Cos2sigma_m = math.cos(sigma) - (2 * math.sin(U1) * math.sin(U2) / pow(math.cos(alpha), 2) )

                C = (f/16) * pow(math.cos(alpha), 2) * (4 + f * (4 - 3 * pow(math.cos(alpha), 2)))

                last_lembda = lembda

                lembda = omega + (1-C) * f * math.sin(alpha) * (sigma + C * math.sin(sigma) * \
                        (Cos2sigma_m + C * math.cos(sigma) * (-1 + 2 * pow(Cos2sigma_m, 2) )))

        u2 = pow(math.cos(alpha),2) * (a*a-b*b) / (b*b)

        A = 1 + (u2/16384) * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))

        B = (u2/1024) * (256 + u2 * (-128+ u2 * (74 - 47 * u2)))

        delta_sigma = B * Sin_sigma * (Cos2sigma_m + (B/4) * \
                (Cos_sigma * (-1 + 2 * pow(Cos2sigma_m, 2) ) - \
                (B/6) * Cos2sigma_m * (-3 + 4 * sqr_sin_sigma) * \
                (-3 + 4 * pow(Cos2sigma_m,2 ) )))

        s = b * A * (sigma - delta_sigma)

        alpha12 = math.atan2( (math.cos(U2) * math.sin(lembda)), \
                (math.cos(U1) * math.sin(U2) - math.sin(U1) * math.cos(U2) * math.cos(lembda)))

        alpha21 = math.atan2( (math.cos(U1) * math.sin(lembda)), \
                (-math.sin(U1) * math.cos(U2) + math.cos(U1) * math.sin(U2) * math.cos(lembda)))

        if ( alpha12 < 0.0 ) : 
                alpha12 =  alpha12 + two_pi
        if ( alpha12 > two_pi ) : 
                alpha12 = alpha12 - two_pi

        alpha21 = alpha21 + two_pi / 2.0
        if ( alpha21 < 0.0 ) : 
                alpha21 = alpha21 + two_pi
        if ( alpha21 > two_pi ) : 
                alpha21 = alpha21 - two_pi

        alpha12    = alpha12    * 45.0 / piD4
        alpha21    = alpha21    * 45.0 / piD4
        return s, alpha12#,  alpha21 

        # END of Vincenty's Inverse formulae
            
            
    def Sort_Core_Messages(self):
        #check SINS to sort types
        with self.lock:
            messageList = self.returnMsgIDPList._list
            self.returnMsgIDPList._list = [] 
        
        systemMessages = [x for x in messageList if x.SIN == 0]
        IDPStatusMessages = [x for x in messageList if x.SIN == 128]
        shellMessages = []#[x for x in messageList if x.SIN == 26]

        return systemMessages, IDPStatusMessages, shellMessages


    def Parse_System_Messages(self, messageList):
        for msg in messageList:
            msgDict ={}
            for Field in msg.Payload.Fields[0]:
                msgDict[Field._Name]=Field._Value            
            print msg.Payload._Name
            print msgDict
            print ""
            
    def Parse_Shell_Messages(self, messageList):
        for msg in messageList:
            msgDict ={}
            for Field in msg.Payload.Fields[0]:
                msgDict[Field._Name]=Field._Value
                print msgDict
                print msg.Payload._Name
                print msgDict['success']
                if msgDict['success'] == True: ##TO DO add try statement to handle errors / other shell requests
                    self.Update_Geofence(msgDict['output'])
                else:
                    print 'Geofence Request Failed...'
                    
    def Update_Geofence(self, binaryFenceData):
        pass
        #FIXME!


    def Parse_IDP_Status_Messages(self, messageList):
        msgDict ={}
        for msg in messageList:
            msgDict ={}
            if msg.Payload._MIN == 3: #decode geofenceStatus message
                for Field in msg.Payload.Fields[0]:
                    msgDict[Field._Name]=Field._Value

                originalFence = ['Outside','Inside','Unknown']
                replacementFence = [-1,1,None]

                substituteListFence = map(None, originalFence, replacementFence)

                operations = [('altitude', '/', 100.),
                              ('heading', '/', 100.),
                              ('speed', '/', 3.6),
                              ('latitude', '/', 60000.),
                              ('longitude', '/', 60000.)]#,

                
                (msgDict, errorCount) = self.Parse_Fence_Values(msgDict, operations)

                print msg.Payload._Name
                print "Decoded Update: ", errorCount, " : ", msgDict
                
                #msgDict #return updated values...
        if msgDict != {}:
            return msgDict
        else:
            False #failed to update values
                
            
    def Update_IDP_Report_Values(self, msgDict):
        
        for key in msgDict:
            try:
                self.IDPState[key]=msgDict[key]
                self.IDPState['localUpdateTime'] = time.time()
            except:
                print 'Error updating IDPState: '+str(key)
        print self.IDPState

            
    def Parse_Fence_Values(self, msgDict,operations):
        errorCount = 0
        for target, operation, value in operations:
            if operation == '-':
                try:
                    msgDict[target] = float(msgDict[target])-value
                except:
                    errorCount+=1
                
            elif operation == '+':
                try:
                    msgDict[target] = float(msgDict[target])+value
                except:
                    errorCount+=1
                    
            elif operation == '*':
                try:
                    msgDict[target] = float(msgDict[target])*value
                except:
                    errorCount+=1

            elif operation == '/':
                try:
                    msgDict[target] = float(msgDict[target])/value
                except:
                    errorCount+=1
                
            elif operation == 'R': #replace operation
                try:
                    for x,y in value:
                        if msgDict[target] == x:
                            msgDict[target] = y
                except:
                    errorCount+=1
            else:
                pass

        return (msgDict, errorCount)

class Thread_List():
    def __init__(self):
        self._list = []
        
if __name__ == '__main__':
    targetIDPNumber = 2
    
    gateway =  SkywaveIDP.Skywave_IDP_Gateway()
    lock = threading.Lock()
    submitMsgList = Thread_List()
    returnMsgRTDList = Thread_List()
    returnMsgIDPList = Thread_List()
    
    
    gatewayThread = Gateway_Thread(lock, gateway, submitMsgList, returnMsgRTDList, returnMsgIDPList)
    RTDThread = RTD_Thread(lock, gateway, submitMsgList, returnMsgRTDList, targetIDPNumber)
    coreThread = Core_Thread(lock, gateway, submitMsgList, returnMsgIDPList, targetIDPNumber)
    
    gatewayThread.join()


