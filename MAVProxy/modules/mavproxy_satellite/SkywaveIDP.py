import datetime
import suds.client as sc
import suds
import base64
import time
import urllib2
class Skywave_IDP_Gateway():

    #Contains all gateway functions
    def __init__(self, accessID = '70000637', password = 'TC100K', serialNumbers = ["01020523SKYF654","01020524SKY7A59"]):
    
        self.IDP = dict(zip(xrange(1, len(serialNumbers) + 1),serialNumbers))
        
        url = "https://isatdatapro.skywave.com/GLGW/GWServices_v1/Messages.wsdl"
        self.accessID = accessID
        self.password = password
        while True:
            try:
                self.gateway  = sc.Client(url)
                break
            except urllib2.URLError, e:
                print e
            time.sleep(1)
            
        self.errorDiscrptionShort= None
        #self.Get_Error_Info() #map short discriptions to gateway errors (takes a while...)
        
        self.gatewayStartTimeUTC = self.Parse_Gateway_Time(self.gateway.service.InfoUTC())
        time.sleep(1)
        self.localTimeOffset = datetime.datetime.now() - self.Parse_Gateway_Time(self.gateway.service.InfoUTC())
        time.sleep(1)
        self.Return_Message_Filter_High_Tide = None

        print "GATEWAY CREATED"
        
        self.alive = True
    
    def is_alive(self):
        return self.alive
        
    def Get_Error_Info(self):
        errors =self.gateway.service.InfoErrorInfos()
        self.errorDiscrptionShort = dict(zip([x.ID for x in errors.ErrorInfo], [x.Name for x in errors.ErrorInfo]))    
        return self.errorDiscrptionShort

    def Parse_Error(self, errorKey):
        print errorKey
        if self.errorDiscrptionShort != None:
            try:
                print self.errorDiscrptionShort[errorKey] #something went wrong, print the error
            except:
                print "NO_MAPPED_ERROR"
        
    def Get_Mobile_ID_Array(self):
        Mobile_Info = self.gateway.service.GetMobileInfos(self.accessID,self.password)
        mobileIDArray = [x.ID for x in Mobile_Info.Mobiles.MobileInfo]
        return mobileIDArray

    def Parse_Gateway_Time(self, timeString):
        gatewayTimeUTC = datetime.datetime.strptime(timeString, "%Y-%m-%d %H:%M:%S")
        return gatewayTimeUTC #python datetime object

    def Format_Time_For_Gateway(self, datetimeObject):
        return datetime.datetime.strftime(datetimeObject,"%Y-%m-%d %H:%M:%S")
        
    def Get_Seconds_Since_Gateway_Start(self):
        currentTime = datetime.datetime.now() #python datetime object
        return (currentTime-(self.gatewayStartTimeUTC+self.localTimeOffset)).total_seconds()

    def Generate_IDP_Payload_Return_Geofence_Points(self,IDP_ID):
        #msg that returns a binary fences.dat file
        
        messageHolder = self.gateway.factory.create('ns0:ForwardMessage')
        messageHolder.DestinationID = self.IDP[IDP_ID]
        messageHolder.UserMessageID = len(IDP_Forward_Message.all_messages)
        del messageHolder.RawPayload
        del messageHolder.StreamHandle
        
        messageHolder.Payload = self.gateway.factory.create('ns0:Message')
        
        messageHolder.Payload._SIN = 26
        messageHolder.Payload._MIN = 1
        messageHolder.Payload._Name ="executeCmd"#"executeCmd"
        del messageHolder.Payload._IsForward# = True
        
        arrayField0 = self.gateway.factory.create('ns0:ArrayOfField')

        field0 = self.gateway.factory.create('ns0:Field')
        field0._Name = "tag"
        field0._Value = 0
        del field0.Message
        del field0.Elements
        del field0._Type
        
        field1 = self.gateway.factory.create('ns0:Field')
        field1._Name = "data"
        field1._Value = "type -hex /data/svc/geofence/fences.dat"
        del field1.Message
        del field1.Elements
        del field1._Type

        arrayField0.Field = [field0, field1]
        messageHolder.Payload.Fields = arrayField0


        message = IDP_Forward_Message(messageHolder)
        
        return message
        

    def Generate_IDP_Payload_DigitalIO(self, states = [None,None] ):
        #make message that targets state of digital IO (enable disable) base on states
        return True
    
    def Generate_IDP_Payload_Geofence(self, states = [None,None] ):
        #make message that targets state of geofence (enable disable) base on states
        
        
        
        message = self.gateway.factory.create('ns0:Message')

        
        message.Payload._SIN = 128
        message.Payload._MIN = 4
        message.Payload._Name = "digitalOutputChange"
        del message.Payload._IsForward# = True
        
        arrayField0 = self.gateway.factory.create('ns0:ArrayOfField')
        field0 = self.gateway.factory.create('ns0:Field')
        field0._Name = "outputs"
        del field0.Message
        del field0._Value
        del field0._Type

        elements = []
        
        if states[0] != None:
            arrayField1 = self.gateway.factory.create('ns0:ArrayOfField')
            element1 = self.gateway.factory.create('ns0:Element')

            field1._Name = "output"
            field1._Value = 0
            del field1._Type
            del field1.Message
            del field1.Elements

            field2._Name = "status"
            field2._Value = states[0]
            del field2._Type
            del field2.Message
            del field2.Elements

            arrayField1.Field = [field1,field2]
            element1.Field = arrayField1
            element1._Index = len(elements)

            elements.append(element1)
            

        if states[1] != None:
            arrayField2 = self.gateway.factory.create('ns0:ArrayOfField')
            element2 = self.gateway.factory.create('ns0:Element')
            
            field3._Name = "output"
            field3._Value = 1
            del field3._Type
            del field3.Message
            del field3.Elements

            field4._Name = "status"
            field4._Value = states[1]
            del field4._Type
            del field4.Message
            del field4.Elements

            arrayField2.Field = [field3,field4]
            element2.Field = arrayField2
            element2._Index = len(elements)


            elements.append(element1)
            

        arrayElement0 = self.gateway.factory.create('ns0:ArrayOfElement')
        arrayElement0.Element = elements
        field0.Elements = arrayElement0
        arrayField0.Field = [field0]

        message.Payload.Fields = arrayField0

        return message

    def Generate_IDP_Message(self, payload, IDP_ID): #message with IDP hardware target (e.g. digital  IO's)
        messageHolder = self.gateway.factory.create('ns0:ForwardMessage')
        messageHolder.DestinationID = self.IDP[IDP_ID]
        messageHolder.UserMessageID = len(IDP_Forward_Message.all_messages)
        del messageHolder.RawPayload
        del messageHolder.StreamHandle
        
    
        

        messageHolder.Payload.Fields = payload

        message = IDP_Forward_Message(messageHolder)
        return message
                             
    def Generate_RTD_Message(self, payload, IDP_ID): #message with IDP serial port target
        
        messageHolder = self.gateway.factory.create('ns0:ForwardMessage')
        messageHolder.DestinationID = self.IDP[IDP_ID]
        messageHolder.UserMessageID = len(IDP_Forward_Message.all_messages)
        del messageHolder.RawPayload
        del messageHolder.StreamHandle
        
        messageHolder.Payload = self.gateway.factory.create('ns0:Message')
        
        messageHolder.Payload._SIN = 28
        messageHolder.Payload._MIN = 129
        messageHolder.Payload._Name = "RS232data"
        del messageHolder.Payload._IsForward# = True
        
        arrayField0 = self.gateway.factory.create('ns0:ArrayOfField')

        field0 = self.gateway.factory.create('ns0:Field')
        field0._Name = "payload"
        field0._Value = base64.b64encode(payload)
        del field0.Message
        del field0.Elements
        del field0._Type

        arrayField0.Field = [field0]
        messageHolder.Payload.Fields = arrayField0


        message = IDP_Forward_Message(messageHolder)
        
        return message
    
    def Submit_Forward_Message_To_Gateway(self, messageList):
        #print "messageList",messageList
        if messageList == []:
            return []
        
        arrayMessageHolder = self.gateway.factory.create('ns0:ArrayOfForwardMessage')

        newMessages = [msg.message for msg in messageList if (msg.ID[0] == None) and (msg.Status == 'BUFFER')] #list of messages that have not been submitted to gateway
        if newMessages == []:
            return messageList  
        
        print "New Submit Messages", newMessages
              
        arrayMessageHolder.ForwardMessage = newMessages
        
        submitMessageResult = self.gateway.service.SubmitForwardMessages(self.accessID,self.password,arrayMessageHolder)

        
        if submitMessageResult.ErrorID == 0:#the list of forward subs was okay
            for msg in messageList:
                for result in submitMessageResult.Submissions.ForwardSubmission:
                    if result.UserMessageID == msg.ID[1]:
                        msg.ErrorID = result.ErrorID
                        msg.ID[0] = result.ForwardMessageID
                        historyDict = {}
                        historyDict['event'] = 'submit'
                        historyDict['time'] = time.time()
                        historyDict['data'] = result
                        msg.History.append(historyDict)
                        if result.ErrorID == 0:
                            msg.Status = 'SUBMITTED'
                        else:
                            try:
                                msg.Status = self.errorDiscrptionShort[result.ErrorID]
                            except:
                                msg.Status = 'UNKNOWN_ERROR'
            return messageList
        else:
            self.Parse_Error(submitMessageResult.ErrorID)
            return False
        

    def Query_List_Of_Forward_Message_Status(self, messageList, timeDelta = None):
        forwardFilter = self.gateway.factory.create('ns0:ForwardStatusFilter')

        queryMessageIDs = [msg.ID[0] for msg in messageList if (msg.ID[0] != None) and (msg.IsClosed == False)] #might be a better test...
        if queryMessageIDs == []:
            return messageList
        
        if (timeDelta == None):
            timeDelta = self.Get_Seconds_Since_Gateway_Start()

        
        forwardMessageIDArray = self.gateway.factory.create('ns0:ArrayOfInt')
        forwardMessageIDArray.int = queryMessageIDs
        forwardFilter.ForwardMessageIDs = forwardMessageIDArray
        
        if timeDelta != None:
            forwardFilter.StartUTC = self.Format_Time_For_Gateway(datetime.datetime.now()-
                                     (datetime.timedelta(seconds = timeDelta+self.localTimeOffset.total_seconds())))
          
        forwardMessageStatusResult =  self.gateway.service.GetForwardStatuses(self.accessID,self.password,forwardFilter)
        
        if (forwardMessageStatusResult.ErrorID == 0) and (forwardMessageStatusResult.Statuses != ""):
            for msg in messageList:
                for result in forwardMessageStatusResult.Statuses.ForwardStatus:
                    if result.ForwardMessageID == msg.ID[0]:
                        msg.ErrorID = result.ErrorID
                        msg.IsClosed = result.IsClosed
                        historyDict = {}
                        historyDict['event'] = 'status'
                        historyDict['time'] = time.time()
                        historyDict['data'] = result
                        msg.History.append(historyDict)
                        if result.ErrorID == 0:
                            msg.Status = result.State
                        else:
                            try:
                                msg.Status = self.errorDiscrptionShort[result.ErrorID]
                            except:
                                msg.Status = 'UNKNOWN_ERROR'
            return messageList
        elif (forwardMessageStatusResult.ErrorID == 0):#no submit results
            return messageList 
        else:
            self.Parse_Error(forwardMessageStatusResult.ErrorID)
            return False
        
    

    def Get_Time_Difference_To_Now(self, olderTime, serverTime = True):
        currentTime = datetime.datetime.now()
        if serverTime == True:
            return (currentTime-olderTime).total_seconds()-self.localTimeOffset.total_seconds()
        else:
            return (currentTime-olderTime).total_seconds()
            
    
        

    def Get_Return_Message_List(self, timeDelta = None, IDP_ID = None):
        returnFilter = self.gateway.factory.create('ns0:ReturnMessageFilter')
    
        if IDP_ID != None:
            returnFilter.MobileID = self.IDP[IDP_ID]
        
        if timeDelta == None:
            timeDelta = self.Get_Seconds_Since_Gateway_Start()
            
        if timeDelta != None:
            returnFilter.StartUTC = self.Format_Time_For_Gateway(datetime.datetime.now()-(datetime.timedelta(seconds = timeDelta+self.localTimeOffset.total_seconds())))


        atts = returnFilter.__keylist__ #get all atts
        for att in atts:
            if getattr(returnFilter, att) == None:
                delattr(returnFilter, att)

        returnMessageList=[]
        try:
            returnMessageResult = self.gateway.service.GetReturnMessages(self.accessID, self.password, returnFilter)
        except:
            print "error retrieving messages from gateway"
            return False
        
        if (returnMessageResult.ErrorID == 0) and (returnMessageResult.NextStartUTC != None):
            self.Return_Message_Filter_High_Tide = self.Parse_Gateway_Time(returnMessageResult.NextStartUTC) #use for next get_return_message
            for result in returnMessageResult.Messages[0]:
                returnMessageList.append(IDP_Return_Message(result))

            return returnMessageList
        
        elif (returnMessageResult.ErrorID == 0):
            return returnMessageList
        
        else:
            self.Parse_Error(returnMessageResult.ErrorID)
            return False

        

        #for ins in IDP_Return_Message.all_messages:
        #   print ins.ID
        print returnMessageResult

            
class IDP_Forward_Message():
    all_messages = []
    
    def __init__(self, message):
        self.__class__.all_messages.append(self)
        self.message = message
        self.IsClosed = False
        self.ErrorID = 0
        self.Status = "BUFFER"
        localID = message.UserMessageID
        self.ID = [None,localID] #list will expand to hold at most two entries
        historyDict = {}
        historyDict['event'] = 'create'
        historyDict['time'] = time.time()
        historyDict['data'] = message
        self.History = [historyDict] #contains all the reports for this message

class IDP_Return_Message():
    all_messages = []
    
    def __init__(self, message):
        self.__class__.all_messages.append(self)
        self.ID = [message.ID]
        self.MessageUTC = message.MessageUTC
        self.ReceiveUTC = message.ReceiveUTC
        self.SIN = message.SIN
        self.MobileID = message.MobileID
        self.Payload = message.Payload
        self.RegionName = message.RegionName
        self.OTAMessageSize = message.OTAMessageSize
        self.IsClosed = False
        self.Status = "BUFFER"
        historyDict = {}
        historyDict['event'] = 'create'
        historyDict['time'] = time.time()
        historyDict['data'] = message
        self.History = [historyDict] #contains all the reports for this message

        

"""
test = Skywave_IDP_Gateway()
msg = test.Generate_IDP_Payload_Return_Geofence_Points(1)
msglist = [msg]
print msglist
print test.Submit_Forward_Message_To_Gateway(msglist)[-1].History
"""

