'''mavlink log to numpy array writer'''

from pymavlink import mavutil

import numpy as np
import struct
import memory_profiler


class mavmemlog(mavutil.mavfile):
    '''a MAVLink log in memory. This allows loading a log into
    memory to make it easier to do multiple sweeps over a log'''
#     @profile
    def __init__(self, mav, progress_callback=None):
        mavutil.mavfile.__init__(self, None, 'memlog')
        self._msgs = []
        self._count = 0
        self.rewind()
        self._flightmodes = []
        last_flightmode = None
        last_timestamp = None
        last_pct = 0
        
        self.ignore = ['FMT', 'PARM', 'MSG',
                       'TERRAIN_DATA', 'STATUSTEXT', 'PARAM_VALUE']
        self.write_flag = False
        self.fds = {}
        self.dtypes = {}
        self.msg_mults = {}
        self.struct_fmts= {}
        
        self.message_count = {}
        self.message_field_count = {}
        
        import os, shutil
        
        folder = '/tmp/mav'
        
        if not os.path.exists(folder):
            os.makedirs(folder)
        
        for the_file in os.listdir(folder):
            file_path = os.path.join(folder, the_file)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
                #elif os.path.isdir(file_path): shutil.rmtree(file_path)
            except Exception, e:
                print e
                
        
        while True:
            m = mav.recv_msg()
            if m is None:
                break
            
            if int(mav.percent) != last_pct and progress_callback:
                progress_callback(int(mav.percent))
                last_pct = int(mav.percent)
#             self._msgs.append(m)
            
            
            type = m.get_type()
            if type in self.ignore:
                self.write_flag = False
            else:
                self.write_flag = True
                
#             print type

            
            if type not in self.message_count.keys() and self.write_flag:
                # if this is the first of the msg type
                self.message_count[type] = 0
                
                #struct_fmt = m.fmt.msg_struct # a string used to discribe how to pack the vars of a BIN file
                struct_fmt = m.format
                struct_fmt+='d'
                self.struct_fmts[type] = struct_fmt
                
                #struct_columns = m.fmt.columns # the names of the cols of a BIN
                struct_columns = m.ordered_fieldnames
                struct_columns.append('timestamp')
#                 struct_fmt = '<'+'d'*len(struct_columns)
                #msg_mults = m.fmt.msg_mults # mults to apply later of a BIN
                msg_mults = [None]*len(m.ordered_fieldnames)
                msg_mults.append(None) #for the timestamp
                self.msg_mults[type]= {key:value for key, value in zip(struct_columns,msg_mults)}
                
                self.fds[type] = open('/tmp/mav/'+type+'.np', 'ab')
                
                msg_dtype = np.dtype(zip(struct_columns,[struct_fmt[0]+x for x in struct_fmt[1:]]))
                print msg_dtype
                self.dtypes[type] = msg_dtype
                # get the field names and make colums
                self.message_field_count[type] = struct_columns
                        
            if self.write_flag:
                self.message_count[type] += 1
                #struct_elements = m._elements # the raw values of a BIN
                struct_elements = [m.to_dict()[x] for x in m.ordered_fieldnames if x != 'timestamp']
                struct_elements.append(m._timestamp)
                self.fds[type].write(struct.pack(self.struct_fmts[type], *struct_elements))
#             now we know how many of each msg we have and the fields that they contain...
            if mav.flightmode != last_flightmode:
                if len(self._flightmodes) > 0:
                    (mode, t1, t2) = self._flightmodes[-1]
                    self._flightmodes[-1] = (mode, t1, m._timestamp)
                self._flightmodes.append((mav.flightmode, m._timestamp, None))
                last_flightmode = mav.flightmode
            self._count += 1
            last_timestamp = m._timestamp
            self.check_param(m)
        if last_timestamp is not None and len(self._flightmodes) > 0:
            (mode, t1, t2) = self._flightmodes[-1]
            self._flightmodes[-1] = (mode, t1, last_timestamp)
        self.close_fds()
        
        if progress_callback:
            progress_callback(int(100))
        mav.data = None # release the memory, no longer needed...
        
    def close_fds(self):
        for key in self.fds.keys():
            self.fds[key].flush()
            self.fds[key].close()
        
    def check_param(self, m):
        type = m.get_type()
        if type == 'PARAM_VALUE':
            s = str(m.param_id)
            self.params[str(m.param_id)] = m.param_value
        elif type == 'PARM' and getattr(m, 'Name', None) is not None:
            self.params[m.Name] = m.Value
    

    
    def rewind(self):
        '''rewind to start'''
        self._index = 0
        self.percent = 0
        self.messages = {}
        self.message_count = {}
        self._flightmode_index = 0
        self._timestamp = None
        self.flightmode = None
        self.params = {}

    def flightmode_list(self):
        '''return list of all flightmodes as tuple of mode and start time'''
        return self._flightmodes
    
# @profile
def test():
    #     DFFormat(144,GPS2,BIHBcLLeEefIBI,['Status', 'TimeMS', 'Week', 'NSats', 'HDop', 'Lat', 'Lng', 'Alt', 'Spd', 'GCrs', 'VZ', 'T', 'DSc', 'DAg'])
#     <BIHBhiiiIifIBI
#     ['Status', 'TimeMS', 'Week', 'NSats', 'HDop', 'Lat', 'Lng', 'Alt', 'Spd', 'GCrs', 'VZ', 'T', 'DSc', 'DAg']
#     [1, 1988889216, 48446, 12, 116, -309283263, 1365452053, 13014, 4, 14259, -0.0240020751953125, 22895, 0, 0]
#     [None, None, None, None, 0.01, 1e-07, 1e-07, 0.01, 0.01, 0.01, None, None, None, None]
#     1 1988889216 48446 12 1.16 -30.9283263 136.5452053 130.14 0.04 142.59 -0.0240020751953 22895 0 0
#     bina = struct.pack("<dddddddddddddd", *[1, 1988889216, 48446, 12, 116, -309283263, 1365452053, 13014, 4, 14259, -0.0240020751953125, 22895, 0, 0])
    bina = struct.pack("<BIHBhiiiIifIBI", *[1, 1988889216, 48446, 12, 116, -309283263, 1365452053, 13014, 4, 14259, -0.0240020751953125, 22895, 0, 0])


    fmt =  "<BIHBhiiiIifIBI"
    
    print len(fmt)
    bits = [None, None, None, None, 0.01, 1e-07, 1e-07, 0.01, 0.01, 0.01, None, None, None, None]
    fie = ['Status', 'TimeMS', 'Week', 'NSats', 'HDop', 'Lat', 'Lng', 'Alt', 'Spd', 'GCrs', 'VZ', 'T', 'DSc', 'DAg']
    #fmt = '<'+'d'*len(bits)
    print len(fmt)

    fin = [fmt[0]+x for x in fmt[1:]]
    
    
    
    print fin
    
    print zip(fie,fin)
    
    gps_type = np.dtype(zip(fie,fin))
    

    fmt = '<'+'d'*len(bits)
    fin = [fmt[0]+x for x in fmt[1:]]
    double_type = np.dtype(zip(fie,fin))
    f = open('temp.np', mode='wb')
    count = 0
    while count < 100000:
        f.write(bina)
        count +=1
    f.close()
    a = np.fromfile('temp.np', dtype=gps_type)
    one = (a.nbytes)*10**-6
    print 'pre', one, 'MiB'
    a=  a.astype(dtype=double_type, casting='safe', subok=False, copy=False)
    two =(a.nbytes)*10**-6
    print 'post', two, 'MiB'
    print 'diff', two-one, 'MiB'
    a[:]['Lat']*= 1e-7
    
    print a
    color = [0,1,2,3]
    N = len(a)
    
    
    
    f_color = np.ones((N, 4), dtype=np.float32)
    f_color[:, 0] = np.linspace(0, 1, N)
    f_color[:, 1] = f_color[::-1, 0]
    print f_color.shape
    
    #true colour array
    color = np.asarray(color, dtype=np.float32)
    
    t_color = np.array([color,]*N)
    #t_color = t_color.reshape((N, 4))
    print t_color.shape
    
    mask =np.ones(N, dtype=np.bool)
    mask = np.repeat(mask, 4)
    mask = mask.reshape((N, 4))
    print mask.shape
    print t_color 
    print np.copyto(t_color, f_color, where=mask)
    print t_color

if __name__ == "__main__":
    import numpy as np
    import struct
    import memory_profiler
    test()

