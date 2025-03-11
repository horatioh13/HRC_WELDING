'''
Python 3.x library to control an UR robot through its TCP/IP interfaces
Copyright (C) 2017  Martin Huus Bjerge, Rope Robotics ApS, Denmark

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL "Rope Robotics ApS" BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Except as contained in this notice, the name of "Rope Robotics ApS" shall not be used
in advertising or otherwise to promote the sale, use or other dealings in this Software
without prior written authorization from "Rope Robotics ApS".
'''
__author__ = "Martin Huus Bjerge"
__copyright__ = "Copyright 2017, Rope Robotics ApS, Denmark"
__license__ = "MIT License"

import os.path
import socket
import struct
import select
import threading
import time
import xml.etree.ElementTree as ET
import numpy as np
from .config import rtde_config
from .state import ConnectionState, DEFAULT_TIMEOUT


class Command:
    RTDE_REQUEST_PROTOCOL_VERSION = 86        # ascii V
    RTDE_GET_URCONTROL_VERSION = 118          # ascii v
    RTDE_TEXT_MESSAGE = 77                    # ascii M
    RTDE_DATA_PACKAGE = 85                    # ascii U
    RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79   # ascii O
    RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73    # ascii I
    RTDE_CONTROL_PACKAGE_START = 83           # ascii S
    RTDE_CONTROL_PACKAGE_PAUSE = 80           # ascii P


class RTDE(threading.Thread): #, metaclass=Singleton
    '''
    Interface to UR robot Real Time Data Exchange interface.
    See this site for more detail:
    http://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/real-time-data-exchange-rtde-guide-22229/

    The constructor takes a UR robot hostname as input and a path to a RTDE configuration file.

    Input parameters:
    host (string):  Hostname or IP of UR Robot (RT CLient server)
    conf_filename (string):  Path to xml file describing what channels to activate
    logger (URBasis_DataLogging obj): A instance if a logger object if common logging is needed.

    Example:
    rob = URBasic.rtde.RTDE('192.168.56.101', 'rtde_configuration.xml')
    rob.close_rtde()
    '''


    def __init__(self, robotModel, conf_filename=None):
        '''
        Constructor see class description for more info.
        '''
        self.__robotModel = robotModel

        self.__reconnectTimeout = 2 #Seconds (while in run)
        self.__dataSend = RTDEDataObject()

        self.__stop_event = True
        threading.Thread.__init__(self)
        self.__dataEvent = threading.Condition()

        self.__conn_state = ConnectionState.DISCONNECTED
        self.__sock = None
        self.__rtde_output_names = None
        self.__rtde_output_config = None
        self.__rtde_input_names = None
        self.__rtde_input_initValues = None
        self.__rtde_input_config = None
        self.__controllerVersion = None
        self.__protocol_version = None
        self.__packageCounter = 0
        self.start()

    def __connect(self):
        '''
        Initialize RTDE connection to host and set up data interfaces based on configuration XML.

        Return value:
        success (boolean)
        '''
        if self.__sock:
            return True

        try:
            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.__sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.__sock.settimeout(DEFAULT_TIMEOUT)
            self.__sock.connect((self.__robotModel.ipAddress, 30004))
            self.__conn_state = ConnectionState.CONNECTED
        except (socket.timeout, socket.error):
            if self.__sock:
                self.__sock.close()
            self.__sock = None
            return False
        return True

    def __disconnect(self):
        '''
        Close the RTDE connection.
        '''
        if self.__sock:
            self.__sock.close()
            self.__sock = None
        self.__conn_state = ConnectionState.DISCONNECTED
        return True

    def isConnected(self):
        '''
        Returns True if the connection is open.

        Return value:
        open (boolean)
        '''
        return self.__conn_state > ConnectionState.DISCONNECTED

    def __isConnected(self):
        '''
        Returns True if the connection is open.

        Return value:
        open (boolean)
        '''
        return self.__conn_state > ConnectionState.DISCONNECTED

    def isRunning(self):
        '''
        Return True if RTDE interface is running
        '''

        return self.__conn_state >= ConnectionState.STARTED

    def __getControllerVersion(self):
        '''
        Returns the software version of the robot controller running the RTDE server.

        Return values:
        major (int)
        minor (int)
        bugfix (int)
        '''
        cmd = Command.RTDE_GET_URCONTROL_VERSION
        self.__send(cmd)

    def __negotiateProtocolVersion(self, protocol):
        '''
        Negotiate the protocol version with the server.
        Returns True if the controller supports the specified protocol version.
        We recommend that you use this to ensure full compatibility between your
        application and future versions of the robot controller.

        Input parameters:
        protocol (int): protocol version number

        Return value:
        success (boolean)
        '''
        cmd = Command.RTDE_REQUEST_PROTOCOL_VERSION
        payload = struct.pack('>H',protocol)
        self.__send(cmd, payload)

    def __setupInput(self, input_variables=None, types=[], initValues=None):
        '''
        Configure an input package that the external(this) application will send to the robot controller.
        An input package is a collection of input input_variables that the external application will provide
        to the robot controller in a single update. Variables is a list of variable names and should be
        a subset of the names supported as input by the RTDE interface.The list of types is optional,
        but if any types are provided it should have the same length as the input_variables list.
        The provided types will be matched with the types that the RTDE interface expects and the
        function returns None if they are not equal. Multiple input packages can be configured.
        The returned InputObject has a reference to the recipe id which is used to identify the
        specific input format when sending an update.
        If input_variables is empty, xml configuration file is used.

        Input parameters:
        input_variables (list<string> or Str): [Optional] Variable names from the list of possible RTDE inputs
        types (list<string> or str): [Optional] Types matching the input_variables

        Return value:
        success (boolean)
        '''

        if input_variables is None:
            root = ET.fromstring(rtde_config)

            #setup data that can be send
            recive = root.find('send')
            input_variables = []
            initValues = []
            for child in recive:
                input_variables.append(child.attrib['name'])
                initValues.append(float(child.attrib['initValue']))

        cmd = Command.RTDE_CONTROL_PACKAGE_SETUP_INPUTS
        if type(input_variables) is list:
            payload = ','.join(input_variables)
        elif type(input_variables) is str:
            payload = input_variables
        else:
            return None

        self.__rtde_input_names = input_variables
        self.__rtde_input_initValues = initValues

        payload = payload.encode('utf-8')
        return self.__send(cmd, payload)

    def __setupOutput(self, output_variables=None, types=[], frequency=500):
        '''
        Configure an output package that the robot controller will send to the
        external(this) application at the control frequency. Variables is a list of
        variable names and should be a subset of the names supported as output by the
        RTDE interface. The list of types is optional, but if any types are provided
        it should have the same length as the output_variables list. The provided types will
        be matched with the types that the RTDE interface expects and the function
        returns False if they are not equal. Only one output package format can be
        specified and hence no recipe id is used for output.
        If output_variables is empty, xml configuration file is used.

        Input parameters:
        output_variables (list<string> or str): [Optional] Variable names from the list of possible RTDE outputs
        types (list<string> or str): [Optional] Types matching the output_variables

        Return value:
        success (boolean)
        '''

        if output_variables is None:
            root = ET.fromstring(rtde_config) 

            #Setup data to be recived
            recive = root.find('receive')
            output_variables = ['timestamp']
            for child in recive:
                output_variables.append(child.attrib['name'])


        cmd = Command.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS
        payload = struct.pack('>d', frequency)
        if type(output_variables) is list:
            payload = ','.join(output_variables)
        elif type(output_variables) is str:
            payload = output_variables
        else:
            return False

        self.__rtde_output_names = output_variables
        payload = payload.encode('utf-8')
        return self.__send(cmd, payload)
        
    def __sendStart(self):
        '''
        Sends a start command to the RTDE server.
        Setup of all inputs and outputs must be done before starting the RTDE interface

        Return value:
        success (boolean)
        '''
        cmd = Command.RTDE_CONTROL_PACKAGE_START
        return self.__send(cmd)

    def __sendPause(self):
        '''
        Sends a pause command to the RTDE server
        When paused it is possible to change the input and output configurations

        Return value:
        success (boolean)
        '''
        cmd = Command.RTDE_CONTROL_PACKAGE_PAUSE
        return self.__send(cmd)

    def sendData(self):
        '''
        Send the contents of a RTDEDataObject as input to the RTDE server.
        Returns True if successful.

        Return value:
        success (boolean)
        '''
        if self.__conn_state != ConnectionState.STARTED:
            return
        if self.__robotModel.StopRunningFlag():
            return
        config = self.__rtde_input_config
        return self.__send(Command.RTDE_DATA_PACKAGE, config.pack(self.__dataSend))

    def setData(self, variable_name, value):
        '''
        Set data to be send to the robot
        Object is locked while updating to avoid sending half updated values,
        hence send all values as two lists of equal lengths

        Input parameters:
        variable_name (List/str):  Variable name from the list of possible RTDE inputs
        value (list/int/double)

        Return value:
        Status (Bool): True=Data sucesfull updated, False=Data not updated
        '''

        #check if input is list of equal length
        if type(variable_name) is list:
            if type(variable_name) != type(value):
                raise ValueError("RTDE " + str(variable_name) + " is not type of " + str(value))
            if len(variable_name) != len(value):
                raise ValueError("List of RTDE Output values does not have same length as list of variable names")
            for ii in range(len(value)):
                if self.hasattr(self.__rtde_input_config.names, variable_name[ii]):
                    self.__dataSend.__dict__[variable_name[ii]] = value[ii]
                else:
                    raise ValueError(str(variable_name[ii]) + " not found in RTDE OUTPUT config")

        else:
            if variable_name in self.__rtde_input_config.names:
                self.__dataSend.__dict__[variable_name] = value
            else:
                raise ValueError(str(variable_name) + " not found in RTDE OUTPUT config")

    def __send(self, command, payload=bytes()):
        '''
        Send command and data (payload) to Robot Controller
        and receive the respond from the Robot Controller.

        Input parameters:
        cmd (int)
        payload (bytes)

        Return value:
        success (boolean)
        '''
        fmt = '>HB'
        size = struct.calcsize(fmt) + len(payload)
        buf = struct.pack(fmt, size, command) + payload
        if self.__sock is None:
            return False

        (_, writable, _) = select.select([], [self.__sock], [], DEFAULT_TIMEOUT)
        if len(writable):
            self.__sock.sendall(buf)
            return True
        else:
            self.__disconnect()
            return False

    def __receive(self):
        byte_buffer = bytes()
        if self.__sock is None:
            return

        (readable, _, xlist) = select.select([self.__sock], [], [], DEFAULT_TIMEOUT)
        if (len(readable)):
            more = self.__sock.recv(16384)
            if len(more) == 0:
                self.__disconnect()
                return None
            byte_buffer +=  more

        if len(xlist) or len(readable) == 0: # Effectively a timeout of DEFAULT_TIMEOUT seconds
            self.__disconnect()
            raise ConnectionError('received 0 bytes from Controller')

        while len(byte_buffer) >= 3:
            (packet_size, packet_command) = struct.unpack_from('>HB', byte_buffer)
            buffer_length = len(byte_buffer)

            if ((buffer_length) >= packet_size):
                packet, byte_buffer = byte_buffer[3:packet_size], byte_buffer[packet_size:]
                data = self.__decodePayload(packet_command, packet)

                if(packet_command == Command.RTDE_GET_URCONTROL_VERSION):
                    self.__verifyControllerVersion(data)
                elif(packet_command == Command.RTDE_REQUEST_PROTOCOL_VERSION):
                    self.__verifyProtocolVersion(data)
                elif(packet_command == Command.RTDE_CONTROL_PACKAGE_SETUP_INPUTS):
                    self.__rtde_input_config = data
                    self.__rtde_input_config.names = self.__rtde_input_names
                    self.__dataSend = RTDEDataObject.create_empty(self.__rtde_input_names, self.__rtde_input_config.id)
                    if self.__rtde_input_initValues is not None:
                        for ii in range(len(self.__rtde_input_config.names)):
                            if 'UINT8' == self.__rtde_input_config.types[ii]:
                                self.setData(self.__rtde_input_config.names[ii], int(self.__rtde_input_initValues[ii]))
                            elif 'UINT32' == self.__rtde_input_config.types[ii]:
                                self.setData(self.__rtde_input_config.names[ii], int(self.__rtde_input_initValues[ii]))
                            elif 'INT32' == self.__rtde_input_config.types[ii]:
                                self.setData(self.__rtde_input_config.names[ii], int(self.__rtde_input_initValues[ii]))
                            elif 'DOUBLE' == self.__rtde_input_config.types[ii]:
                                self.setData(self.__rtde_input_config.names[ii], (self.__rtde_input_initValues[ii]))
                            else:
                                pass

                elif(packet_command == Command.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS):
                    self.__rtde_output_config = data
                    self.__rtde_output_config.names = self.__rtde_output_names
                elif(packet_command == Command.RTDE_CONTROL_PACKAGE_START):
                    self.__conn_state = ConnectionState.STARTED
                elif(packet_command == Command.RTDE_CONTROL_PACKAGE_PAUSE):
                    self.__conn_state = ConnectionState.PAUSED
                elif(packet_command == Command.RTDE_DATA_PACKAGE):
                    self.__updateModel(data)
                elif(packet_command == 0):
                    byte_buffer = bytes()
            else:
                # print("skipping package - unexpected packet_size - length: " + str(len(byte_buffer)))
                byte_buffer = bytes()

        if len(byte_buffer) != 0:
            byte_buffer = bytes()

    def __updateModel(self, rtde_data_package):
        self.__packageCounter = self.__packageCounter + 1
        if(self.__packageCounter % 1000 == 0):
            pass
        if(self.__robotModel.dataDir['timestamp'] != None):
            delta = rtde_data_package['timestamp'] - self.__robotModel.dataDir['timestamp']
            if(delta > 0.00800001):
                pass
        for tagname in rtde_data_package.keys():
            self.__robotModel.dataDir[tagname] = rtde_data_package[tagname]

    def __verifyControllerVersion(self, data):
        self.__controllerVersion = data
        (major, minor, bugfix, build) = self.__controllerVersion
        if major and minor and bugfix:
            if major <= 3 and minor <= 2 and bugfix < 19171:
                raise ValueError("Please upgrade your controller to minimum version 3.2.19171")

    def __verifyProtocolVersion(self, data):
        self.__protocol_version = data
        if(self.__protocol_version != 1):
            raise ValueError("We only support protocol version 1 at the moment")

    def __decodePayload(self, cmd, payload):
        '''
        Decode the package received from the Robot
        payload (bytes)

        Return value(s):
        Output from Robot controller (type is depended on the cmd value)
        '''
        if cmd == Command.RTDE_REQUEST_PROTOCOL_VERSION:
            if len(payload) != 1:
                # self._logger.error('RTDE_REQUEST_PROTOCOL_VERSION: Wrong payload size')
                return None
            return struct.unpack_from('>B', payload)[0]

        elif cmd == Command.RTDE_GET_URCONTROL_VERSION:
            if 12 == len(payload):
                return np.append(np.array(struct.unpack_from('>III', payload)), 0)
            elif 16 == len(payload):
                return np.array(struct.unpack_from('>IIII', payload))
            else:
                # self._logger.error('RTDE_GET_URCONTROL_VERSION: Wrong payload size')
                return None


        elif cmd == Command.RTDE_TEXT_MESSAGE:
            if len(payload) < 1:
                # self._logger.error('RTDE_TEXT_MESSAGE: No payload')
                return None
            EXCEPTION_MESSAGE = 0
            ERROR_MESSAGE = 1
            WARNING_MESSAGE = 2
            INFO_MESSAGE = 3
            fmt = ">" + str(len(payload)) + "B"
            out = struct.unpack_from(fmt, payload)
            level = out[0]
            message = ''.join(map(chr,out[1:]))
            if(level == EXCEPTION_MESSAGE or
               level == ERROR_MESSAGE):
               pass
                # self._logger.error('Server message: ' + message)
            elif level == WARNING_MESSAGE:
                pass
                # self._logger.warning('Server message: ' + message)
            elif level == INFO_MESSAGE:
                pass
                # self._logger.info('Server message: ' + message)

        elif cmd == Command.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
            if len(payload) < 1:
                # self._logger.error('RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS: No payload')
                return None
            has_recipe_id = False
            output_config = RTDE_IO_Config.unpack_recipe(payload, has_recipe_id)
            return output_config

        elif cmd == Command.RTDE_CONTROL_PACKAGE_SETUP_INPUTS:
            if len(payload) < 1:
                # self._logger.error('RTDE_CONTROL_PACKAGE_SETUP_INPUTS: No payload')
                return None
            has_recipe_id = True
            input_config = RTDE_IO_Config.unpack_recipe(payload, has_recipe_id)
            return input_config

        elif cmd == Command.RTDE_CONTROL_PACKAGE_START:
            if len(payload) != 1:
                # self._logger.error('RTDE_CONTROL_PACKAGE_START: Wrong payload size')
                return None
            return bool(struct.unpack_from('>B', payload)[0])

        elif cmd == Command.RTDE_CONTROL_PACKAGE_PAUSE:
            if len(payload) != 1:
                # self._logger.error('RTDE_CONTROL_PACKAGE_PAUSE: Wrong payload size')
                return None
            return bool(struct.unpack_from('>B', payload)[0])

        elif cmd == Command.RTDE_DATA_PACKAGE:
            if self.__rtde_output_config is None:
                # self._logger.error('RTDE_DATA_PACKAGE: Missing output configuration')
                return None
            output = self.__rtde_output_config.unpack(payload)
            return output

        else:
            pass
            # self._logger.error('Unknown RTDE command type: ' + chr(cmd))


    def __listEquals(self, l1, l2):
        if len(l1) != len(l2):
            return False
        for i in range(len((l1))):
            if l1[i] != l2[i]:
                return False
        return True

    def __wait(self):
        '''Wait while the data receiving thread is receiving a new data set.'''
        cnt = 0
        while self.__conn_state < ConnectionState.STARTED:
            time.sleep(1)
            cnt +=1
            if cnt>5:
                # self._logger.warning('wait_rtde timed out while RTDE interface not running')
                return False

        with self.__dataEvent:
            self.__dataEvent.wait(DEFAULT_TIMEOUT)
        return True

    '''Threading Data receive'''
    def close(self):
        if self.__stop_event is False:
            self.__stop_event = True
            self.__wait()
            self.join()
            self.__disconnect()

    def run(self):
        self.__stop_event = False
        t0 = time.time()
        while (time.time()-t0<self.__reconnectTimeout) and self.__conn_state != ConnectionState.STARTED:
            self.__connect()
            self.__disconnect()
            self.__connect()
            self.__getControllerVersion()
            self.__receive()
            self.__negotiateProtocolVersion(1)
            self.__receive()
            self.__setupOutput()
            self.__receive()
            self.__setupInput()
            self.__receive()
            self.__sendStart()
            self.__receive()
        if self.__conn_state != ConnectionState.STARTED:
            return

        while (not self.__stop_event) and (time.time()-t0<self.__reconnectTimeout):
            try:
                self.__receive()
                t0 = time.time()
            except Exception:
                if self.__conn_state >= ConnectionState.STARTED:
                    self.__conn_state = ConnectionState.ERROR

                self.__sendPause()
                if not self.__sendStart():
                    self.__disconnect()
                    time.sleep(1)
                    self.__connect()
                    self.__setupOutput()
                    self.__setupInput()
                    self.__sendStart()

        self.__sendPause()
        with self.__dataEvent:
            self.__dataEvent.notifyAll()
        
        self.__robotModel.stopRunningFlag = True
        if self.__conn_state == ConnectionState.ERROR:
            self.__robotModel.rtcProgramExecutionError = True
        

class RTDE_IO_Config(object):
    __slots__ = ['id', 'names', 'types', 'fmt']
    @staticmethod
    def unpack_recipe(buf, has_recipe_id):
        rmd = RTDE_IO_Config()
        if has_recipe_id:
            rmd.id = struct.unpack_from('>B', buf)[0]
            fmt = ">" + str(len(buf)) + "B"
            buf = struct.unpack_from(fmt, buf)
            buf = ''.join(map(chr,buf[1:]))
            rmd.types = buf.split(',')
            rmd.fmt = '>B'
        else:
            fmt = ">" + str(len(buf)) + "B"
            buf = struct.unpack_from(fmt, buf)
            buf = ''.join(map(chr,buf[:]))
            rmd.types = buf.split(',')
            rmd.fmt = '>'
        for i in rmd.types:
            if i=='INT32':
                rmd.fmt += 'i'
            elif i=='UINT32':
                rmd.fmt += 'I'
            elif i=='VECTOR6D':
                rmd.fmt += 'd'*6
            elif i=='VECTOR3D':
                rmd.fmt += 'd'*3
            elif i=='VECTOR6INT32':
                rmd.fmt += 'i'*6
            elif i=='VECTOR6UINT32':
                rmd.fmt += 'I'*6
            elif i=='DOUBLE':
                rmd.fmt += 'd'
            elif i=='UINT64':
                rmd.fmt += 'Q'
            elif i=='UINT8':
                rmd.fmt += 'B'
            elif i=='IN_USE':
                raise ValueError('An input parameter is already in use.')
            else:
                raise ValueError('Unknown data type: ' + i)
        return rmd

    def pack(self, state):
        l = state.pack(self.names, self.types)
        return struct.pack(self.fmt, *l)

    def unpack(self, data):
        li =  struct.unpack_from(self.fmt, data)
        return RTDEDataObject.unpack(li, self.names, self.types)

class RTDEDataObject(object):
    '''
    Data container for data send to or received from the Robot Controller.
    The Object will have attributes for each of that data tags received or send.
    e.g.  obj.actual_digital_output_bits
    '''
    recipe_id = None
    def pack(self, names, types):
        if len(names) != len(types):
            raise ValueError('List sizes are not identical.')
        l = []
        if(self.recipe_id is not None):
            l.append(self.recipe_id)
        for i in range(len(names)):
            if self.__dict__[names[i]] is None:
                raise ValueError('Uninitialized parameter: ' + names[i])
            if types[i].startswith('VECTOR'):
                l.extend(self.__dict__[names[i]])
            else:
                l.append(self.__dict__[names[i]])
        return l

    @staticmethod
    def unpack(data, names, types):
        if len(names) != len(types):
            raise ValueError('List sizes are not identical.')
        obj = dict()
        offset = 0
        for i in range(len(names)):
            obj[names[i]] = RTDEDataObject.unpack_field(data, offset, types[i])
            offset += RTDEDataObject.get_item_size(types[i])
        return obj

    @staticmethod
    def create_empty(names, recipe_id):
        obj = RTDEDataObject()
        for i in range(len(names)):
            obj.__dict__[names[i]] = None
        obj.recipe_id = recipe_id
        return obj

    @staticmethod
    def get_item_size(data_type):
        if data_type.startswith('VECTOR6'):
            return 6
        elif data_type.startswith('VECTOR3'):
            return 3
        return 1

    @staticmethod
    def unpack_field(data, offset, data_type):
        size = RTDEDataObject.get_item_size(data_type)
        if(data_type == 'VECTOR6D' or
           data_type == 'VECTOR3D'):
            return np.array([float(data[offset+i]) for i in range(size)])
        elif(data_type == 'VECTOR6UINT32'):
            return np.array([int(data[offset+i]) for i in range(size)])
        elif(data_type == 'DOUBLE'):
            return float(data[offset])
        elif(data_type == 'UINT32' or
             data_type == 'UINT64'):
            return int(data[offset])
        elif(data_type == 'VECTOR6INT32'):
            return np.array([int(data[offset+i]) for i in range(size)])
        elif(data_type == 'INT32' or
             data_type == 'UINT8'):
            return int(data[offset])
        raise ValueError('unpack_field: unknown data type: ' + data_type)
