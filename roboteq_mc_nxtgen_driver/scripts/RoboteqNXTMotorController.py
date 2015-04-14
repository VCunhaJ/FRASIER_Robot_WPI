#!/usr/bin/env python
#The following is just to use print in a lambda function. See
#http://stackoverflow.com/questions/2970858/python-lambda-with-print-statement
from __future__ import print_function 
import serial
import io
import re
from numpy import linalg, array, dot
import time
from threading import Lock

class MotorController:

    def __init__(self, port="/dev/ttyACM0", timeout=0.01, sendAttempts=3, 
                        onConnect=[], portCfg={}, debug=None):
        """
        Opens a serial port to communicte with the motorcontroller
        
        Args:
            port: serial port name the MC is connected to.
            timeout: timeout for the motorcontroller to return a complete 
                string reply (it's basically serial port timeout)
            sendAttempts: how many times to try to send a message to 
                the motorcontroller (see sendMessageReliably() below).
            onConnect: list of strings and numbers. Right after connecting to 
                MC and reading echo status, the contrucor will go thorugh this
                list. Every string will be sent to MC, responses ignored, and 
                every number will be interpreted as sleep time in sec. 
            portCfg: dictironary with serial port parameters. 
                As long as motorcontroller serial communication protocol 
                parameters are not different from what they are now at 2013/09
                you don't need to modify this dictironary. Otherwise you can 
                specify in this dictionary any serial port params which are 
                defined here: 
                http://pyserial.sourceforge.net/pyserial_api.html#serial.
                Serial
            debug: Can be: None=False, True, or a reference to a function. If 
                True debug messages will be printed to STDOUT, if a reference
                to a function, then every debug message will be passed to the
                function, otherwise no debug messages.
        
        Returns:
            A new class instance
        
        Raises:
            Same as serial.Serial.__init__()
            
        """

        self._setupLog(debug)
        self.mcLock = Lock()    
        portCfgDefault = { #Default port config:
            'port':port,
            'timeout': timeout,
            'baudrate':115200, 
            'bytesize':8, 
            'parity':"N", 
            'stopbits':1} 
        portCfgDefault.update(portCfg)
        self.port = None #In case of crashing on opening the port, see __del__
        self.log("vv==Opening port %s==vv" % portCfgDefault['port'])
        self.port = serial.Serial(**portCfgDefault)
        
        self.portIo = io.TextIOWrapper(
            io.BufferedRWPair(self.port, self.port, 1),  
            newline = '\r',
            line_buffering = True)
        self.sendAttempts = sendAttempts
        self._setEchoBasedOnMCConfig()
        for cmd in onConnect:
            if type(cmd) is str:
                self.sendMessageReliably(cmd)
            else:
                time.sleep(float(cmd))
                
    def _setupLog(self, debugPrm):
        self.logLock = Lock()
        logFunc = None
        if type(debugPrm) is bool and debugPrm:
            logFunc = lambda msg: print(msg)
        elif hasattr(debugPrm, '__call__'):
            logFunc = debugPrm
        else:
            logFunc = lambda msg: None

        def safeLogFunc(msg):
            self.logLock.acquire()
            logFunc(msg)
            self.logLock.release()
        self.log = safeLogFunc

    def __del__(self):
        if self.port:
            self.log("^^==Closing port %s==^^" % self.port.port)
            self.port.close();

    def _readline(self, maxSize):
        resp = self.portIo.readline(maxSize)
        self.log("RECEIVED<%s" % resp)
        #self.log("BIN RCVD<%s" % ":".join("{0:x}".format(ord(c)) for c in resp))
        return resp
    
    def _writeline(self, msg):
        self.portIo.write(unicode(msg + "\r"))
        self.log("SENT    >%s" % unicode(msg))

    def sendMessageReliably(self, req):
        """ 
        Reliably sends a given string to the motor controller and returns its 
        response.
        
        'Realiably' means that if there is a detectable error in 
        motorcontroller reply (no echo, no reply within a timeout, '-' as a
        reply), then the the command will be resent N-1 times  (N is 
        specified by sendAttempts parameter in the contructor). Echo is
        checked iff echo parameter in contructor is True.
        
        Args:
            req: string to send to motorcontroller without '\r'.

        Returns:
            resp: motor controller response without '\r'. For commands is 
            always '+'. For queries is the reply.

        Raises:
            IOError if failed to received a valid response after N attempts 
            (see above)
        """
        self.mcLock.acquire()
        echo = None
        resp = None
        for k in range(self.sendAttempts):
            self._writeline(req)
            self.port.flushInput()
            self.portIo.flush()
            if self.echoIsOn:
                echo = self._readline(len(req)+1).rstrip()
                if echo != req:
                    self.log("Failed echo. Attempt=%d" % k)
                    continue
            resp = self._readline(1000)
            if len(resp)<1 or resp[-1] != "\r":
                self.log("Got an incomplete string (no '\\r' at the end). "\
                            "Response: '%s'. Attempt=%d" % (resp, k))
            elif resp[0]=='-':
                self.log("Got response starting with '-'. Response: '%s'. "\
                            "Attempt=%d" % (resp, k))
            else:
                self.mcLock.release()
                return resp.rstrip()
            
        errMsg = "Motor controller communication error. Tried to send a " \
            "message %d times, but every time either echo was incorrect, " \
            "or, '-' was returned. Another possible reason is that a read " \
            "timeout occured while reading a response. Request='%s', " \
            "latest response='%s', timeout=%f ms. " \
            % (self.sendAttempts, req, resp, self.port.timeout)
        if self.echoIsOn:
            errMsg = errMsg + "Latest echo= '%s'. " % echo
        self.mcLock.release()
        raise IOError(errMsg)


    def _setEchoBasedOnMCConfig(self):
        """
        Identifies whether motorcontroller echo is ON ir Off
        """
        self.echoIsOn = False
        for k in range(self.sendAttempts):
            resp = self.sendMessageReliably("~ECHOF")
            if resp == "~ECHOF":
                resp = self._readline(10).rstrip()
                if resp == "ECHOF=0":
                    self.echoIsOn = True
                    self.log("ECHO is ON detected")
                    return
            elif resp == "ECHOF=1":
                self.log("ECHO is OFF detected")
                return
        raise IOError("Could not read echo config parameter(ECHOF) from " \
                "the motor controller. Request sent: '~ECHOF', response " \
                "received: '%s'" % resp)

    def parseValue(self, resp, key):
        m = re.match(key+r'=(.*)$', resp)
        if m:
            return m.group(1)
        else:
            raise IOError("Could not parse returned string. Response: '%s'. "\
                "Expected format: '%s=RETURNEDVALUE'" % (resp, key))
    
    def parseFloatList(self, value, key):
        vals = list()
        try:
            for val in value.split(':'):
                vals.append(float(val))
            return vals
        except:
            raise IOError("Could not parse a returned value for key='%s' as "\
                "a column-separated list of floats. Returned value ='%s'" \
                % (key, value))

    #Methods identical to those defined for MC MicroBasic
    def setCommand(self, cmd):
        self.sendMessageReliably("!"+cmd)

    def getValue(self, key):
        return self.parseValue(self.sendMessageReliably("?"+key),key)

    def setConfig(self, key, value):
        self.sendMessageReliably("^%s %s" %(key, str(value)))

    def getConfig(self, key):
        return self.parseValue(self.sendMessageReliably("~"+key),key)

    #Auxiliray methods for list values
    def setCommandList(self, cmd, vals):
        for k,val in enumerate(vals):
            if val is not None:
                self.setCommand("%s %d %d"% (cmd, k+1, int(val)))

    def setConfigList(self, key, vals):
        for k,val in enumerate(vals):
            if val is not None:
                self.setConfig(key, "%d %d"% (k+1, int(val)))
        
    def getValueFloatList(self, key):
        return self.parseFloatList(self.getValue(key), key)

    def getConfigFloatList(self, key):
        return self.parseFloatList(self.getConfig(key), key)

    #Other methods    
    def setModeForAllChannels(self, mode):
        n = len(self.getConfigFloatList("MMOD")) # number of channles
        self.setConfigList("MMOD",[mode]*n)
        
    def setEmergencyState(self, on=True):
        """
        Sets or release emregncy stop on the motor controller.
        
        Args:
            on: if True then Emergency Stop command is sent to the MC,
                otherwise, a Emergency Stop Release command is sent (see MC 
                doc for '!EX' and '!MG'
        """
        if on:
            self.setCommand("EX ")
        else: #release emergency state
            self.setCommand("MG ")


    def getMCVoltages(self):
        vs = self.getValueFloatList("V")
        return [float(vs[0])/10, float(vs[1])/10, float(vs[2])/1000]

    def sendMotorCmd(self, channel, value):
        self.setCommand("G %d %d" % (channel+1, int(value)))
    
    def sendMotorCmds(self,values):
        self.setCommandList("G", values)

    def getRpms(self):
        return self.getValueFloatList("S")



class MotorControllerClosedLoop(MotorController):
    """
    This class set MC into Closed loop mode and implements two additional 
    methods
    """

    def __init__(self, **kw):
        MotorController.__init__(self, **kw)
        #time.sleep(1)
        self.setModeForAllChannels(1)    #set closed loop
        self.maxRpms = self.getConfigFloatList("MXRPM")
    
    def setRpm(self, channel, rpm):
        self.sendMotorCmd(channel, int(1000*float(rpm)/self.maxRpms[channel]))
    
    def setRpms(self, rpms):
        for ch, rpm in enumerate(rpms):
            if rpm is not None:
                self.setRpm(ch, rpm)



class MotorControllerOpenLoop(MotorController):

    def __init__(self, **kw):
        MotorController.__init__(self, **kw)
        self.setModeForAllChannels(0) #set open loop for all channels
        self.maxPwrFwd = self.getConfigFloatList("MXPF")
        self.log("Max forward powers: " + str(self.maxPwrFwd))
        self.maxPwrRvs = self.getConfigFloatList("MXPR")
        self.log("Max reverse powers: " + str(self.maxPwrRvs))

    def setDutyCycle(self, channel, v): 
        """
        Args:
            v: in scale [-1;+1] of max power which is set by MXPF and MXPR
        """
        v_trimmed = min(1.0, max(-1.0, v))
        if v_trimmed != v:
            self.log("Duty cycle command=%f for channel=%d is out of [-1;1] "\
                "range. Trimming it to %f" % (v, channel, v_trimmed))
        self.sendMotorCmd(channel, int(1000*v_trimmed))
    
    def setDutyCycles(self, volts):
        for ch, v in enumerate(volts):
            if v is not None:
                self.setDutyCycle(ch, v)
    
    #def setVoltage(self, channel, v):
        #"""
        #Args:
            #v: in actual volts
        #"""
        #if v>=0:
            #self.setDutyCycle(channel, int(v/float(self.maxPwrFwd[channel])))
        #else:
            #self.setDutyCycle(channel, int(v/float(self.maxPwrRvs[channel])))
    
    #def setVoltages(self, volts):
        #for ch, v in enumerate(volts):
            #if v is not None:
                #self.setVoltage(ch, v)


class Kinematics():
    def __init__(self, fwdKnmtx, invKnmtx = None):
        self.fwdKnmtx = array(fwdKnmtx)
        if invKnmtx is None:    
            self.invKnmtx = linalg.pinv(fwdKnmtx)    #pseudo-inverse
        else:
            self.invKnmtx = invKnmtx
        self.log("Forward kinematics matrix: " + str(self.fwdKnmtx))
        self.log("Inverse kinematics matrix: " + str(self.invKnmtx))
    
    def calcFwdKinematics(self, vals):
        return dot(self.fwdKnmtx, array(vals))
        
    def calcInvKinematics(self, vals):
        return dot(self.invKnmtx, array(vals))



class MotorControllerClosedLoopWithKnmtx(MotorControllerClosedLoop, 
                                                            Kinematics):
    def __init__(self, fwdKnmtx, invKnmtx = None, **kw):
        MotorControllerClosedLoop.__init__(self, **kw)
        Kinematics.__init__(self, fwdKnmtx, invKnmtx)
        
    def getVelocities(self):
        return self.calcFwdKinematics(self.getRpms())
    
    def setVelocities(self, velocities):
        self.setRpms(self.calcInvKinematics(velocities))



class MotorControllerOpenLoopWithKnmtx(MotorControllerOpenLoop, Kinematics):
    def __init__(self, fwdKnmtx, invKnmtx = None, **kw):
        MotorControllerOpenLoop.__init__(self, **kw)
        Kinematics.__init__(self, fwdKnmtx, invKnmtx)
        
    def getVelocities(self):
        return self.calcFwdKinematics(self.getRpms())
    
    def setVelocities(self, velocities):
        self.setDutyCycles(self.calcInvKinematics(velocities))
