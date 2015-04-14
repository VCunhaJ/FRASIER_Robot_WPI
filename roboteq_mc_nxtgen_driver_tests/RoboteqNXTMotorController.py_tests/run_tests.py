#!/usr/bin/env python
import os
import sys


# EDIT THESE:
RoboteqNXTMotorController_dir = \
    os.getcwd() + "/../../roboteq_mc_nxtgen_driver/scripts"
defaultPort = "/dev/robot/RoboteqController"
doOnConnect = ["!VAR 0 1", "!G 1 0", "!G 2 0", 0.06]

sys.path.append(RoboteqNXTMotorController_dir)
from RoboteqNXTMotorController import *
from time import sleep
from test_accessories import *



def test_connect(port, wrongPort = "/dev/unexisting_port_23r3454cf5345345"):
    print "Starting test"

    try:
        mc = MotorControllerOpenLoop(port=wrongPort)
        raise RuntimeException("A serial.serialutil.SerialException was " \
                "expected    because port named '%s' doesn't exist, or " \
                "does it?" % port)
    except serial.serialutil.SerialException:
        pass

    print "\nPassed phase 0"

    print "\nTesting start in open loop"
    mc = MotorControllerOpenLoop(port=port, onConnect=doOnConnect, debug=True)

    print "\nTest switching echo on and off"
    print "\nTurn ON the echo"
    mc.setConfig("ECHOF", "0")
    del mc


    print "\nNow will use another log function"

    log = MyLog()
        
    mc = MotorControllerOpenLoop(port=port, debug=log.log)
    print "\nTurn OFF the echo"
    mc.setConfig("ECHOF", "1")
    del mc

    log.clean()
    mc = MotorControllerOpenLoop(port=port, debug=log.log)
    if not log.has("ECHO is OFF detected"):
        raise  RuntimeError("ECHO IS NOT PROPERLY SET OFF")
    print "\nTurn ON the echo"
    mc.setConfig("ECHOF", "0")
    del mc

    log.clean()
    mc = MotorControllerOpenLoop(port=port, debug=log.log)
    if not log.has("ECHO is ON detected"):
        raise  RuntimeError("ECHO IS NOT PROPERLY SET ON")

    #wt = 1000
    #print "\nChange watchdog timeout to %d ms" % wt
    #mc.setConfig("RWD", str(wt))

    
def test_open_loop(port):
    print "Testing open loop mode without kinematics"

    log = MyLog()
    mc = MotorControllerOpenLoop(port=port, onConnect=doOnConnect,
                                 debug=log.log)
    print "\nRestart MicroBasic Script"
    print "\nTest motor 1 in open loop"
    c = 0.15
    t1 = 0.5
    n = 4
    print "Forward"
    for k in range(n):
        mc.setDutyCycles([c, 0])
        print "RPMS: " + str(mc.getRpms())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
    mc.setDutyCycles([0, 0])
    sleep(1)
    print "Reverse"
    for k in range(n):
        mc.setDutyCycles([-c, 0])
        print "RPMS: " + str(mc.getRpms())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
    mc.setDutyCycles([0, 0])
    sleep(1)

    print "\nTest motor 2 in open loop"
    print "Forward"
    for k in range(n):
        mc.setDutyCycles([0, c])
        print "RPMS: " + str(mc.getRpms())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
    mc.setDutyCycles([0, 0])
    sleep(1)
    print "Reverse"
    for k in range(n):
        mc.setDutyCycles([0, -c])
        print "RPMS: " + str(mc.getRpms())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))        
        sleep(t1)
    mc.setDutyCycles([0, 0])
    sleep(1)

    
def test_open_loop_knmtx(port):
    log = MyLog()
    
    print "\nTest in open loop with Kinematics"

    R = 0.12 #rough radius of the wheel
    b = 1.0  #base width
    kFwd = 50.0    
    kRot = 20.0
    fwdKnmtx = [[kFwd*R/2, kFwd*R/2],[2*kRot*R/b, -2*kRot*R/b]]
    mc = MotorControllerOpenLoopWithKnmtx(fwdKnmtx, port=port, 
                                    onConnect=doOnConnect,  debug=log.log)
    n=4
    t1=0.5
    print "\nGo Forward"
    for k in range(n):
        mc.setVelocities([1.0, 0])
        print "RPMS: " + str(mc.getRpms())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
    mc.setVelocities([0.0, 0])
    sleep(1)
    
    print "\nGo Back"
    for k in range(n):
        mc.setVelocities([-1, 0])
        print "RPMS: " + str(mc.getRpms())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
    mc.setVelocities([0.0, 0])
    sleep(1)

    print "\nTurn Left"
    for k in range(n):
        mc.setVelocities([0, 1])
        print "RPMS: " + str(mc.getRpms())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
    mc.setVelocities([0.0, 0])
    sleep(1)    

    print "\nTurn Right"
    for k in range(n):
        mc.setVelocities([0, -1])
        print "RPMS: " + str(mc.getRpms())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
    mc.setVelocities([0.0, 0])
    sleep(1)    

def test_closed_loop(port):
    print "Testing CLOSED LOOP mode without kinematics"

    log = MyLog()
    mc = MotorControllerClosedLoop(port=port, debug=log.log)
    print "\nTest motor 1 in CLOSED LOOP"
    rpm = 50
    t1 = 0.5
    n = 4
    print "Forward"
    for k in range(n):
        mc.setRpms([rpm, 0])
        print "RPMS: " + str(mc.getRpms())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
        
    print "Now stop!"
    mc.setRpms([0, 0])
    for k in range(8):
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(0.5)
        
    print "Reverse"
    for k in range(n):
        mc.setRpms([-rpm, 0])
        print "RPMS: " + str(mc.getRpms())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
        
    print "Now stop!"
    mc.setRpms([0, 0])
    for k in range(8):
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(0.5)

    print "\nTest motor 2 in CLOSED LOOP"

    print "Forward"
    for k in range(n):
        mc.setRpms([0, rpm])
        print "RPMS: " + str(mc.getRpms())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
        
    print "Now stop!"
    mc.setRpms([0, 0])
    for k in range(8):
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(0.5)
        
    print "Reverse"
    for k in range(n):
        mc.setRpms([0, -rpm])
        print "RPMS: " + str(mc.getRpms())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
        
    print "Now stop!"
    mc.setRpms([0, 0])
    for k in range(8):
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(0.5)

def test_closed_loop_knmtx(port):
    log = MyLog()
    
    print "\nTest in closed loop with Kinematics"

    robotWheelRad = .127 #meter
    encWheelRad = 0.025227 #meter   
    baseWidth = 0.52141 #meter
    a = 2*3.14159*encWheelRad/60
    b = a/baseWidth
    fwdKnmtx = [[a/2, a/2],[b, -b]]
    mc = MotorControllerClosedLoopWithKnmtx(fwdKnmtx, port=port, 
            onConnect=doOnConnect,
debug=log.log)
    n=8
    t1=0.5
    print "\nGo Forward"
    for k in range(n):
        mc.setVelocities([0.2, 0])
        print "RPMS: " + str(mc.getRpms())
        print "Velocities: " + str(mc.getVelocities())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
    mc.setVelocities([0.0, 0])
    sleep(1)
    
    print "\nGo Back"
    for k in range(n):
        mc.setVelocities([-0.2, 0])
        print "RPMS: " + str(mc.getRpms())
        print "Velocities: " + str(mc.getVelocities())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
    mc.setVelocities([0.0, 0])
    sleep(1)

    print "\nTurn Left"
    for k in range(n):
        mc.setVelocities([0.0, 0.2])
        print "RPMS: " + str(mc.getRpms())
        print "Velocities: " + str(mc.getVelocities())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
    mc.setVelocities([0.0, 0])
    sleep(1)
    
    print "\nTurn Right"
    for k in range(n):
        mc.setVelocities([0, -0.2])
        print "RPMS: " + str(mc.getRpms())
        print "Velocities: " + str(mc.getVelocities())
        print "Modes: " + str(mc.getConfigFloatList("MMOD"))
        sleep(t1)
    mc.setVelocities([0.0, 0])
    sleep(1)
    

    
def test_comm(port):
    print "Testing how fast it can communicate"

    log = MyLog()
    mc = MotorControllerClosedLoop(port=port, onConnect=doOnConnect,
                                 debug=False)#log.log)
    n = 10000
    t1 = time.time()
    for k in range(n):
        mc.setRpms([100,100])
        mc.sendMessageReliably("?FID")
        #mc.sendMessageReliably("~MMOD")
        #print "Velocities at step %d: %s" % (k, str(mc.getRpms()))
    print "Average T=%f ms" % (1000*(time.time()-t1)/n)
    
if __name__ == "__main__":
    tests = [test_connect, test_open_loop, test_open_loop_knmtx, \
        test_closed_loop, test_closed_loop_knmtx, test_comm]
    if len(sys.argv)>1:
        tests = [locals()["test_" + test] for test in sys.argv[1:]]
    for test in tests:
        test(defaultPort)
