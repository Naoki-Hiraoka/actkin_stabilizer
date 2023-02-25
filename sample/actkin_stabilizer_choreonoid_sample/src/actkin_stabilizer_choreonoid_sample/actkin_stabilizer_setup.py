#!/usr/bin/env python

import time
import math
import sys

from hrpsys import rtm

from hrpsys.OpenHRP import *
import OpenHRP

import hrpsys_ext_rtc
from hrpsys_ext_rtc.EmergencyStopper2Service_idl import *
from hrpsys_ext_rtc.CollisionDetector2Service_idl import *

import actkin_stabilizer
from actkin_stabilizer.ActKinStabilizerService_idl import *

def findComp(name):
    timeout_count = 0
    comp = None
    while timeout_count < 10:
        comp = rtm.findRTC(name)
        if comp != None and comp.isActive():
            break
        print("find Comp wait for " + name)
        time.sleep(1)
        timeout_count += 1
    if comp == None:
        print("Cannot find component: %s" % name)
    return comp

class ActKinStabilizer_Configurator(object):
    Groups = {}

    rh_svc = None
    seq_svc = None
    sh_svc = None
    akst_svc = None
    co_svc = None
    es_svc = None
    kf_svc = None
    rmfo_svc = None
    log_svc = None

    def __init__(self, *args, **kwargs):
        self.Groups = {}

        self.rh_svc = rtm.findService(findComp("RobotHardware0"),"RobotHardwareService","RobotHardwareService","service0")._narrow(OpenHRP.RobotHardwareService)
        self.seq_svc = rtm.findService(findComp("seq"),"SequencePlayerService","SequencePlayerService","service0")._narrow(OpenHRP.SequencePlayerService)
        self.sh_svc = rtm.findService(findComp("sh"),"StateHolderService","StateHolderService","service0")._narrow(OpenHRP.StateHolderService)
        self.akst_svc = rtm.findService(findComp("akst"),"ActKinStabilizerService","ActKinStabilizerService","service0")._narrow(actkin_stabilizer.ActKinStabilizerService)
        self.co_svc = rtm.findService(findComp("CollisionDetector20"),"CollisionDetector2Service","CollisionDetector2Service","service0")._narrow(hrpsys_ext_rtc.CollisionDetector2Service)
        self.es_svc = rtm.findService(findComp("es"),"EmergencyStopper2Service","EmergencyStopper2Service","service0")._narrow(hrpsys_ext_rtc.EmergencyStopper2Service)
        self.ces_svc = rtm.findService(findComp("ces"),"EmergencyStopper2Service","EmergencyStopper2Service","service0")._narrow(hrpsys_ext_rtc.EmergencyStopper2Service)
        self.kf_svc = rtm.findService(findComp("kf"),"KalmanFilterService","KalmanFilterService","service0")._narrow(OpenHRP.KalmanFilterService)
        self.rmfo_svc = rtm.findService(findComp("rmfo"),"RemoveForceSensorLinkOffsetService","RemoveForceSensorLinkOffsetService","service0")._narrow(OpenHRP.RemoveForceSensorLinkOffsetService)
        self.log_svc = findComp("log").service("service0")._narrow(OpenHRP.DataLoggerService)


    def servoOn(self, jname='all'):
        self.servoOff()

        c = False
        while (c != 'Y' and c != 'y'):
            c = raw_input("press 'Y' for servo ON and power ON. >> ")

        # urata system flag resetting for jaxon
        self.rh_svc.power(jname ,OpenHRP.RobotHardwareService.SWITCH_ON)
        time.sleep(0.01)
        self.rh_svc.power(jname ,OpenHRP.RobotHardwareService.SWITCH_OFF)
        time.sleep(0.02)

        self.rh_svc.setServoGainPercentage(jname,100) # for setting 100 to initial servo position gain

        # reset JointGroups
        for k, v in self.Groups.items():
            self.seq_svc.removeJointGroup(k)
        for k, v in self.Groups.items():
            self.seq_svc.waitInterpolationOfGroup(k)
        for k, v in self.Groups.items():
            self.seq_svc.addJointGroup(k, v)

        # move to idle mode for filter type RTCs
        for rtc_name in ["akst","es","ces","co","el"]:
            rtc = rtm.findRTC(rtc_name)
            if rtc:
                rtc.stop();
                rtc.start();

        time.sleep(2) # withtout this. command angle jumps
        self.sh_svc.goActual()
        time.sleep(0.1)
        self.rh_svc.power(jname, OpenHRP.RobotHardwareService.SWITCH_ON)
        time.sleep(0.2)
        self.rh_svc.servo(jname, OpenHRP.RobotHardwareService.SWITCH_ON)
        time.sleep(2) # wait for gain transition
        return True

    def servoOff(self,jname='all'):
        c = False
        while (c != 'Y' and c != 'y'):
            c = raw_input("press 'Y' for servo OFF and power OFF. >> ")

        self.rh_svc.servo('all', OpenHRP.RobotHardwareService.SWITCH_OFF)
        time.sleep(0.2)
        if jname == 'all':
            self.rh_svc.power('all', OpenHRP.RobotHardwareService.SWITCH_OFF)
        return True

    def handServoOn(self):
        self.handServoOff()

        c = False
        while (c != 'Y' and c != 'y'):
            c = raw_input("press 'Y' for HAND servo ON and power ON. >> ")

        # reset JointGroups
        for k in ["rhand", "lhand"]:
            self.seq_svc.removeJointGroup(k)
        for k in ["rhand", "lhand"]:
            self.seq_svc.waitInterpolationOfGroup(k)
        for k in ["rhand", "lhand"]:
            self.seq_svc.addJointGroup(k, self.Groups[k])

        # go Actual (hand only)
        commandAngle = self.sh_svc.getCommand().jointRefs;
        actualAngle = self.rh_svc.getStatus().angle
        jointId = len(self.Groups["rleg"] + self.Groups["lleg"] + self.Groups["torso"] + self.Groups["head"] + self.Groups["rarm"] + self.Groups["larm"])
        commandAngle[jointId:] = actualAngle[jointId:]
        self.seq_svc.setJointAngles(commandAngle, 1.0)
        self.seq_svc.waitInterpolation()

        for j in self.Groups["rhand"] + self.Groups["lhand"]:
            self.rh_svc.power(j ,OpenHRP.RobotHardwareService.SWITCH_ON)
            time.sleep(0.01)
            self.rh_svc.servo(j ,OpenHRP.RobotHardwareService.SWITCH_ON)
        return True

    def handServoOff(self):
        c = False
        while (c != 'Y' and c != 'y'):
            c = raw_input("press 'Y' for HAND servo OFF and power OFF. >> ")

        for j in self.Groups["rhand"] + self.Groups["lhand"]:
            self.rh_svc.servo(j ,OpenHRP.RobotHardwareService.SWITCH_OFF)
            time.sleep(0.01)
            self.rh_svc.power(j ,OpenHRP.RobotHardwareService.SWITCH_OFF)
        return True

    def setResetPose(self):
        return True

    def setCollisionFreeResetPose (self):
        return True

    def servoOnWithResetPose(self):
        if self.servoOn() == True:
            self.co_svc.enableCollisionDetection()
            self.setCollisionFreeResetPose()
            print "go to collision-free-reset-pose"
            self.seq_svc.waitInterpolation()
            self.setResetPose()
            print "go to reset-pose"
            self.seq_svc.waitInterpolation()
            self.co_svc.disableCollisionDetection() # for torque control

    def removeForceSensorOffsetRMFO(self, sensor_names=[], tm=8.0):
        return self.rmfo_svc.removeForceSensorOffset(sensor_names, tm)

    def startST():
        self.akst_svc.startStabilizer()
        #self.co_svc.disableCollisionDetection()
        self.es_svc.startTorque()

    def stopST():
        self.es_svc.stopMotion("all")
        self.akst_svc.stopStabilizer()
        self.co_svc.enableCollisionDetection()

    def setupLogger(self):
        self.log_svc.add("TimedDoubleSeq","sh_qOut")
        rtm.connectPorts(rtm.findRTC("sh").port("qOut"),rtm.findRTC("log").port("sh_qOut"))
        self.log_svc.add("TimedDoubleSeq","RobotHardware0_q")
        rtm.connectPorts(rtm.findRTC("RobotHardware0").port("q"),rtm.findRTC("log").port("RobotHardware0_q"))
        self.log_svc.add("TimedDoubleSeq","RobotHardware0_dq")
        rtm.connectPorts(rtm.findRTC("RobotHardware0").port("dq"),rtm.findRTC("log").port("RobotHardware0_dq"))
        self.log_svc.add("TimedDoubleSeq","RobotHardware0_tau")
        rtm.connectPorts(rtm.findRTC("RobotHardware0").port("tau"),rtm.findRTC("log").port("RobotHardware0_tau"))
        self.log_svc.add("TimedLongSeqSeq","RobotHardware0_servoState")
        rtm.connectPorts(rtm.findRTC("RobotHardware0").port("servoState"),rtm.findRTC("log").port("RobotHardware0_servoState"))
        for sen in ["rfsensor", "lfsensor", "rhsensor", "lhsensor"]:
            self.log_svc.add("TimedDoubleSeq","RobotHardware0_" + sen)
            rtm.connectPorts(rtm.findRTC("RobotHardware0").port(sen),rtm.findRTC("log").port("RobotHardware0_" + sen))
        self.log_svc.add("TimedOrientation3D","kf_rpy")
        rtm.connectPorts(rtm.findRTC("kf").port("rpy"),rtm.findRTC("log").port("kf_rpy"))
        self.log_svc.add("TimedDoubleSeq","akst_genTauOut")
        rtm.connectPorts(rtm.findRTC("akst").port("genTauOut"),rtm.findRTC("log").port("akst_genTauOut"))
        self.log_svc.add("TimedDoubleSeq","el_q")
        rtm.connectPorts(rtm.findRTC("el").port("q"),rtm.findRTC("log").port("el_q"))
        self.log_svc.maxLength(500*60)
        self.log_svc.clear()

    def init(self):
        for j in self.Groups["rhand"] + self.Groups["lhand"]:
            self.rh_svc.setServoErrorLimit(j, 0.0)

        for j in self.Groups["rleg"] + self.Groups["lleg"] + self.Groups["torso"] + self.Groups["head"] + self.Groups["rarm"] + self.Groups["larm"]:
            self.rh_svc.setJointControlMode(j,OpenHRP.RobotHardwareService.TORQUE)
            self.rh_svc.setServoTorqueGainPercentage(j,100)

        self.setupLogger()
