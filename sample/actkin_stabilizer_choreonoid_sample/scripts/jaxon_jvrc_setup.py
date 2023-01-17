#!/usr/bin/env python

from actkin_stabilizer_choreonoid_sample.actkin_stabilizer_setup import *

class JAXON_JVRC_Configurator(ActKinStabilizer_Configurator):
    def __init__(self, *args, **kwargs):
        super(JAXON_JVRC_Configurator, self).__init__(*args, **kwargs)
        self.Groups = {'rleg': ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5'],
                       'lleg': ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5'],
                       'torso': ['CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2'],
                       'head': ['HEAD_JOINT0', 'HEAD_JOINT1'],
                       'rarm': ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7'],
                       'larm': ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7'],
                       "rhand": ['RARM_F_JOINT0', 'RARM_F_JOINT1'],
                       "lhand": ['LARM_F_JOINT0', 'LARM_F_JOINT1']}

    def setResetPose(self):
        self.seq_svc.setJointAngles([0.000128, -0.002474, -0.488908, 1.01524, -0.526335, 0.002474, 0.000128, -0.002474, -0.488869, 1.01524,-0.526374, 0.002474,0.0,0.0,0.0,0.0,0.0,0.0,0.698132,-0.349066,-0.087266,-1.39626,0.0,0.0,-0.349066,0.0,0.698132,0.349066,0.087266,-1.39626,0.0,0.0,-0.349066]+[0.0]*4, 5.0)
        return True

    def setCollisionFreeResetPose (self):
        self.seq_svc.setJointAngles([0.000128, -0.002474, -0.488908, 1.01524, -0.526335, 0.002474, 0.000128, -0.002474, -0.488869, 1.01524,-0.526374, 0.002474,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.523599,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.523599,0.0,0.0,0.0,0.0,0.0]+[0.0]*4,10.0)
        return True

    def setAkstParametersJAXON(self):
        # akst setting
        akstp=self.akst_svc.getActKinStabilizerParam()[1]
        akstp.controllable_joints = self.Groups["rleg"] + self.Groups["lleg"] + self.Groups["torso"] + self.Groups["head"] + self.Groups["rarm"] + self.Groups["larm"]
        # remove hand joints
        akstp.dq_weight[len(self.Groups["rleg"] + self.Groups["lleg"]):len(self.Groups["rleg"] + self.Groups["lleg"] + self.Groups["torso"])] = [1e2]*len(self.Groups["torso"]) # reduce chest joint move
        self.akst_svc.setActKinStabilizerParam(akstp)
        # kf setting
        kfp=self.kf_svc.getKalmanFilterParam()[1]
        kfp.R_angle=1000
        self.kf_svc.setKalmanFilterParam(kfp)

    def init(self):
        super(JAXON_JVRC_Configurator, self).init()
        self.setAkstParametersJAXON()
        self.akst_svc.startAutoBalancer()
        self.akst_svc.startStabilizer()

if __name__ == '__main__':
    from hrpsys import rtm
    rtm.nshost = "localhost"
    rtm.nsport = "15005"
    rtm.initCORBA()

    hcf = JAXON_JVRC_Configurator()

    if len(sys.argv) > 1 and sys.argv[1] == "init":
        hcf.init()
