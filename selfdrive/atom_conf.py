
from  selfdrive.kegman_conf import kegman_conf

class AtomConf():
  def __init__(self, CP=None):
    self.kegman = kegman_conf()

    self.tun_type   = 'lqr'
    self.sR_KPH         = [0]   # Speed  kph
    self.sR_BPV         = [[-5,0,5]]
    self.sR_steerRatioV = [[13.95,13.85,13.95]]
    self.sR_ActuatorDelayV = [[0.1,0.2,0.1]]
    self.sR_pid_KiV     = [[0.02,0.01,0.02]]
    self.sR_pid_KpV     = [[0.20,0.15,0.20]]
    self.sR_pid_deadzone  = 0.1
    self.sR_lqr_kiV     = [[0.01,0.01,0.01]]
    self.sR_lqr_scaleV  = [[2000,2000,2000]]

    self.cv_KPH    = [0.]   # Speed  kph
    self.cv_BPV    = [[200., 255.]]  # CV
    self.cv_sMaxV  = [[384., 255.]]
    self.cv_sdUPV  = [[3,2]]
    self.cv_sdDNV  = [[7,5]]

    self.steerOffset = 0.0
    self.steerRateCost = 0.4
    self.steerLimitTimer = 0.8
    self.steerActuatorDelay = 0.1
    self.cameraOffset = 0.05
    self.ap_autoReasume = 1
    self.ap_autoScnOffTime = 0

    self.read_tune()


  def read_tune(self):
    conf = self.kegman.read_config()
    self.ap_autoReasume = conf['ap_autoReasume']
    self.ap_autoScnOffTime = conf['ap_autoScnOffTime']
    self.tun_type   = conf['tun_type']
    self.sR_KPH   = conf['sR_KPH']
    self.sR_BPV  = conf['sR_BPV']
    self.sR_steerRatioV  = conf['sR_steerRatioV']
    self.sR_ActuatorDelayV = conf['sR_ActuatorDelayV']    
    self.sR_pid_KiV  = conf['sR_pid_KiV']
    self.sR_pid_KpV  = conf['sR_pid_KpV']
    self.sR_pid_deadzone  = conf['sR_pid_deadzone']
    self.sR_lqr_kiV  = conf['sR_lqr_kiV']
    self.sR_lqr_scaleV  = conf['sR_lqr_scaleV']
    self.cv_KPH  = conf['cv_KPH']
    self.cv_BPV  = conf['cv_BPV']
    self.cv_sMaxV  = conf['cv_sMaxV']
    self.cv_sdUPV  = conf['cv_sdUPV']
    self.cv_sdDNV = conf['cv_sdDNV']
    self.steerOffset = conf['steerOffset']
    self.steerRateCost = conf['steerRateCost']
    self.steerLimitTimer = conf['steerLimitTimer']
    self.cameraOffset = conf['cameraOffset']
