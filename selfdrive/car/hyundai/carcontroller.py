import os
from numpy import clip
from common.realtime import DT_CTRL
from cereal import car, log, messaging
from common.numpy_fast import interp
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.carstate import GearShifter
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfahda_mfc, create_acc_commands, create_acc_opt, create_frt_radar_opt, create_mdps12
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR, FEATURES
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.longcontrol import LongCtrlState

from selfdrive.controls.lib.pathplanner import LANE_CHANGE_SPEED_MIN

# speed controller
from selfdrive.car.hyundai.spdcontroller  import SpdController

from common.params import Params
import common.log as trace1
import common.CTime1000 as tm

VisualAlert = car.CarControl.HUDControl.VisualAlert

# TODO: adjust?
#HYUNDAI_ACCEL_LOOKUP_BP = [-1., 0., 2./3.5]
#HYUNDAI_ACCEL_LOOKUP_V = [-3.5, 0., 2.]

def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart):
  sys_warning = (visual_alert == VisualAlert.steerRequired)

  # initialize to no line visible
  sys_state = 1
  if left_lane and right_lane or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif left_lane:
    sys_state = 5
  elif right_lane:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  #if left_lane_depart:
  #  left_lane_warning = 1 if fingerprint in [CAR.GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 2
  #if right_lane_depart:
  #  right_lane_warning = 1 if fingerprint in [CAR.GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(dbc_name)
    self.steer_rate_limited = False
    self.prev_gapButton = 0
    self.sm = messaging.SubMaster(['controlsState'])

    self.lanechange_manual_timer = 0
    self.emergency_manual_timer = 0
    self.driver_steering_torque_above = False
    self.driver_steering_torque_above_timer = 100

    self.acc_standstill_timer = 0
    self.acc_standstill = False
    
    self.params = Params()
    self.gapsettingdance = int(self.params.get('OpkrCruiseGapSet'))
    self.opkr_autoresume = int(self.params.get('OpkrAutoResume'))

    self.opkr_turnsteeringdisable = int(self.params.get('OpkrTurnSteeringDisable')) == 1

    self.opkr_maxanglelimit = int(self.params.get('OpkrMaxAngleLimit'))

    self.timer1 = tm.CTime1000("time")

    self.SC = SpdController()
    self.model_speed = 0
    self.model_sum = 0

    self.safety_camera_timer = 0
    self.model_speed_range = [30, 90, 255, 300]
    self.steerMax_range = [SteerLimitParams.STEER_MAX, int(self.params.get('SteerMaxBaseAdj')), int(self.params.get('SteerMaxBaseAdj')), 0]
    self.steerDeltaUp_range = [5, int(self.params.get('SteerDeltaUpAdj')), int(self.params.get('SteerDeltaUpAdj')), 0]
    self.steerDeltaDown_range = [10, int(self.params.get('SteerDeltaDownAdj')), int(self.params.get('SteerDeltaDownAdj')), 0]

    self.steerMax = int(self.params.get('SteerMaxBaseAdj'))
    self.steerDeltaUp = int(self.params.get('SteerDeltaUpAdj'))
    self.steerDeltaDown = int(self.params.get('SteerDeltaDownAdj'))

    self.variable_steer_max = int(self.params.get('OpkrVariableSteerMax')) == 1
    self.variable_steer_delta = int(self.params.get('OpkrVariableSteerDelta')) == 1

    if CP.lateralTuning.which() == 'pid':
      self.str_log2 = 'T={:0.2f}/{:0.3f}/{:0.2f}/{:0.5f}'.format(CP.lateralTuning.pid.kpV[1], CP.lateralTuning.pid.kiV[1], CP.lateralTuning.pid.kdV[0], CP.lateralTuning.pid.kf)
    elif CP.lateralTuning.which() == 'indi':
      self.str_log2 = 'T={:03.1f}/{:03.1f}/{:03.1f}/{:03.1f}'.format(CP.lateralTuning.indi.innerLoopGain, CP.lateralTuning.indi.outerLoopGain, CP.lateralTuning.indi.timeConstant, CP.lateralTuning.indi.actuatorEffectiveness)
    elif CP.lateralTuning.which() == 'lqr':
      self.str_log2 = 'T={:04.0f}/{:05.3f}/{:06.4f}'.format(CP.lateralTuning.lqr.scale, CP.lateralTuning.lqr.ki, CP.lateralTuning.lqr.dcGain)

    self.p = SteerLimitParams

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert,
             left_lane, right_lane, left_lane_depart, right_lane_depart,
             lead_visible, set_speed, lead, vTargetFuture, sm):

    param = self.p

    self.model_speed, self.model_sum = self.SC.calc_va(sm, CS.out.vEgo)

    if CS.out.vEgo > 8:
      if self.variable_steer_max:
        self.steerMax = interp(int(abs(self.model_speed)), self.model_speed_range, self.steerMax_range)
      else:
        self.steerMax = int(self.params.get('SteerMaxBaseAdj'))
      if self.variable_steer_delta:
        self.steerDeltaUp = interp(int(abs(self.model_speed)), self.model_speed_range, self.steerDeltaUp_range)
        self.steerDeltaDown = interp(int(abs(self.model_speed)), self.model_speed_range, self.steerDeltaDown_range)
      else:
        self.steerDeltaUp = int(self.params.get('SteerDeltaUpAdj'))
        self.steerDeltaDown = int(self.params.get('SteerDeltaDownAdj'))
    else:
      self.steerMax = int(self.params.get('SteerMaxBaseAdj'))
      self.steerDeltaUp = int(self.params.get('SteerDeltaUpAdj'))
      self.steerDeltaDown = int(self.params.get('SteerDeltaDownAdj'))

    param.STEER_MAX = min(SteerLimitParams.STEER_MAX, self.steerMax) # variable steermax
    param.STEER_DELTA_UP = max(int(self.params.get('SteerDeltaUpAdj')), self.steerDeltaUp) # variable deltaUp
    param.STEER_DELTA_DOWN = max(int(self.params.get('SteerDeltaDownAdj')), self.steerDeltaDown) # variable deltaDown
    #param.STEER_DELTA_UP = SteerLimitParams.STEER_DELTA_UP # fixed deltaUp
    #param.STEER_DELTA_DOWN = SteerLimitParams.STEER_DELTA_DOWN # fixed deltaDown

    # Steering Torque
    if 0 <= self.driver_steering_torque_above_timer < 100:
      new_steer = actuators.steer * self.steerMax * (self.driver_steering_torque_above_timer / 100)
    else:
      new_steer = actuators.steer * self.steerMax
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, param)
    self.steer_rate_limited = new_steer != apply_steer

    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    self.high_steer_allowed = True if self.car_fingerprint in FEATURES["allow_high_steer"] else False
    if self.opkr_maxanglelimit >= 90:
      lkas_active = enabled and abs(CS.out.steeringAngle) < self.opkr_maxanglelimit
    else:
      lkas_active = enabled

    if (( CS.out.leftBlinker and not CS.out.rightBlinker) or ( CS.out.rightBlinker and not CS.out.leftBlinker)) and CS.out.vEgo < LANE_CHANGE_SPEED_MIN:
      self.lanechange_manual_timer = 50
    if CS.out.leftBlinker and CS.out.rightBlinker:
      self.emergency_manual_timer = 50
    if self.lanechange_manual_timer:
      lkas_active = 0
    if self.lanechange_manual_timer > 0:
      self.lanechange_manual_timer -= 1
    if self.emergency_manual_timer > 0:
      self.emergency_manual_timer -= 1

    if abs(CS.out.steeringTorque) > 200 and CS.out.vEgo < LANE_CHANGE_SPEED_MIN:
      self.driver_steering_torque_above = True
    else:
      self.driver_steering_torque_above = False

    if self.driver_steering_torque_above == True:
      self.driver_steering_torque_above_timer -= 1
      if self.driver_steering_torque_above_timer <= 0:
        self.driver_steering_torque_above_timer = 0
    elif self.driver_steering_torque_above == False:
      self.driver_steering_torque_above_timer += 5
      if self.driver_steering_torque_above_timer >= 100:
        self.driver_steering_torque_above_timer = 100

    if not lkas_active:
      apply_steer = 0

    if self.prev_gapButton != CS.cruise_buttons:  # gap change.
      if CS.cruise_buttons == 3:
        self.gapsettingdance -= 1
      if self.gapsettingdance < 1:
        self.gapsettingdance = 4
      self.prev_gapButton = CS.cruise_buttons

    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning = \
      process_hud_alert(enabled, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart)

    speed_conv = CV.MS_TO_MPH if CS.is_set_speed_in_mph else CV.MS_TO_KPH
    self.clu11_speed = CS.clu11["CF_Clu_Vanz"]
    enabled_speed = 38 if CS.is_set_speed_in_mph else 55
    if self.clu11_speed > enabled_speed or not lkas_active or CS.out.gearShifter != GearShifter.drive:
      enabled_speed = self.clu11_speed

    can_sends = []
    if (frame % 10) == 0:
      # tester present - w/ no response (keeps radar disabled)
      can_sends.append([0x7D0, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 0])

    can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled,
                                   left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, 0))

    if CS.CP.mdpsHarness:  # send lkas11 bus 1 if mdps
      can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled,
                                   left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, 1))
      can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.NONE, enabled_speed, 1))


    str_log1 = 'CV={:03.0f}  TQ={:03.0f}  R={:03.0f}  ST={:03.0f}/{:01.0f}/{:01.0f}  G={:01.0f}'.format(abs(self.model_speed), abs(new_steer), self.timer1.sampleTime(), self.steerMax, self.steerDeltaUp, self.steerDeltaDown, CS.out.cruiseGapSet)

    if int(self.params.get('OpkrLiveTune')) == 1:
      if int(self.params.get('LateralControlMethod')) == 0:
        self.str_log2 = 'T={:0.2f}/{:0.3f}/{:0.2f}/{:0.5f}'.format(float(int(self.params.get('PidKp')) * 0.01), float(int(self.params.get('PidKi')) * 0.001), float(int(self.params.get('PidKd')) * 0.01), float(int(self.params.get('PidKf')) * 0.00001))
      elif int(self.params.get('LateralControlMethod')) == 1:
        self.str_log2 = 'T={:03.1f}/{:03.1f}/{:03.1f}/{:03.1f}'.format(float(int(self.params.get('InnerLoopGain')) * 0.1), float(int(self.params.get('OuterLoopGain')) * 0.1), float(int(self.params.get('TimeConstant')) * 0.1), float(int(self.params.get('ActuatorEffectiveness')) * 0.1))
      elif int(self.params.get('LateralControlMethod')) == 2:
        self.str_log2 = 'T={:04.0f}/{:05.3f}/{:06.4f}'.format(float(int(self.params.get('Scale')) * 1.0), float(int(self.params.get('LqrKi')) * 0.001), float(int(self.params.get('DcGain')) * 0.0001))

    trace1.printf1('{}  {}'.format(str_log1, self.str_log2))

    if ((self.params.get('LimitSetSpeedCamera') is not None and self.params.get('LimitSetSpeedCamera') != "0") or self.params.get('OpkrSafetyCamera') == "1") and not CS.acc_active:
      self.safety_camera_timer += 1
      if self.safety_camera_timer > 100:
        self.safety_camera_timer = 0
        os.system("logcat -c &")
        os.system("echo -n 0 > /data/params/d/OpkrSafetyCamera &")
        os.system("echo -n 0 > /data/params/d/LimitSetSpeedCamera &")
        
    if CS.out.vEgo <= 1:
      self.sm.update(0)
      long_control_state = self.sm['controlsState'].longControlState
      self.acc_standstill = True if long_control_state == LongCtrlState.stopping else False
      if self.acc_standstill == True and not CS.out.gasPressed:
        self.acc_standstill_timer += 1
        if self.acc_standstill_timer >= 200:
          self.acc_standstill_timer = 200
      elif CS.out.gasPressed:
        self.acc_standstill_timer = 0
      else:
        self.acc_standstill_timer = 0
    elif CS.out.gasPressed or CS.out.vEgo > 1:
      self.acc_standstill = False
      self.acc_standstill_timer = 0      
    else:
      self.acc_standstill = False
      self.acc_standstill_timer = 0

    if not CS.CP.openpilotLongitudinalControl:
      if pcm_cancel_cmd:
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL))
      elif CS.out.cruiseState.standstill:
        # send resume at a max freq of 10Hz
        if (frame - self.last_resume_frame)*DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL)] * 25)
          self.last_resume_frame = frame

    if frame % 2 == 0 and CS.CP.openpilotLongitudinalControl:
      accel_target = round(clip(actuators.gas - actuators.brake, -3.5, 2.0), 2)
      stopping = vTargetFuture < 0.05 and CS.out.vEgo < 0.1
      # if stopping:
      #   accel_target = -3.5
      set_speed_in_units = set_speed * (CV.MS_TO_MPH if CS.clu11["CF_Clu_SPEED_UNIT"] == 1 else CV.MS_TO_KPH)
      can_sends.extend(create_acc_commands(self.packer, enabled, accel_target, accel_target, int(frame / 2), lead, set_speed_in_units, stopping, self.gapsettingdance))

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.car_fingerprint in [CAR.SONATA, CAR.PALISADE, CAR.IONIQ, CAR.KIA_NIRO_EV, CAR.IONIQ_EV_2020]:
      can_sends.append(create_lfahda_mfc(self.packer, enabled))

    # 5 Hz ACC options
    if frame % 20 == 0 and CS.CP.openpilotLongitudinalControl:
      can_sends.extend(create_acc_opt(self.packer))

    # 2 Hz front radar options
    if frame % 50 == 0 and CS.CP.openpilotLongitudinalControl:
      can_sends.append(create_frt_radar_opt(self.packer))

    can_sends.append(create_mdps12(self.packer, frame, CS.mdps12))

    return can_sends
