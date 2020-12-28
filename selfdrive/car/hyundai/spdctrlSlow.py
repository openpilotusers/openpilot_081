import math
import numpy as np
from common.numpy_fast import clip, interp

from selfdrive.car.hyundai.spdcontroller  import SpdController



class SpdctrlSlow(SpdController):
    def __init__(self, CP=None):
        super().__init__( CP )
        self.cv_Raio = 0.8
        self.cv_Dist = -4


    def update_lead(self, CS,  dRel, yRel, vRel):
        lead_set_speed = self.cruise_set_speed_kph
        lead_wait_cmd = 600
        self.seq_step_debug = 0
        
        if int(self.cruise_set_mode) != 2:
            return lead_wait_cmd, lead_set_speed

        self.seq_step_debug = 1
        if CS.lead_distance < 150:
            dRel = CS.lead_distance
            vRel = CS.lead_objspd

        dst_lead_distance = (CS.clu_Vanz*self.cv_Raio)   # 유지 거리.

        if dst_lead_distance > 100:
            dst_lead_distance = 100
        elif dst_lead_distance < 50:
            dst_lead_distance = 50

        if dRel < 150:
            self.time_no_lean = 0
            d_delta = dRel - dst_lead_distance
            lead_objspd = vRel  # 선행차량 상대속도.
        else:
            d_delta = 0
            lead_objspd = 0

        # 가속이후 속도 설정.
        if CS.driverAcc_time:
          lead_set_speed = CS.clu_Vanz
          lead_wait_cmd = 100
          self.seq_step_debug = 2
        elif CS.VSetDis > 70 and lead_objspd < -20:
            self.seq_step_debug = 3
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -2)
        elif CS.VSetDis > 60 and lead_objspd < -15:
            self.seq_step_debug = 4
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -2)      
        # 1. 거리 유지.
        elif d_delta < 0:
            # 선행 차량이 가까이 있으면.
            dVanz = dRel - CS.clu_Vanz
            self.seq_step_debug = 5
            if lead_objspd >= 0:    # 속도 유지 시점 결정.
                self.seq_step_debug = 6
                if CS.VSetDis > (CS.clu_Vanz + 10):
                    lead_wait_cmd = 200
                    lead_set_speed = CS.VSetDis - 1  # CS.clu_Vanz + 5
                    if lead_set_speed < 40:
                        lead_set_speed = 40
                else:
                    lead_set_speed = int(CS.VSetDis)

            elif lead_objspd < -30 or (dRel < 50 and CS.VSetDis > 60 and lead_objspd < -5):
                self.seq_step_debug = 7
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -2)
            elif lead_objspd < -20 or (dRel < 70 and CS.VSetDis > 60 and lead_objspd < -5):
                self.seq_step_debug = 8
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 20, -2)
            elif lead_objspd < -10:
                self.seq_step_debug = 9
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, -1)
            elif lead_objspd < 0:
                self.seq_step_debug = 10
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 80, -1)
            else:
                self.seq_step_debug = 11
                lead_set_speed = int(CS.VSetDis)

        # 선행차량이 멀리 있으면.
        elif lead_objspd < -20:
            self.seq_step_debug = 12
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -2)
        elif lead_objspd < -10:
            self.seq_step_debug = 13
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, -1)
        elif lead_objspd < -5:
            self.seq_step_debug = 14
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 150, -1)
        elif lead_objspd < -1:
            self.seq_step_debug = 15
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 200, -1)
        elif self.cruise_set_speed_kph > CS.clu_Vanz:
            self.seq_step_debug = 16
            # 선행 차량이 가속하고 있으면.
            if dRel >= 150:
                self.seq_step_debug = 17
                lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 200, 1 )
            elif lead_objspd < self.cv_Dist:
                self.seq_step_debug = 18
                lead_set_speed = int(CS.VSetDis)
            elif lead_objspd < 2:
                self.seq_step_debug = 19
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 500, 1)
            elif lead_objspd < 5:
                self.seq_step_debug = 20
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 300, 1)
            elif lead_objspd < 10:
                self.seq_step_debug = 21
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 200, 1)
            elif lead_objspd < 30:
                self.seq_step_debug = 22
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 100, 1)                
            else:
                self.seq_step_debug = 23
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, 1)

        return lead_wait_cmd, lead_set_speed

    def update_curv(self, CS, sm, model_speed):
        wait_time_cmd = 0
        set_speed = self.cruise_set_speed_kph
        v_ego_kph = CS.clu_Vanz 

        if int(self.cruise_set_mode) == 4:
            set_speed = model_speed * 0.8
            delta_spd = abs(model_speed - v_ego_kph)
            xp = [2,10,20]
            fp = [100,30,15]
            wait_time_cmd = interp( delta_spd, xp, fp )

        # 2. 커브 감속.
        elif self.cruise_set_speed_kph >= 70:
            if model_speed < 80:
                set_speed = self.cruise_set_speed_kph - 15
                wait_time_cmd = 100
            elif model_speed < 110:  # 6도
                set_speed = self.cruise_set_speed_kph - 10
                wait_time_cmd = 150
            elif model_speed < 160:  # 3 도
                set_speed = self.cruise_set_speed_kph - 5
                wait_time_cmd = 200

            if set_speed > model_speed:
                set_speed = model_speed

        return wait_time_cmd, set_speed

