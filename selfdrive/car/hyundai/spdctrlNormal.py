import math
import numpy as np
from common.numpy_fast import clip, interp

from selfdrive.car.hyundai.spdcontroller  import SpdController

import common.log as trace1


class SpdctrlNormal(SpdController):
    def __init__(self, CP=None):
        super().__init__( CP )
        self.cv_Raio = 0.6
        self.cv_Dist = -5

    def update_lead(self, CS,  dRel, yRel, vRel):
        lead_set_speed = self.cruise_set_speed_kph
        lead_wait_cmd = 600
        if int(self.cruise_set_mode) != 2:
            return lead_wait_cmd, lead_set_speed

        #dRel, yRel, vRel = self.get_lead( sm, CS )
        if CS.lead_distance < 150:
            dRel = CS.lead_distance
            vRel = CS.lead_objspd

        dst_lead_distance = (CS.clu_Vanz*self.cv_Raio)   # 유지 거리.
        
        if dst_lead_distance > 100:
            dst_lead_distance = 100
        elif dst_lead_distance < 30:
            dst_lead_distance = 30

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
          self.seq_step_debug = 0
          lead_wait_cmd = 15
        elif (CS.VSetDis >= 70 and lead_objspd < -30) or (lead_objspd < -40):
            self.seq_step_debug = 1
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -8)  
        elif (CS.VSetDis >= 60 and lead_objspd < -30) or (lead_objspd < -35):
            self.seq_step_debug = 2
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -6)  
        elif (CS.VSetDis >= 60 and lead_objspd < -20) or (lead_objspd < -25):
            self.seq_step_debug = 3
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -5)  
        elif (CS.VSetDis >= 50 and lead_objspd < -15) or (lead_objspd < -20):
            self.seq_step_debug = 4
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -4)    
        # 1. 거리 유지.
        elif d_delta < 0:
            # 선행 차량이  기준 유지거리보다 더 가까이 있으면.
            dVanz = dRel - CS.clu_Vanz  # 이 코드는 왜 있을까???????????????
            self.seq_step_debug = 5
            if lead_objspd >= 0:    # 속도 유지 시점 결정.
                if CS.VSetDis > (CS.clu_Vanz + 30):
                    self.seq_step_debug = 6
                    lead_wait_cmd = 15
                    lead_set_speed = CS.VSetDis - 1  # CS.clu_Vanz + 5
                    if lead_set_speed < 40:
                        lead_set_speed = 40
                else:
                    self.seq_step_debug = 7
                    #lead_set_speed = int(CS.VSetDis)
                    lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, 5)                    
                    
            elif lead_objspd < -30 or (dRel < 60 and CS.clu_Vanz > 60 and lead_objspd < -5):            
                self.seq_step_debug = 10
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -3)
            elif lead_objspd < -20 or (dRel < 80 and CS.clu_Vanz > 80 and lead_objspd < -5):            
                self.seq_step_debug = 11
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -2)
            elif lead_objspd < -10:
                self.seq_step_debug = 12
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 30, -1)
            elif lead_objspd < 0:
                self.seq_step_debug = 13
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, -1)
            else:
                self.seq_step_debug = 14
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, 1)

        # 선행차량이 멀리 있으면.
        elif lead_objspd < -20 and dRel < 50:  #거리 조건 추가
            self.seq_step_debug = 15
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -2)
        elif lead_objspd < -10 and dRel < 30:  #거리 조건 추가:
            self.seq_step_debug = 16
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 30, -1)
        elif lead_objspd < -7:
            self.seq_step_debug = 17
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, -1)
        elif self.cruise_set_speed_kph > CS.clu_Vanz:
            self.seq_step_debug = 18
            # 선행 차량이 가속하고 있으면.
            if dRel >= 150: # 감지범위 밖에 멀리 떨어져 있으면
                if CS.clu_Vanz >= 60: 
                   self.seq_step_debug = 20
                   lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 15, 3)
                else:
                   self.seq_step_debug = 21
                   lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 15, 2)
            elif lead_objspd < self.cv_Dist:
                self.seq_step_debug = 22
                lead_set_speed = int(CS.VSetDis)
            elif lead_objspd < 5:
                self.seq_step_debug = 23
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, 2)
            elif lead_objspd < 10:
                self.seq_step_debug = 24
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, 3)
            elif lead_objspd < 20:
                if CS.clu_Vanz >= 60: 
                    self.seq_step_debug = 25
                    lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 15, 5)
                else:
                    self.seq_step_debug = 26
                    lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 15, 3)
            else:
                if CS.clu_Vanz >= 70: 
                    self.seq_step_debug = 27
                    lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 15, 5)
                else:
                    self.seq_step_debug = 28
                    lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 15, 3)

            if dRel > (CS.clu_Vanz + lead_objspd) * self.cv_Raio :   # 선행차 속도를 감안한(가감속) "내차 주행 속도" 수치의 비율(cv_Raio) 보다 선행차가 멀리 있다면 가속할 수 있도록 최대 설정 속도로 설정
                self.seq_step_debug = 29
                lead_set_speed = self.cruise_set_speed_kph

        return lead_wait_cmd, lead_set_speed

    def update_curv(self, CS, sm, model_speed):
        wait_time_cmd = 0
        set_speed = self.cruise_set_speed_kph
        v_ego_kph = CS.clu_Vanz

        if int(self.cruise_set_mode) == 4:
            set_speed = model_speed
            delta_spd = abs(model_speed - v_ego_kph)
            xp = [2,10,30]
            fp = [100,30,15]
            wait_time_cmd = interp( delta_spd, xp, fp )

        # 2. 커브 감속.
        #if self.cruise_set_speed_kph >= 100:
        elif CS.clu_Vanz >= 100:            
            if model_speed < 50:
                set_speed = self.cruise_set_speed_kph - 7 
                self.seq_step_debug = 30
                wait_time_cmd = 50
            elif model_speed < 60:  
                set_speed = self.cruise_set_speed_kph - 4
                self.seq_step_debug = 31
                wait_time_cmd = 70
            elif model_speed < 80:  
                set_speed = self.cruise_set_speed_kph - 2
                self.seq_step_debug = 32
                wait_time_cmd = 100
            elif model_speed < 90:  
                set_speed = self.cruise_set_speed_kph - 1 
                self.seq_step_debug = 33
                wait_time_cmd = 150
            if set_speed > model_speed:
                self.seq_step_debug = 34
                set_speed = model_speed
        elif CS.clu_Vanz >= 85:
            if model_speed < 80:  
                set_speed = self.cruise_set_speed_kph - 2 
                self.seq_step_debug = 35
                wait_time_cmd = 70
            elif model_speed < 90:  
                set_speed = self.cruise_set_speed_kph - 1
                self.seq_step_debug = 36
                wait_time_cmd = 100
                if set_speed > model_speed:
                   self.seq_step_debug = 37
                   set_speed = model_speed
        elif CS.clu_Vanz >= 70:
            if model_speed < 50: 
                set_speed = self.cruise_set_speed_kph - 2 
                self.seq_step_debug = 38
                wait_time_cmd = 70
            elif model_speed < 70:  
                set_speed = self.cruise_set_speed_kph - 1 
                self.seq_step_debug = 39
                wait_time_cmd = 100
                if set_speed > model_speed:
                   self.seq_step_debug = 40
                   set_speed = model_speed
        else:
            if set_speed > model_speed:
                self.seq_step_debug = 41
                set_speed = model_speed

        return wait_time_cmd, set_speed


    def update_log(self, CS, set_speed, target_set_speed, long_wait_cmd ):
        str3 = 'SS={:03.0f}/{:03.0f} DAt={:03.0f}/{:03.0f}/{:03.0f} DG={:02.0f}'.format(
            set_speed,  CS.VSetDis, CS.driverAcc_time, long_wait_cmd, self.long_curv_timer, self.seq_step_debug )
        str4 = ' LD/LS={:03.0f}/{:03.0f} '.format(  CS.lead_distance, CS.lead_objspd )

        str5 = str3 +  str4
        trace1.printf2( str5 )
