import math
import numpy as np
from cereal import car, log
import cereal.messaging as messaging
from selfdrive.config import Conversions as CV
from common.numpy_fast import clip
import common.MoveAvg as moveavg1

MAX_SPEED = 255.0

class SpdController():
    def __init__(self, CP=None):
        self.path_x = np.arange(192)
        self.l_poly = []
        self.r_poly = []
        self.movAvg = moveavg1.MoveAvg()


    def calc_va(self, sm, v_ego):
        md = sm['model']
        if len(md.path.poly):
            path = list(md.path.poly)

            self.l_poly = np.array(md.leftLane.poly)
            self.r_poly = np.array(md.rightLane.poly)
            #self.p_poly = np.array(md.path.poly)

            # Curvature of polynomial https://en.wikipedia.org/wiki/Curvature#Curvature_of_the_graph_of_a_function
            # y = a x^3 + b x^2 + c x + d, y' = 3 a x^2 + 2 b x + c, y'' = 6 a x + 2 b
            # k = y'' / (1 + y'^2)^1.5
            # TODO: compute max speed without using a list of points and without numpy
            y_p = 3 * path[0] * self.path_x**2 + \
                2 * path[1] * self.path_x + path[2]
            y_pp = 6 * path[0] * self.path_x + 2 * path[1]
            curv = y_pp / (1. + y_p**2)**1.5

            a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
            v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
            model_speed = np.min(v_curvature)
            # Don't slow down below 20mph
            model_speed = max(30.0 * CV.KPH_TO_MS, model_speed)

            model_sum = curv[2] * 1000.  #np.sum( curv, 0 )

            model_speed = model_speed * CV.MS_TO_KPH
            if model_speed > MAX_SPEED:
                model_speed = MAX_SPEED
        else:
            model_speed = MAX_SPEED
            model_sum = 0

        model_speed = self.movAvg.get_min(model_speed, 10)

        return model_speed, model_sum