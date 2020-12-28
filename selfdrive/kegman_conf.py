import json
import os


json_file_name = '/data/atom_080.json'

class kegman_conf():
  def __init__(self, CP=None):
    self.config = None
    self.init = { 
        "ap_autoReasume": 1,
        "ap_autoScnOffTime": 10,
        "tun_type": "lqr",
        "cv_KPH": [10,30],
        "cv_BPV": [[30,80,255],[100,150,255]],
        "cv_sMaxV": [[150,130,100],[255,255,200]],
        "cv_sdDNV": [[2,2,1],[7,7,5]],
        "cv_sdUPV": [[1,1,1],[3,3,3]],
        "sR_KPH": [30,60],
        "sR_BPV": [[-5,0,5],[-5,0,5]],
        "sR_lqr_kiV": [[0.0,0.0,0.0],[0.02,0.02,0.02]],
        "sR_lqr_scaleV": [[1900,2200,1900],[1800,2000,1800]],
        "sR_pid_KiV": [[0.02,0.01,0.02],[0.03,0.02,0.03]],
        "sR_pid_KpV": [[0.2,0.15,0.2],[0.25,0.2,0.25]],
        "sR_pid_deadzone": 0.1,
        "sR_steerRatioV": [[15.1,15.2,15.1],[15.3,15.6,15.3]],
        "sR_ActuatorDelayV": [[0.1,0.2,0.1],[0.1,0.3,0.1]],
        "steerLimitTimer": 0.8,
        "steerOffset": 0.0,
        "steerRateCost": 0.5,
        "cameraOffset": 0.05
         }


  def data_check(self, name, value ):
    if name not in self.config:
        self.config.update({name:value})
        self.element_updated = True


  def read_config(self):
    self.element_updated = False

    if os.path.isfile( json_file_name ):
      with open( json_file_name, 'r') as f:
        str_kegman = f.read()
        print( str_kegman )
        self.config = json.loads(str_kegman)

      for name in self.init:
        self.data_check( name, self.init[name] )

      if self.element_updated:
        print("updated")
        self.write_config(self.config)

    else:
      self.config = self.init      
      self.write_config(self.config)

    return self.config

  def write_config(self, config):
    try:
      with open( json_file_name, 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod( json_file_name, 0o764)
    except IOError:
      os.mkdir('/data')
      with open( json_file_name, 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod( json_file_name, 0o764)
