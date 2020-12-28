#include <stdio.h>
#include <cmath>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
#include <math.h>
#include <poll.h>
#include <sys/mman.h>

#include "common/util.h"
#include "common/swaglog.h"
#include "common/visionimg.h"
#include "common/utilpp.h"
#include "ui.hpp"
#include "paint.hpp"

extern volatile sig_atomic_t do_exit;

int  old_cruiseSwState = 0;

int write_param_float(float param, const char* param_name, bool persistent_param) {
  char s[16];
  int size = snprintf(s, sizeof(s), "%f", param);
  return Params(persistent_param).write_db_value(param_name, s, size < sizeof(s) ? size : sizeof(s));
}

void ui_init(UIState *s) {
  s->sm = new SubMaster({"modelV2", "controlsState", "uiLayoutState", "liveCalibration", "radarState", "thermal", "frame",
                         "health", "carParams", "ubloxGnss", "driverState", "dMonitoringState", "sensorEvents",
                         "carState", "pathPlan", "gpsLocationExternal", "liveMpc","liveParameters"

#ifdef SHOW_SPEEDLIMIT                         
                        ,"liveMapData",
#endif
                         });

  s->awake_timeout = 60 * UI_FREQ;;
  s->started = false;
  s->status = STATUS_OFFROAD;
  s->scene.satelliteCount = -1;
  s->scene.batteryPercent = 50;
  read_param(&s->is_metric, "IsMetric");

  s->fb = framebuffer_init("ui", 0, true, &s->fb_w, &s->fb_h);
  assert(s->fb);

  ui_nvg_init(s);
}

static void ui_init_vision(UIState *s) {
  // Invisible until we receive a calibration message.
  s->scene.world_objects_visible = false;

  for (int i = 0; i < UI_BUF_COUNT; i++) {
    if (s->khr[i] != 0) {
      visionimg_destroy_gl(s->khr[i], s->priv_hnds[i]);
      glDeleteTextures(1, &s->frame_texs[i]);
    }

    VisionImg img = {
      .fd = s->stream.bufs[i].fd,
      .format = VISIONIMG_FORMAT_RGB24,
      .width = s->stream.bufs_info.width,
      .height = s->stream.bufs_info.height,
      .stride = s->stream.bufs_info.stride,
      .bpp = 3,
      .size = s->stream.bufs_info.buf_len,
    };
#ifndef QCOM
    s->priv_hnds[i] = s->stream.bufs[i].addr;
#endif
    s->frame_texs[i] = visionimg_to_gl(&img, &s->khr[i], &s->priv_hnds[i]);

    glBindTexture(GL_TEXTURE_2D, s->frame_texs[i]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // BGR
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, GL_GREEN);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, GL_RED);
  }
  assert(glGetError() == GL_NO_ERROR);
}

void ui_update_vision(UIState *s) {

  if (!s->vision_connected && s->started) {
    const VisionStreamType type = s->scene.frontview ? VISION_STREAM_RGB_FRONT : VISION_STREAM_RGB_BACK;
    int err = visionstream_init(&s->stream, type, true, nullptr);
    if (err == 0) {
      ui_init_vision(s);
      s->vision_connected = true;
    }
  }

  if (s->vision_connected) {
    if (!s->started) goto destroy;

    // poll for a new frame
    struct pollfd fds[1] = {{
      .fd = s->stream.ipc_fd,
      .events = POLLOUT,
    }};
    int ret = poll(fds, 1, 100);
    if (ret > 0) {
      if (!visionstream_get(&s->stream, nullptr)) goto destroy;
    }
  }

  return;

destroy:
  visionstream_destroy(&s->stream);
  s->vision_connected = false;
}

void update_sockets(UIState *s) {

  UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  if (sm.update(0) == 0){
    return;
  }

  if (s->started && sm.updated("controlsState")) {
    auto event = sm["controlsState"];
    scene.controls_state = event.getControlsState();

    // TODO: the alert stuff shouldn't be handled here
    auto alert_sound = scene.controls_state.getAlertSound();
    if (scene.alert_type.compare(scene.controls_state.getAlertType()) != 0) {
      if (alert_sound == AudibleAlert::NONE) {
        s->sound->stop();
      } else {
        s->sound->play(alert_sound);
      }
    }
    scene.alert_text1 = scene.controls_state.getAlertText1();
    scene.alert_text2 = scene.controls_state.getAlertText2();
    scene.alert_size = scene.controls_state.getAlertSize();
    scene.alert_type = scene.controls_state.getAlertType();
    auto alertStatus = scene.controls_state.getAlertStatus();
    if (alertStatus == cereal::ControlsState::AlertStatus::USER_PROMPT) {
      s->status = STATUS_WARNING;
    } else if (alertStatus == cereal::ControlsState::AlertStatus::CRITICAL) {
      s->status = STATUS_ALERT;
    } else {
      s->status = scene.controls_state.getEnabled() ? STATUS_ENGAGED : STATUS_DISENGAGED;
    }

    float alert_blinkingrate = scene.controls_state.getAlertBlinkingRate();
    if (alert_blinkingrate > 0.) {
      if (s->alert_blinked) {
        if (s->alert_blinking_alpha > 0.0 && s->alert_blinking_alpha < 1.0) {
          s->alert_blinking_alpha += (0.05*alert_blinkingrate);
        } else {
          s->alert_blinked = false;
        }
      } else {
        if (s->alert_blinking_alpha > 0.25) {
          s->alert_blinking_alpha -= (0.05*alert_blinkingrate);
        } else {
          s->alert_blinking_alpha += 0.25;
          s->alert_blinked = true;
        }
      }
    }



    if( scene.cruiseState.cruiseSwState != old_cruiseSwState )
    {
      old_cruiseSwState = scene.cruiseState.cruiseSwState;
       ui_awake_aleat( s );
    } 
    else if( scene.canErrorCounter > 0)
    {
       ui_awake_aleat( s );
    }
    scene.model_sum = scene.controls_state.getModelSum();
    scene.canErrorCounter = scene.controls_state.getCanErrorCounter();
    scene.v_cruise = scene.controls_state.getVCruise();
    scene.v_ego = scene.controls_state.getVEgo();
    scene.angleSteers = scene.controls_state.getAngleSteers();
    scene.angleSteersDes = scene.controls_state.getAngleSteersDes();
    scene.curvature = scene.controls_state.getCurvature();
    scene.engaged = scene.controls_state.getEnabled();
    scene.engageable = scene.controls_state.getEngageable();
    scene.gps_planner_active = scene.controls_state.getGpsPlannerActive();
    scene.monitoring_active = scene.controls_state.getDriverMonitoringOn();
    scene.decel_for_model = scene.controls_state.getDecelForModel();

// debug Message
    std::string user_text1 = scene.controls_state.getAlertTextMsg1();
    std::string user_text2 = scene.controls_state.getAlertTextMsg2();
    const char* va_text1 = user_text1.c_str();
    const char* va_text2 = user_text2.c_str();    
    if (va_text1) snprintf(scene.alert.text1, sizeof(scene.alert.text1), "%s", va_text1);
    else  scene.alert.text1[0] = '\0';

    if (va_text2) snprintf(scene.alert.text2, sizeof(scene.alert.text2), "%s", va_text2);
    else scene.alert.text2[0] = '\0';

    scene.kegman.output_scale = scene.controls_state.getOutput();
    scene.kegman.steerOverride = scene.controls_state.getSteerOverride();       
  }






  if (sm.updated("radarState")) {
    auto data = sm["radarState"].getRadarState();
    scene.lead_data[0] = data.getLeadOne();
    scene.lead_data[1] = data.getLeadTwo();

    scene.lead_status1 = scene.lead_data[0].getStatus();
    scene.lead_d_rel1 = scene.lead_data[0].getDRel();
    scene.lead_y_rel1 = scene.lead_data[0].getYRel();
    scene.lead_v_rel1 = scene.lead_data[0].getVRel();

    scene.lead_status2 = scene.lead_data[1].getStatus();
    scene.lead_d_rel2 = scene.lead_data[1].getDRel();
    scene.lead_y_rel2 = scene.lead_data[1].getYRel();
    scene.lead_v_rel2 = scene.lead_data[1].getVRel();        
  }
  if (sm.updated("liveCalibration")) {
    scene.world_objects_visible = true;
    auto extrinsicl = sm["liveCalibration"].getLiveCalibration().getExtrinsicMatrix();
    for (int i = 0; i < 3 * 4; i++) {
      scene.extrinsic_matrix.v[i] = extrinsicl[i];
    }
  }
  if (sm.updated("modelV2")) {
    scene.model = sm["modelV2"].getModelV2();
    scene.max_distance = fmin(scene.model.getPosition().getX()[TRAJECTORY_SIZE - 1], MAX_DRAW_DISTANCE);
    for (int ll_idx = 0; ll_idx < 4; ll_idx++) {
      if (scene.model.getLaneLineProbs().size() > ll_idx) {
        scene.lane_line_probs[ll_idx] = scene.model.getLaneLineProbs()[ll_idx];
      } else {
        scene.lane_line_probs[ll_idx] = 0.0;
      }
    }

    for (int re_idx = 0; re_idx < 2; re_idx++) {
      if (scene.model.getRoadEdgeStds().size() > re_idx) {
        scene.road_edge_stds[re_idx] = scene.model.getRoadEdgeStds()[re_idx];
      } else {
        scene.road_edge_stds[re_idx] = 1.0;
      }
    }
  }
  if (sm.updated("uiLayoutState")) {
    auto data = sm["uiLayoutState"].getUiLayoutState();
    s->active_app = data.getActiveApp();
    scene.uilayout_sidebarcollapsed = data.getSidebarCollapsed();
  }
  if (sm.updated("thermal")) {
    scene.thermal = sm["thermal"].getThermal();

    auto cpuList = scene.thermal.getCpu();
    scene.maxBatTemp = scene.thermal.getBat();
    scene.maxCpuTemp = cpuList[0]; 
    scene.kegman.cpuPerc = scene.thermal.getCpuPerc();

    scene.batteryPercent = scene.thermal.getBatteryPercent();
    scene.fanSpeed = scene.thermal.getFanSpeed();        
  }

#ifdef SHOW_SPEEDLIMIT
  if (sm.updated("liveMapData")) {
    scene.live.MapData = sm["liveMapData"].getLiveMapData();
    scene.live.map_valid = scene.live.MapData.getMapValid();
    scene.live.speedlimit = scene.live.MapData.getSpeedLimit();
    scene.live.speedlimit_valid = scene.live.MapData.getSpeedLimitValid();
    scene.live.speedlimitahead = scene.live.MapData.getSpeedLimitAhead();
    scene.live.speedlimitahead_valid = scene.live.MapData.getSpeedLimitAheadValid();
    scene.live.speedlimitaheaddistance = scene.live.MapData.getSpeedLimitAheadDistance();
  }  
#endif

  if (sm.updated("ubloxGnss")) {
    auto data = sm["ubloxGnss"].getUbloxGnss();
    if (data.which() == cereal::UbloxGnss::MEASUREMENT_REPORT) {
      scene.satelliteCount = data.getMeasurementReport().getNumMeas();
    }

    auto data2 = sm["gpsLocationExternal"].getGpsLocationExternal();
    scene.gpsAccuracyUblox = data2.getAccuracy();
    scene.altitudeUblox = data2.getAltitude();      

    if (scene.gpsAccuracyUblox > 100)
      scene.gpsAccuracyUblox = 99.99;
    else if (scene.gpsAccuracyUblox == 0)
      scene.gpsAccuracyUblox = 99.8;       
  }
  if (sm.updated("health")) {
    auto health = sm["health"].getHealth();
    scene.hwType = health.getHwType();
    s->ignition = health.getIgnitionLine() || health.getIgnitionCan();
  } else if ((s->sm->frame - s->sm->rcv_frame("health")) > 5*UI_FREQ) {
    scene.hwType = cereal::HealthData::HwType::UNKNOWN;
  }
  if (sm.updated("carParams")) {
    s->longitudinal_control = sm["carParams"].getCarParams().getOpenpilotLongitudinalControl();
  }
  if (sm.updated("driverState")) {
    scene.driver_state = sm["driverState"].getDriverState();
  }
  if (sm.updated("dMonitoringState")) {
    scene.dmonitoring_state = sm["dMonitoringState"].getDMonitoringState();
    scene.is_rhd = scene.dmonitoring_state.getIsRHD();
    scene.frontview = scene.dmonitoring_state.getIsPreview();
  } else if ((sm.frame - sm.rcv_frame("dMonitoringState")) > UI_FREQ/2) {
    scene.frontview = false;
  }
  if (sm.updated("sensorEvents")) {
    for (auto sensor : sm["sensorEvents"].getSensorEvents()) {
      if (sensor.which() == cereal::SensorEventData::LIGHT) {
        s->light_sensor = sensor.getLight();
      } else if (!s->started && sensor.which() == cereal::SensorEventData::ACCELERATION) {
        s->accel_sensor = sensor.getAcceleration().getV()[2];
      } else if (!s->started && sensor.which() == cereal::SensorEventData::GYRO_UNCALIBRATED) {
        s->gyro_sensor = sensor.getGyroUncalibrated().getV()[1];
      }
    }
  }


  if (sm.updated("carState")) 
  {
    scene.car_state = sm["carState"].getCarState();
    scene.brakePress = scene.car_state.getBrakePressed();
    scene.brakeLights = scene.car_state.getBrakeLights();

    scene.leftBlinker = scene.car_state.getLeftBlinker();
    scene.rightBlinker = scene.car_state.getRightBlinker();
    scene.getGearShifter = scene.car_state.getGearShifter();

    scene.leftBlindspot = scene.car_state.getLeftBlindspot();
    scene.rightBlindspot = scene.car_state.getRightBlindspot();

    auto cruiseState = scene.car_state.getCruiseState();

    scene.cruiseState.standstill = cruiseState.getStandstill();
    scene.cruiseState.modeSel = cruiseState.getModeSel();
    scene.cruiseState.cruiseSwState = cruiseState.getCruiseSwState();

    int  ngetGearShifter = int(scene.getGearShifter);
    switch( ngetGearShifter )
    {
      case 1 :  ui_awake_aleat( s ); break;  // D
      case 4 :  ui_awake_aleat( s ); break;  // R
    }        
  }

  if (sm.updated("liveParameters")) 
  {
    auto data = sm["liveParameters"].getLiveParameters();
    scene.liveParams.gyroBias = data.getGyroBias();
    scene.liveParams.angleOffset = data.getAngleOffset();
    scene.liveParams.angleOffsetAverage = data.getAngleOffsetAverage();
    scene.liveParams.stiffnessFactor = data.getStiffnessFactor();
    scene.liveParams.steerRatio = data.getSteerRatio();
    scene.liveParams.yawRate = data.getYawRate();
    scene.liveParams.posenetSpeed = data.getPosenetSpeed();
  }  


   if (sm.updated("pathPlan"))
   {
    scene.path_plan = sm["pathPlan"].getPathPlan();

    scene.pathPlan.laneWidth = scene.path_plan.getLaneWidth();
   // scene.pathPlan.steerRatio = scene.path_plan.getSteerRatio();
    scene.pathPlan.cProb = scene.path_plan.getCProb();
    scene.pathPlan.lProb = scene.path_plan.getLProb();
    scene.pathPlan.rProb = scene.path_plan.getRProb();
    scene.pathPlan.angleOffset = scene.path_plan.getAngleOffset();
  //  scene.pathPlan.steerActuatorDelay = scene.path_plan.getSteerActuatorDelay();

    auto l_list = scene.path_plan.getLPoly();
    auto r_list = scene.path_plan.getRPoly();

    scene.pathPlan.lPoly = l_list[3];
    scene.pathPlan.rPoly = r_list[3];
   } 

  s->started = scene.thermal.getStarted() || scene.frontview;
}

void ui_update(UIState *s) {

  update_sockets(s);
  ui_update_vision(s);

  // Handle onroad/offroad transition
  if (!s->started && s->status != STATUS_OFFROAD) {
    s->status = STATUS_OFFROAD;
    s->active_app = cereal::UiLayoutState::App::HOME;
    s->scene.uilayout_sidebarcollapsed = false;
    s->sound->stop();
  } else if (s->started && s->status == STATUS_OFFROAD) {
    s->status = STATUS_DISENGAGED;
    s->started_frame = s->sm->frame;

    s->active_app = cereal::UiLayoutState::App::NONE;
    s->scene.uilayout_sidebarcollapsed = true;
    s->alert_blinked = false;
    s->alert_blinking_alpha = 1.0;
    s->scene.alert_size = cereal::ControlsState::AlertSize::NONE;
  }

  // Handle controls/fcamera timeout
  if (s->started && !s->scene.frontview && ((s->sm)->frame - s->started_frame) > 10*UI_FREQ) {
    if ((s->sm)->rcv_frame("controlsState") < s->started_frame) {
      // car is started, but controlsState hasn't been seen at all
      s->scene.alert_text1 = "openpilot Unavailable";
      s->scene.alert_text2 = "Waiting for controls to start";
      s->scene.alert_size = cereal::ControlsState::AlertSize::MID;
    } else if (((s->sm)->frame - (s->sm)->rcv_frame("controlsState")) > 5*UI_FREQ) {
      // car is started, but controls is lagging or died
      if (s->scene.alert_text2 != "Controls Unresponsive" &&
          s->scene.alert_text1 != "Camera Malfunction") {
        s->sound->play(AudibleAlert::CHIME_WARNING_REPEAT);
        LOGE("Controls unresponsive");
      }

      s->scene.alert_text1 = "TAKE CONTROL IMMEDIATELY";
      s->scene.alert_text2 = "Controls Unresponsive";
      s->scene.alert_size = cereal::ControlsState::AlertSize::FULL;
      s->status = STATUS_ALERT;
    }

    const uint64_t frame_pkt = (s->sm)->rcv_frame("frame");
    const uint64_t frame_delayed = (s->sm)->frame - frame_pkt;
    const uint64_t since_started = (s->sm)->frame - s->started_frame;
    if ((frame_pkt > s->started_frame || since_started > 15*UI_FREQ) && frame_delayed > 5*UI_FREQ) {
      // controls is fine, but rear camera is lagging or died
      s->scene.alert_text1 = "Camera Malfunction";
      s->scene.alert_text2 = "Contact Support";
      s->scene.alert_size = cereal::ControlsState::AlertSize::FULL;
      s->status = STATUS_DISENGAGED;
      s->sound->stop();
    }
  }

  // Read params
  if ((s->sm)->frame % (5*UI_FREQ) == 0) {
    read_param(&s->is_metric, "IsMetric");
  } else if ((s->sm)->frame % (6*UI_FREQ) == 0) {
    int param_read = read_param(&s->last_athena_ping, "LastAthenaPingTime");
    if (param_read != 0) { // Failed to read param
      s->scene.athenaStatus = NET_DISCONNECTED;
    } else if (nanos_since_boot() - s->last_athena_ping < 70e9) {
      s->scene.athenaStatus = NET_CONNECTED;
    } else {
      s->scene.athenaStatus = NET_ERROR;
    }
  }
}
