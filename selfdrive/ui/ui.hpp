#pragma once
#include "messaging.hpp"

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define NANOVG_GL3_IMPLEMENTATION
#define nvgCreate nvgCreateGL3
#else
#include <GLES3/gl3.h>
#define NANOVG_GLES3_IMPLEMENTATION
#define nvgCreate nvgCreateGLES3
#endif

#include <atomic>
#include <map>
#include <string>
#include <sstream>

#include "nanovg.h"

#include "common/mat.h"
#include "common/visionipc.h"
#include "common/visionimg.h"
#include "common/framebuffer.h"
#include "common/modeldata.h"
#include "common/params.h"
#include "sound.hpp"

#define COLOR_BLACK nvgRGBA(0, 0, 0, 255)
#define COLOR_BLACK_ALPHA(x) nvgRGBA(0, 0, 0, x)
#define COLOR_WHITE nvgRGBA(255, 255, 255, 255)
#define COLOR_WHITE_ALPHA(x) nvgRGBA(255, 255, 255, x)
#define COLOR_YELLOW nvgRGBA(218, 202, 37, 255)
#define COLOR_RED nvgRGBA(201, 34, 49, 255)
#define COLOR_OCHRE nvgRGBA(218, 111, 37, 255)

#define UI_BUF_COUNT 4

#define SHOW_SPEEDLIMIT 1

typedef struct Rect {
  int x, y, w, h;
  int centerX() const { return x + w / 2; }
  int right() const { return x + w; }
  int bottom() const { return y + h; }
  bool ptInRect(int px, int py) const {
    return px >= x && px < (x + w) && py >= y && py < (y + h);
  }
} Rect;

const int sbr_w = 300;
const int bdr_s = 30;
const int header_h = 420;
const int footer_h = 280;
const Rect settings_btn = {50, 35, 200, 117};
const Rect home_btn = {60, 1080 - 180 - 40, 180, 180};

const int UI_FREQ = 20;   // Hz

const int MODEL_PATH_MAX_VERTICES_CNT = TRAJECTORY_SIZE*2;
const int TRACK_POINTS_MAX_CNT = TRAJECTORY_SIZE*4;

const int SET_SPEED_NA = 255;

const int bdr_is = 30;
const int box_y = bdr_s;
typedef enum NetStatus {
  NET_CONNECTED,
  NET_DISCONNECTED,
  NET_ERROR,
} NetStatus;

typedef enum UIStatus {
  STATUS_OFFROAD,
  STATUS_DISENGAGED,
  STATUS_ENGAGED,
  STATUS_WARNING,
  STATUS_ALERT,
} UIStatus;

static std::map<UIStatus, NVGcolor> bg_colors = {
  {STATUS_OFFROAD, nvgRGBA(0x07, 0x23, 0x39, 0xe1)},
  {STATUS_DISENGAGED, nvgRGBA(0x17, 0x33, 0x49, 0xc8)},
  {STATUS_ENGAGED, nvgRGBA(0x17, 0x86, 0x44, 0x51)},
  {STATUS_WARNING, nvgRGBA(0xDA, 0x6F, 0x25, 0x51)},
  {STATUS_ALERT, nvgRGBA(0xC9, 0x22, 0x31, 0xe1)},
};

typedef struct {
  float x[TRAJECTORY_SIZE];
  float y[TRAJECTORY_SIZE];
  float z[TRAJECTORY_SIZE];
} line;


typedef struct UIScene {

  mat4 extrinsic_matrix;      // Last row is 0 so we can use mat4.
  bool world_objects_visible;

  bool is_rhd;
  bool frontview;
  bool uilayout_sidebarcollapsed;
  // responsive layout
  Rect viz_rect;

  std::string alert_text1;
  std::string alert_text2;
  std::string alert_type;
  cereal::ControlsState::AlertSize alert_size;

  cereal::HealthData::HwType hwType;
  int satelliteCount;
  NetStatus athenaStatus;

  cereal::ThermalData::Reader thermal;
  cereal::RadarState::LeadData::Reader lead_data[2];
  cereal::ControlsState::Reader controls_state;
  cereal::DriverState::Reader driver_state;
  cereal::DMonitoringState::Reader dmonitoring_state;
  cereal::ModelDataV2::Reader model;
  line path;
  line outer_left_lane_line;
  line left_lane_line;
  line right_lane_line;
  line outer_right_lane_line;
  line left_road_edge;
  line right_road_edge;
  float max_distance;
  float lane_line_probs[4];
  float road_edge_stds[2];




  cereal::CarState::Reader car_state;
  //cereal::CarControl::Reader car_control;
  cereal::PathPlan::Reader path_plan;



  // pathcoloring
  int lead_status1;
  float lead_d_rel1, lead_y_rel1, lead_v_rel1;

  int lead_status2;
  float lead_d_rel2, lead_y_rel2, lead_v_rel2;

  struct _KEGMEN_
  {
      bool steerOverride;
      float output_scale;
      int   cpuPerc;
  } kegman;

  // dev ui
  int     batteryPercent;  
  int     nTimer;  
  float   maxCpuTemp;
  float   maxBatTemp;

  int   fanSpeed;
  float angleSteers;  
  float angleSteersDes;

  float gpsAccuracyUblox;
  float altitudeUblox;
  float model_sum;
  float v_cruise;
  float v_ego;
  float curvature;  

  int  engaged;
  int  canErrorCounter; 
  uint64_t v_cruise_update_ts;

  bool decel_for_model;
  bool engageable;
  bool monitoring_active;
  bool gps_planner_active;  


  bool  brakePress;
  bool  brakeLights;
  bool  leftBlinker;
  bool  rightBlinker;

  bool  leftBlindspot;
  bool  rightBlindspot;  



  cereal::CarState::GearShifter  getGearShifter;  

  struct _STATUS_
  {
      char text1[512];
      char text2[512];
  } alert;


  struct LateralsRatom {
    float deadzone;
    float steerOffset;
    float cameraOffset;
    int opkrAutoResume;
    int opkrAutoScreenOff;  
  };

  struct _CAR_PARAMS
  {
      float steerRatio;
      struct LateralsRatom  lateralsRatom;
  } carParams;

  struct _LiveParams
  {
    float gyroBias;
    float angleOffset;
    float angleOffsetAverage;
    float stiffnessFactor;
    float steerRatio;
    float yawRate;
    float posenetSpeed;
  } liveParams;

  struct _LIVE_
  {
    cereal::LiveMapData::Reader MapData;
    bool map_valid;
    bool speedlimit_valid;
    bool speedlimitahead_valid;
    float speedlimit;
    float speedlimitahead;
    float speedlimitaheaddistance;
  } live;

  struct _PathPlan
  {
    float laneWidth;
   // float steerRatio;
   // float steerActuatorDelay;

    float cProb;
    float lProb;
    float rProb;

    float angleOffset;

    float lPoly;
    float rPoly;
  } pathPlan;  

  struct _CRUISE_STATE
  {
    bool standstill;
    int  modeSel;
    int  cruiseSwState;
  } cruiseState;  
} UIScene;

typedef struct {
  float x, y;
} vertex_data;

typedef struct {
  vertex_data v[MODEL_PATH_MAX_VERTICES_CNT];
  int cnt;
} line_vertices_data;

typedef struct {
  vertex_data v[TRACK_POINTS_MAX_CNT];
  int cnt;
} track_vertices_data;


typedef struct UIState {
  // framebuffer
  FramebufferState *fb;
  int fb_w, fb_h;

  // NVG
  NVGcontext *vg;

  // fonts and images
  int font_sans_regular;
  int font_sans_semibold;
  int font_sans_bold;
  int img_wheel;
  int img_turn;
  int img_face;
  int img_button_settings;
  int img_button_home;
  int img_battery;
  int img_battery_charging;
  int img_network[6];

  SubMaster *sm;

  Sound *sound;
  UIStatus status;
  UIScene scene;
  cereal::UiLayoutState::App active_app;

  // vision state
  bool vision_connected;
  VisionStream stream;

  // graphics
  GLuint frame_program;
  GLuint frame_texs[UI_BUF_COUNT];
  EGLImageKHR khr[UI_BUF_COUNT];
  void *priv_hnds[UI_BUF_COUNT];

  GLint frame_pos_loc, frame_texcoord_loc;
  GLint frame_texture_loc, frame_transform_loc;
  GLuint frame_vao[2], frame_vbo[2], frame_ibo[2];
  mat4 rear_frame_mat, front_frame_mat;


  // timeouts
  int   awake_timeout;
  int   is_awake_command;
  bool  is_ego_over_limit;


  // device state
  bool  awake;
  float light_sensor, accel_sensor, gyro_sensor;

  bool started;
  bool ignition;
  bool is_metric;
  bool longitudinal_control;
  uint64_t last_athena_ping;
  uint64_t started_frame;

  bool  alert_blinked;
  float alert_blinking_alpha;

  track_vertices_data track_vertices;
  line_vertices_data lane_line_vertices[4];
  line_vertices_data road_edge_vertices[2];

  Rect video_rect;
} UIState;

void ui_init(UIState *s);
void ui_update(UIState *s);
void ui_awake_aleat(UIState *s, bool awake = true);

int write_param_float(float param, const char* param_name, bool persistent_param = false);
template <class T>
int read_param(T* param, const char *param_name, bool persistent_param = false){
  T param_orig = *param;
  char *value;
  size_t sz;

  int result = Params(persistent_param).read_db_value(param_name, &value, &sz);
  if (result == 0){
    std::string s = std::string(value, sz); // value is not null terminated
    free(value);

    // Parse result
    std::istringstream iss(s);
    iss >> *param;

    // Restore original value if parsing failed
    if (iss.fail()) {
      *param = param_orig;
      result = -1;
    }
  }
  return result;
}
