#include "ui.hpp"
#include <assert.h>
#include <map>
#include <cmath>
#include <iostream>
#include "common/util.h"
#include "common/swaglog.h"
#include <algorithm>


#define NANOVG_GLES3_IMPLEMENTATION
#include "nanovg_gl.h"
#include "nanovg_gl_utils.h"

extern "C"{
#include "common/glutil.h"
}

#include "paint.hpp"
#include "sidebar.hpp"


float  fFontSize = 0.8;

void ui_awake_aleat(UIState *s, bool awake)
{
   s->is_awake_command = awake;
}


static void ui_print(UIState *s, int x, int y,  const char* fmt, ... )
{
  //char speed_str[512];  
  char* msg_buf = NULL;
  va_list args;
  va_start(args, fmt);
  vasprintf( &msg_buf, fmt, args);
  va_end(args);

  nvgText(s->vg, x, y, msg_buf, NULL);
}

// TODO: this is also hardcoded in common/transformations/camera.py
// TODO: choose based on frame input size
#ifdef QCOM2
const float y_offset = 150.0;
const float zoom = 1.1;
const mat3 intrinsic_matrix = (mat3){{
  2648.0, 0.0, 1928.0/2,
  0.0, 2648.0, 1208.0/2,
  0.0,   0.0,   1.0
}};
#else
const float y_offset = 0.0;
const float zoom = 2.35;
const mat3 intrinsic_matrix = (mat3){{
  910., 0., 1164.0/2,
  0., 910., 874.0/2,
  0.,   0.,   1.
}};
#endif

// Projects a point in car to space to the corresponding point in full frame
// image space.
bool car_space_to_full_frame(const UIState *s, float in_x, float in_y, float in_z, float *out_x, float *out_y, float margin=0.0) {
  const vec4 car_space_projective = (vec4){{in_x, in_y, in_z, 1.}};
  // We'll call the car space point p.
  // First project into normalized image coordinates with the extrinsics matrix.
  const vec4 Ep4 = matvecmul(s->scene.extrinsic_matrix, car_space_projective);

  // The last entry is zero because of how we store E (to use matvecmul).
  const vec3 Ep = {{Ep4.v[0], Ep4.v[1], Ep4.v[2]}};
  const vec3 KEp = matvecmul3(intrinsic_matrix, Ep);

  // Project.
  *out_x = KEp.v[0] / KEp.v[2];
  *out_y = KEp.v[1] / KEp.v[2];

  return *out_x >= -margin && *out_x <= s->fb_w + margin && *out_y >= -margin && *out_y <= s->fb_h + margin;
}


static void ui_draw_text(NVGcontext *vg, float x, float y, const char* string, float size, NVGcolor color, int font){
  nvgFontFaceId(vg, font);
  nvgFontSize(vg, size*fFontSize);
  nvgFillColor(vg, color);
  nvgText(vg, x, y, string, NULL);
}

static void draw_chevron(UIState *s, float x_in, float y_in, float sz,
                          NVGcolor fillColor, NVGcolor glowColor) {
  float x, y;
  if (!car_space_to_full_frame(s, x_in, y_in, 0.0, &x, &y)) {
    return;
  }

  sz *= 30;
  sz /= (x_in / 3 + 30);
  if (sz > 30) sz = 30;
  if (sz < 15) sz = 15;

  // glow
  float g_xo = sz/5;
  float g_yo = sz/10;
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.35)+g_xo, y+sz+g_yo);
  nvgLineTo(s->vg, x, y-g_xo);
  nvgLineTo(s->vg, x-(sz*1.35)-g_xo, y+sz+g_yo);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, glowColor);
  nvgFill(s->vg);

  // chevron
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.25), y+sz);
  nvgLineTo(s->vg, x, y);
  nvgLineTo(s->vg, x-(sz*1.25), y+sz);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, fillColor);
  nvgFill(s->vg);
}

static void ui_draw_circle_image(NVGcontext *vg, float x, float y, int size, int image, NVGcolor color, float img_alpha, float angleSteers = 0) 
{
  const int img_size = size * 1.5;
  float img_rotation =  angleSteers/180*3.141592;
  int ct_pos = -size * 0.75;

  nvgBeginPath(vg);
  nvgCircle(vg, x, y + (bdr_s * 1.5), size);
  nvgFillColor(vg, color);
  nvgFill(vg);

   nvgSave( vg );
  nvgTranslate(vg,x,(y + (bdr_s*1.5)));
  nvgRotate(vg,-img_rotation);  

  ui_draw_image(vg, ct_pos, ct_pos, img_size, img_size, image, img_alpha);
  //ui_draw_image(vg, x - (img_size / 2), img_y ? img_y : y - (size / 4), img_size, img_size, image, img_alpha);
  nvgRestore(vg);
}

static void ui_draw_circle_image(NVGcontext *vg, float x, float y, int size, int image, bool active) {
  float bg_alpha = active ? 0.3f : 0.1f;
  float img_alpha = active ? 1.0f : 0.15f;
  ui_draw_circle_image(vg, x, y, size, image, nvgRGBA(0, 0, 0, (255 * bg_alpha)), img_alpha);
}

static void draw_lead(UIState *s, const cereal::RadarState::LeadData::Reader &lead){
  // Draw lead car indicator
  float fillAlpha = 0;
  float speedBuff = 10.;
  float leadBuff = 40.;
  float d_rel = lead.getDRel();
  float v_rel = lead.getVRel();
  if (d_rel < leadBuff) {
    fillAlpha = 255*(1.0-(d_rel/leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255*(-1*(v_rel/speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }
  draw_chevron(s, d_rel, lead.getYRel(), 25, nvgRGBA(201, 34, 49, fillAlpha), COLOR_YELLOW);
}

static void ui_draw_line(UIState *s, const vertex_data *v, const int cnt, NVGcolor *color, NVGpaint *paint) {
  if (cnt == 0) return;

  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, v[0].x, v[0].y);
  for (int i = 1; i < cnt; i++) {
    nvgLineTo(s->vg, v[i].x, v[i].y);
  }
  nvgClosePath(s->vg);
  if (color) {
    nvgFillColor(s->vg, *color);
  } else if (paint) {
    nvgFillPaint(s->vg, *paint);
  }
  nvgFill(s->vg);
}

static void update_track_data(UIState *s, const cereal::ModelDataV2::XYZTData::Reader &line, track_vertices_data *pvd) {
  const UIScene *scene = &s->scene;
  const float off = 0.5;
  int max_idx = 0;
  float lead_d;
  if (s->sm->updated("radarState")) {
    lead_d = scene->lead_data[0].getDRel()*2.;
  } else {
    lead_d = MAX_DRAW_DISTANCE;
  }
  float path_length = (lead_d>0.)?lead_d-fmin(lead_d*0.35, 10.):MAX_DRAW_DISTANCE;
  path_length = fmin(path_length, scene->max_distance);


  vertex_data *v = &pvd->v[0];
  const float margin = 500.0f;
  for (int i = 0; line.getX()[i] <= path_length and i < TRAJECTORY_SIZE; i++) {
    v += car_space_to_full_frame(s, line.getX()[i], -line.getY()[i] - off, -line.getZ()[i], &v->x, &v->y, margin);
    max_idx = i;
  }
  for (int i = max_idx; i >= 0; i--) {
    v += car_space_to_full_frame(s, line.getX()[i], -line.getY()[i] + off, -line.getZ()[i], &v->x, &v->y, margin);
  }
  pvd->cnt = v - pvd->v;
}

static void ui_draw_track(UIState *s, track_vertices_data *pvd) 
{
  /*
  NVGpaint track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                        COLOR_WHITE, COLOR_WHITE_ALPHA(0));
  ui_draw_line(s, &pvd->v[0], pvd->cnt, nullptr, &track_bg);
  */

 
  // kegman
  if (pvd->cnt == 0) return;

  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, pvd->v[0].x, pvd->v[0].y);
  for (int i=1; i<pvd->cnt; i++) {
    nvgLineTo(s->vg, pvd->v[i].x, pvd->v[i].y);
  }
  nvgClosePath(s->vg);

  NVGpaint track_bg;
  if (s->scene.controls_state.getEnabled()) {
    // Draw colored MPC track Kegman's
    if (s->scene.kegman.steerOverride) {
      track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
        nvgRGBA(0, 191, 255, 255), nvgRGBA(0, 95, 128, 50));
    } else {
      int torque_scale = (int)fabs(510*(float)s->scene.kegman.output_scale);
      int red_lvl = fmin(255, torque_scale);
      int green_lvl = fmin(255, 510-torque_scale);
      track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
        nvgRGBA(          red_lvl,            green_lvl,  0, 255),
        nvgRGBA((int)(0.5*red_lvl), (int)(0.5*green_lvl), 0, 50));
    }
  } else {
    // Draw white vision track
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                        COLOR_WHITE, COLOR_WHITE_ALPHA(0));
  }

  nvgFillPaint(s->vg, track_bg);
  nvgFill(s->vg); 
}

static void draw_frame(UIState *s) {
  mat4 *out_mat;
  if (s->scene.frontview) {
    glBindVertexArray(s->frame_vao[1]);
    out_mat = &s->front_frame_mat;
  } else {
    glBindVertexArray(s->frame_vao[0]);
    out_mat = &s->rear_frame_mat;
  }
  glActiveTexture(GL_TEXTURE0);

  if (s->stream.last_idx >= 0) {
    glBindTexture(GL_TEXTURE_2D, s->frame_texs[s->stream.last_idx]);
#ifndef QCOM
    // this is handled in ion on QCOM
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, s->stream.bufs_info.width, s->stream.bufs_info.height,
                 0, GL_RGB, GL_UNSIGNED_BYTE, s->priv_hnds[s->stream.last_idx]);
#endif
  }

  glUseProgram(s->frame_program);
  glUniform1i(s->frame_texture_loc, 0);
  glUniformMatrix4fv(s->frame_transform_loc, 1, GL_TRUE, out_mat->v);

  assert(glGetError() == GL_NO_ERROR);
  glEnableVertexAttribArray(0);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, (const void*)0);
  glDisableVertexAttribArray(0);
  glBindVertexArray(0);
}

static void update_line_data(UIState *s, const cereal::ModelDataV2::XYZTData::Reader &line, float off, line_vertices_data *pvd, float max_distance) {
  // TODO check that this doesn't overflow max vertex buffer
  int max_idx;
  vertex_data *v = &pvd->v[0];
  const float margin = 500.0f;
  for (int i = 0; ((i < TRAJECTORY_SIZE) and (line.getX()[i] < fmax(MIN_DRAW_DISTANCE, max_distance))); i++) {
    v += car_space_to_full_frame(s, line.getX()[i], -line.getY()[i] - off, -line.getZ()[i] + 1.22, &v->x, &v->y, margin);
    max_idx = i;
  }
  for (int i = max_idx - 1; i > 0; i--) {
    v += car_space_to_full_frame(s, line.getX()[i], -line.getY()[i] + off, -line.getZ()[i] + 1.22, &v->x, &v->y, margin);
  }
  pvd->cnt = v - pvd->v;
}


static void ui_draw_vision_lane_lines(UIState *s) {
  const UIScene *scene = &s->scene;

  // paint lanelines
  line_vertices_data *pvd_ll = &s->lane_line_vertices[0];
  for (int ll_idx = 0; ll_idx < 4; ll_idx++) {
    if (s->sm->updated("modelV2")) {
      update_line_data(s, scene->model.getLaneLines()[ll_idx], 0.025*scene->model.getLaneLineProbs()[ll_idx], pvd_ll + ll_idx, scene->max_distance);
    }
    NVGcolor color = nvgRGBAf(1.0, 1.0, 1.0, scene->lane_line_probs[ll_idx]);
    ui_draw_line(s, (pvd_ll + ll_idx)->v, (pvd_ll + ll_idx)->cnt, &color, nullptr);
  }

  // paint road edges
  line_vertices_data *pvd_re = &s->road_edge_vertices[0];
  for (int re_idx = 0; re_idx < 2; re_idx++) {
    if (s->sm->updated("modelV2")) {
      update_line_data(s, scene->model.getRoadEdges()[re_idx], 0.025, pvd_re + re_idx, scene->max_distance);
    }
    NVGcolor color = nvgRGBAf(1.0, 0.0, 0.0, std::clamp<float>(1.0-scene->road_edge_stds[re_idx], 0.0, 1.0));
    ui_draw_line(s, (pvd_re + re_idx)->v, (pvd_re + re_idx)->cnt, &color, nullptr);
  }

  // paint path
  if (s->sm->updated("modelV2")) {
    update_track_data(s, scene->model.getPosition(), &s->track_vertices);
  }
  ui_draw_track(s, &s->track_vertices);
}

// Draw all world space objects.
static void ui_draw_world(UIState *s) {
  const UIScene *scene = &s->scene;

  nvgSave(s->vg);

  // Don't draw on top of sidebar
  nvgScissor(s->vg, scene->viz_rect.x, scene->viz_rect.y, scene->viz_rect.w, scene->viz_rect.h);

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  nvgTranslate(s->vg, s->video_rect.x + s->video_rect.w / 2, s->video_rect.y + s->video_rect.h / 2 + y_offset);

  // 2) Apply same scaling as video
  nvgScale(s->vg, zoom, zoom);

  // 3) Put (0, 0) in top left corner of video
  nvgTranslate(s->vg, -intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);

  // Draw lane edges and vision/mpc tracks
  ui_draw_vision_lane_lines(s);

  // Draw lead indicators if openpilot is handling longitudinal
  //if (s->longitudinal_control) {
  if( true ) {
    if (scene->lead_data[0].getStatus()) {
      draw_lead(s, scene->lead_data[0]);
    }
    if (scene->lead_data[1].getStatus() && (std::abs(scene->lead_data[0].getDRel() - scene->lead_data[1].getDRel()) > 3.0)) {
      draw_lead(s, scene->lead_data[1]);
    }
  }
  nvgRestore(s->vg);
}

static void ui_draw_vision_maxspeed(UIState *s) {
  char maxspeed_str[32];
  float maxspeed = s->scene.controls_state.getVCruise();
  int maxspeed_calc = maxspeed * 0.6225 + 0.5;
  if (s->is_metric) {
    maxspeed_calc = maxspeed + 0.5;
  }

  bool is_cruise_set = (maxspeed != 0 && maxspeed != SET_SPEED_NA);

  int viz_maxspeed_w = 184;
  int viz_maxspeed_h = 202;
  int viz_maxspeed_x = s->scene.viz_rect.x + (bdr_s*2);
  int viz_maxspeed_y = s->scene.viz_rect.y + (bdr_s*1.5);
  int viz_maxspeed_xo = 180;


#ifdef SHOW_SPEEDLIMIT
  viz_maxspeed_w += viz_maxspeed_xo;
  viz_maxspeed_x += viz_maxspeed_w - (viz_maxspeed_xo * 2);
#else
  viz_maxspeed_xo = 0;
#endif
  //viz_maxspeed_xo = 0;

  // Draw Background
  ui_draw_rect(s->vg, viz_maxspeed_x, viz_maxspeed_y, viz_maxspeed_w, viz_maxspeed_h, COLOR_BLACK_ALPHA(100), 30);

  // Draw Border
  NVGcolor color = COLOR_WHITE_ALPHA(100);
  ui_draw_rect(s->vg, viz_maxspeed_x, viz_maxspeed_y, viz_maxspeed_w, viz_maxspeed_h, color, 20, 10);

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  const int text_x = viz_maxspeed_x + (viz_maxspeed_xo / 2) + (viz_maxspeed_w / 2);
  ui_draw_text(s->vg, text_x, 148, "MAX", 26 * 2.5, COLOR_WHITE_ALPHA(is_cruise_set ? 200 : 100), s->font_sans_regular);

  if (is_cruise_set && s->scene.controls_state.getEnabled()) {
    snprintf(maxspeed_str, sizeof(maxspeed_str), "%d", maxspeed_calc);
    ui_draw_text(s->vg, text_x, 242, maxspeed_str, 48 * 2.5, COLOR_WHITE, s->font_sans_bold);
  } else {
    ui_draw_text(s->vg, text_x, 242, "-", 42 * 2.5, COLOR_WHITE_ALPHA(100), s->font_sans_semibold);
  }
}


static void ui_draw_vision_speedlimit(UIState *s) 
{
  char speedlim_str[32];
  float speedlimit = s->scene.live.speedlimit;
  int speedlim_calc = speedlimit * 2.2369363 + 0.5;
  if (s->is_metric) {
    speedlim_calc = speedlimit * 3.6 + 0.5;
  }


  bool is_speedlim_valid = s->scene.live.speedlimit_valid;
  float hysteresis_offset = 0.5;
  if (s->is_ego_over_limit) {
    hysteresis_offset = 0.0;
  }
  s->is_ego_over_limit = is_speedlim_valid && s->scene.v_ego > (speedlimit + hysteresis_offset);

  int viz_speedlim_w = 180;
  int viz_speedlim_h = 202;
  int ui_viz_rx = s->scene.viz_rect.x;    
  int viz_speedlim_x = (ui_viz_rx + (bdr_s*2));
  int viz_speedlim_y = (box_y + (bdr_s*1.5));
  if (!is_speedlim_valid) {
    viz_speedlim_w -= 5;
    viz_speedlim_h -= 10;
    viz_speedlim_x += 9;
    viz_speedlim_y += 5;
  }
  // Draw Background
  NVGcolor color = COLOR_WHITE_ALPHA(100);
  if (is_speedlim_valid && s->is_ego_over_limit) {
    color = nvgRGBA(218, 111, 37, 180);
  } else if (is_speedlim_valid) {
    color = COLOR_WHITE;
  }
  ui_draw_rect(s->vg, viz_speedlim_x, viz_speedlim_y, viz_speedlim_w, viz_speedlim_h, color, is_speedlim_valid ? 30 : 15);

  // Draw Border
  if (is_speedlim_valid) {
    ui_draw_rect(s->vg, viz_speedlim_x, viz_speedlim_y, viz_speedlim_w, viz_speedlim_h,
                 s->is_ego_over_limit ? COLOR_OCHRE : COLOR_WHITE, 20, 10);
  }
  const float text_x = viz_speedlim_x + viz_speedlim_w / 2;
  const float text_y = viz_speedlim_y + (is_speedlim_valid ? 50 : 45);
  // Draw "Speed Limit" Text
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  color = is_speedlim_valid && s->is_ego_over_limit ? COLOR_WHITE : COLOR_BLACK;
  ui_draw_text(s->vg, text_x + (is_speedlim_valid ? 6 : 0), text_y, "SMART", 50, color, s->font_sans_semibold);
  ui_draw_text(s->vg, text_x + (is_speedlim_valid ? 6 : 0), text_y + 40, "SPEED", 50, color, s->font_sans_semibold);

  // Draw Speed Text
  color = s->is_ego_over_limit ? COLOR_WHITE : COLOR_BLACK;
  if (is_speedlim_valid) {
    snprintf(speedlim_str, sizeof(speedlim_str), "%d", speedlim_calc);
    ui_draw_text(s->vg, text_x, viz_speedlim_y + (is_speedlim_valid ? 170 : 165), speedlim_str, 42*2.3, color, s->font_sans_bold);
  } else {
    ui_draw_text(s->vg, text_x, viz_speedlim_y + (is_speedlim_valid ? 170 : 165), "N/A", 42*2.3, color, s->font_sans_semibold);
  }
}


static void ui_draw_debug(UIState *s) 
{
  UIScene &scene = s->scene;
  int ui_viz_rx = scene.viz_rect.x;
  int ui_viz_rw = scene.viz_rect.w;

  char str_msg[512];

  const int viz_speed_w = 280;
  const int viz_speed_x = ui_viz_rx+((ui_viz_rw/2)-(viz_speed_w/2));

  int  y_pos = 0;
  int  x_pos = 0; 

  

  nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BASELINE);
  nvgFontSize(s->vg, 36*1.5*fFontSize);


  ui_print( s, ui_viz_rx+10, 50, "S:%d",  s->awake_timeout );

  x_pos = ui_viz_rx + 300;
  y_pos = 100+250; 

  ui_print( s, x_pos, y_pos+0,   "sR:%.2f,  Fan:%d", scene.liveParams.steerRatio, scene.fanSpeed );
  ui_print( s, x_pos, y_pos+50,  "aO:%.2f, %.2f", scene.liveParams.angleOffset, scene.liveParams.angleOffsetAverage );
  ui_print( s, x_pos, y_pos+100, "sF:%.2f", scene.liveParams.stiffnessFactor );

  //ui_print( s, x_pos, y_pos+200, "prob:%.2f, %.2f", scene.pathPlan.lProb, scene.pathPlan.rProb );
  ui_print( s, x_pos, y_pos+200, "prob:%.2f, %.2f, %.2f, %.2f", scene.lane_line_probs[0], scene.lane_line_probs[1], scene.lane_line_probs[2], scene.lane_line_probs[3] );
  //ui_print( s, x_pos, y_pos+300, "edge:%.2f, %.2f", scene.road_edge_stds[0], scene.road_edge_stds[1] );

  ui_print( s, x_pos, y_pos+250, "lW:%.2f  cpuPerc:%d", scene.pathPlan.laneWidth, scene.kegman.cpuPerc );
  ui_print( s, x_pos, y_pos+300, "Poly:%.2f, %.2f", scene.pathPlan.lPoly, scene.pathPlan.rPoly );
  ui_print( s, x_pos, y_pos+350, "map:%d,cam:%d", scene.live.map_valid, scene.live.speedlimitahead_valid  );
  ui_print( s, x_pos, y_pos+400, "spd:%.1f, dist:%.1f", scene.live.speedlimitahead*3.6, scene.live.speedlimitaheaddistance  );

    
    
    
  

  ui_print( s, 0, 1020, "%s", scene.alert.text1 );
  ui_print( s, 0, 1078, "%s", scene.alert.text2 );


  NVGcolor nColor = COLOR_WHITE;
  x_pos = viz_speed_x + 300;
  y_pos = 120;

  nvgFontSize(s->vg, 80);
  switch( scene.cruiseState.modeSel  )
  {
    case 0: strcpy( str_msg, "0.OP MODE" ); nColor = COLOR_WHITE; break;
    case 1: strcpy( str_msg, "1.CURVE" );    nColor = nvgRGBA(200, 200, 255, 255);  break;
    case 2: strcpy( str_msg, "2.FWD CAR" );  nColor = nvgRGBA(200, 255, 255, 255);  break;
    case 3: strcpy( str_msg, "3.HYUNDAI" );  nColor = nvgRGBA(200, 255, 255, 255);  break;
    case 4: strcpy( str_msg, "4.CURVATURE" );   nColor = nvgRGBA(200, 255, 255, 255);  break;
    default :  sprintf( str_msg, "%d.NORMAL", scene.cruiseState.modeSel ); nColor = COLOR_WHITE;  break;
  }
  nvgFillColor(s->vg, nColor);  
  ui_print( s, x_pos, y_pos+80, str_msg );
}


/*
  park @1;
  drive @2;
  neutral @3;
  reverse @4;
  sport @5;
  low @6;
  brake @7;
  eco @8;
*/
static void ui_draw_gear( UIState *s )
{
  UIScene &scene = s->scene;  
  NVGcolor nColor = COLOR_WHITE;

  int  ngetGearShifter = int(scene.getGearShifter);
  int  x_pos = 1700;
  int  y_pos = 200;
  char str_msg[512];

  nvgFontSize(s->vg, 150 );
  switch( ngetGearShifter )
  {
    case 1 : strcpy( str_msg, "P" ); nColor = nvgRGBA(200, 200, 255, 255); break;
    case 2 : strcpy( str_msg, "D" ); nColor = nvgRGBA(200, 200, 255, 255); break;
    case 3 : strcpy( str_msg, "N" ); nColor = COLOR_WHITE; break;
    case 4 : strcpy( str_msg, "R" ); nColor = COLOR_RED;  break;
    case 7 : strcpy( str_msg, "B" ); break;
    default: sprintf( str_msg, "%d", ngetGearShifter ); break;
  }

  nvgFillColor(s->vg, nColor);
  ui_print( s, x_pos, y_pos, str_msg );
}


static void ui_draw_vision_speed(UIState *s) {
  const Rect &viz_rect = s->scene.viz_rect;
  float v_ego = s->scene.controls_state.getVEgo();
  float speed = v_ego * 2.2369363 + 0.5;
  if (s->is_metric){
    speed = v_ego * 3.6 + 0.5;
  }
  const int viz_speed_w = 280;
  const int viz_speed_x = viz_rect.centerX() - viz_speed_w/2;
  char speed_str[32];
  NVGcolor val_color = COLOR_WHITE;

  if( s->scene.brakePress  ) val_color = COLOR_RED;
  else if( s->scene.brakeLights ) val_color = nvgRGBA(201, 34, 49, 100);  

  nvgBeginPath(s->vg);
  nvgRect(s->vg, viz_speed_x, viz_rect.y, viz_speed_w, header_h);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

  snprintf(speed_str, sizeof(speed_str), "%d", (int)speed);
  ui_draw_text(s->vg, viz_rect.centerX(), 240, speed_str, 96*2.5, val_color, s->font_sans_bold);
  ui_draw_text(s->vg, viz_rect.centerX(), 320, s->is_metric?"km/h":"mph", 36*2.5, COLOR_WHITE_ALPHA(200), s->font_sans_regular);

  ui_draw_debug( s ); 
}

static void ui_draw_vision_event(UIState *s) 
{
  const int viz_event_w = 220;
  const int viz_event_x = s->scene.viz_rect.right() - (viz_event_w + bdr_s*2);
  const int viz_event_y = s->scene.viz_rect.y + (bdr_s*1.5);
  if (s->scene.controls_state.getDecelForModel() && s->scene.controls_state.getEnabled()) {
    // draw winding road sign
    const int img_turn_size = 160*1.5;
    ui_draw_image(s->vg, viz_event_x - (img_turn_size / 4), viz_event_y + bdr_s - 25, img_turn_size, img_turn_size, s->img_turn, 1.0f);
  } else if (s->scene.controls_state.getEngageable()) {
    // draw steering wheel
    const int bg_wheel_size = 96;
    const int bg_wheel_x = viz_event_x + (viz_event_w-bg_wheel_size);
    const int bg_wheel_y = viz_event_y + (bg_wheel_size/2);
    const NVGcolor color = bg_colors[s->status];

    ui_draw_circle_image(s->vg, bg_wheel_x, bg_wheel_y, bg_wheel_size, s->img_wheel, color, 1.0f, s->scene.angleSteers);
  }
  else
  {
    ui_draw_gear( s );
    /* code */
  }    
}


static void ui_draw_vision_face(UIState *s) {
  const int face_size = 96;
  const int face_x = (s->scene.viz_rect.x + face_size + (bdr_s * 2));
  const int face_y = (s->scene.viz_rect.bottom() - footer_h + ((footer_h - face_size) / 2));
  ui_draw_circle_image(s->vg, face_x, face_y, face_size, s->img_face, s->scene.dmonitoring_state.getFaceDetected());
}

static void ui_draw_driver_view(UIState *s) {
  const UIScene *scene = &s->scene;
  s->scene.uilayout_sidebarcollapsed = true;
  const Rect &viz_rect = s->scene.viz_rect;
  const int ff_xoffset = 32;
  const int frame_x = viz_rect.x;
  const int frame_w = viz_rect.w;
  const int valid_frame_w = 4 * viz_rect.h / 3;
  const int box_y = viz_rect.y;
  const int box_h  = viz_rect.h;
  const int valid_frame_x = frame_x + (frame_w - valid_frame_w) / 2 + ff_xoffset;

  // blackout
  NVGpaint gradient = nvgLinearGradient(s->vg, scene->is_rhd ? valid_frame_x : (valid_frame_x + valid_frame_w),
                                        box_y,
                                        scene->is_rhd ? (valid_frame_w - box_h / 2) : (valid_frame_x + box_h / 2), box_y,
                                        COLOR_BLACK, COLOR_BLACK_ALPHA(0));
  ui_draw_rect(s->vg, scene->is_rhd ? valid_frame_x : (valid_frame_x + box_h / 2), box_y, valid_frame_w - box_h / 2, box_h, gradient);
  ui_draw_rect(s->vg, scene->is_rhd ? valid_frame_x : valid_frame_x + box_h / 2, box_y, valid_frame_w - box_h / 2, box_h, COLOR_BLACK_ALPHA(144));

  // borders
  ui_draw_rect(s->vg, frame_x, box_y, valid_frame_x - frame_x, box_h, nvgRGBA(23, 51, 73, 255));
  ui_draw_rect(s->vg, valid_frame_x + valid_frame_w, box_y, frame_w - valid_frame_w - (valid_frame_x - frame_x), box_h, nvgRGBA(23, 51, 73, 255));

  // draw face box
  if (scene->dmonitoring_state.getFaceDetected()) {
    auto fxy_list = scene->driver_state.getFacePosition();
    const float face_x = fxy_list[0];
    const float face_y = fxy_list[1];
    float fbox_x;
    float fbox_y = box_y + (face_y + 0.5) * box_h - 0.5 * 0.6 * box_h / 2;;
    if (!scene->is_rhd) {
      fbox_x = valid_frame_x + (1 - (face_x + 0.5)) * (box_h / 2) - 0.5 * 0.6 * box_h / 2;
    } else {
      fbox_x = valid_frame_x + valid_frame_w - box_h / 2 + (face_x + 0.5) * (box_h / 2) - 0.5 * 0.6 * box_h / 2;
    }

    if (std::abs(face_x) <= 0.35 && std::abs(face_y) <= 0.4) {
      ui_draw_rect(s->vg, fbox_x, fbox_y, 0.6 * box_h / 2, 0.6 * box_h / 2,
                   nvgRGBAf(1.0, 1.0, 1.0, 0.8 - ((std::abs(face_x) > std::abs(face_y) ? std::abs(face_x) : std::abs(face_y))) * 0.6 / 0.375),
                   35, 10);
    } else {
      ui_draw_rect(s->vg, fbox_x, fbox_y, 0.6 * box_h / 2, 0.6 * box_h / 2, nvgRGBAf(1.0, 1.0, 1.0, 0.2), 35, 10);
    }
  }

  // draw face icon
  const int face_size = 85;
  const int x = (valid_frame_x + face_size + (bdr_s * 2)) + (scene->is_rhd ? valid_frame_w - box_h / 2:0);
  const int y = (box_y + box_h - face_size - bdr_s - (bdr_s * 1.5));
  ui_draw_circle_image(s->vg, x, y, face_size, s->img_face, scene->dmonitoring_state.getFaceDetected());
}



//BB START: functions added for the display of various items
static int bb_ui_draw_measure(UIState *s,  const char* bb_value, const char* bb_uom, const char* bb_label,
    int bb_x, int bb_y, int bb_uom_dx,
    NVGcolor bb_valueColor, NVGcolor bb_labelColor, NVGcolor bb_uomColor,
    int bb_valueFontSize, int bb_labelFontSize, int bb_uomFontSize )
    {
  //const UIScene *scene = &s->scene;
  //const UIScene *scene = &s->scene;
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  int dx = 0;
  if (strlen(bb_uom) > 0) {
    dx = (int)(bb_uomFontSize*2.5/2);
   }
  //print value
  nvgFontFace(s->vg, "sans-semibold");
  nvgFontSize(s->vg, bb_valueFontSize*2*fFontSize);
  nvgFillColor(s->vg, bb_valueColor);
  nvgText(s->vg, bb_x-dx/2, bb_y+ (int)(bb_valueFontSize*2.5)+5, bb_value, NULL);
  //print label
  nvgFontFace(s->vg, "sans-regular");
  nvgFontSize(s->vg, bb_labelFontSize*2*fFontSize);
  nvgFillColor(s->vg, bb_labelColor);
  nvgText(s->vg, bb_x, bb_y + (int)(bb_valueFontSize*2.5)+5 + (int)(bb_labelFontSize*2.5)+5, bb_label, NULL);
  //print uom
  if (strlen(bb_uom) > 0) {
      nvgSave(s->vg);
    int rx =bb_x + bb_uom_dx + bb_valueFontSize -3;
    int ry = bb_y + (int)(bb_valueFontSize*2.5/2)+25;
    nvgTranslate(s->vg,rx,ry);
    nvgRotate(s->vg, -1.5708); //-90deg in radians
    nvgFontFace(s->vg, "sans-regular");
    nvgFontSize(s->vg, (int)(bb_uomFontSize*2*fFontSize));
    nvgFillColor(s->vg, bb_uomColor);
    nvgText(s->vg, 0, 0, bb_uom, NULL);
    nvgRestore(s->vg);
  }
  return (int)((bb_valueFontSize + bb_labelFontSize)*2.5) + 5;
}


static void bb_ui_draw_measures_left(UIState *s, int bb_x, int bb_y, int bb_w ) 
{
  const UIScene *scene = &s->scene;
  int bb_rx = bb_x + (int)(bb_w/2);
  int bb_ry = bb_y;
  int bb_h = 5;
  NVGcolor lab_color = nvgRGBA(255, 255, 255, 200);
  NVGcolor uom_color = nvgRGBA(255, 255, 255, 200);
  int value_fontSize=25;
  int label_fontSize=15;
  int uom_fontSize = 15;
  int bb_uom_dx =  (int)(bb_w /2 - uom_fontSize*2.5) ;

  //add CPU temperature
  if( true ) 
  {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    float cpuTemp = scene->maxCpuTemp;

      if( cpuTemp > 80) {
        val_color = nvgRGBA(255, 188, 3, 200);
      }
      if(cpuTemp > 92) {
        val_color = nvgRGBA(255, 0, 0, 200);
      }
      // temp is alway in C * 10
      snprintf(val_str, sizeof(val_str), "%.1f", cpuTemp );
      snprintf(uom_str, sizeof(uom_str), "");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "CPU TEMP",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

   //add battery temperature
  if( true )
  {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    float batteryTemp = scene->maxBatTemp;

    if(batteryTemp > 40) {
      val_color = nvgRGBA(255, 188, 3, 200);
    }
    if(batteryTemp > 50) {
      val_color = nvgRGBA(255, 0, 0, 200);
    }
    // temp is alway in C * 1000
    snprintf(val_str, sizeof(val_str), "%.1f", batteryTemp);
    snprintf(uom_str, sizeof(uom_str), "");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "BAT TEMP",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

    //add grey panda GPS accuracy
  if (true) {
    char val_str[16];
    char uom_str[3];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    //show red/orange if gps accuracy is low
      if(scene->gpsAccuracyUblox > 1.0) {
         val_color = nvgRGBA(255, 188, 3, 200);
      }
      if(scene->gpsAccuracyUblox > 2.0) {
         val_color = nvgRGBA(255, 0, 0, 200);
      }
    // gps accuracy is always in meters
    if(scene->gpsAccuracyUblox > 99 || scene->gpsAccuracyUblox == 0) {
       snprintf(val_str, sizeof(val_str), "None");
    }else if(scene->gpsAccuracyUblox > 9.99) {
      snprintf(val_str, sizeof(val_str), "%.1f", (scene->gpsAccuracyUblox));
    }
    else {
      snprintf(val_str, sizeof(val_str), "%.2f", (scene->gpsAccuracyUblox));
    }
    snprintf(uom_str, sizeof(uom_str), "%d", (scene->satelliteCount));
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "GPS PREC",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

    //add altitude
  if (scene->gpsAccuracyUblox != 0.00) {
    char val_str[16];
    char uom_str[3];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    snprintf(val_str, sizeof(val_str), "%.1f", (s->scene.altitudeUblox));
    snprintf(uom_str, sizeof(uom_str), "m");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "ALTITUDE",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }



  //finally draw the frame
  bb_h += 20;
  nvgBeginPath(s->vg);
  nvgRoundedRect(s->vg, bb_x, bb_y, bb_w, bb_h, 20);
  nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
  nvgStrokeWidth(s->vg, 6);
  nvgStroke(s->vg);
}


static void bb_ui_draw_measures_right(UIState *s, int bb_x, int bb_y, int bb_w ) 
{
  const UIScene *scene = &s->scene;
  int bb_rx = bb_x + (int)(bb_w/2);
  int bb_ry = bb_y;
  int bb_h = 5;
  NVGcolor lab_color = nvgRGBA(255, 255, 255, 200);
  NVGcolor uom_color = nvgRGBA(255, 255, 255, 200);
  int value_fontSize=25;
  int label_fontSize=15;
  int uom_fontSize = 15;
  int bb_uom_dx =  (int)(bb_w /2 - uom_fontSize*2.5) ;

  //add visual radar relative distance
  if( true )
  {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    if (scene->lead_status1) {
      //show RED if less than 5 meters
      //show orange if less than 15 meters
      if((int)(scene->lead_d_rel1) < 15) {
        val_color = nvgRGBA(255, 188, 3, 200);
      }
      if((int)(scene->lead_d_rel1) < 5) {
        val_color = nvgRGBA(255, 0, 0, 200);
      }
      // lead car relative distance is always in meters
      snprintf(val_str, sizeof(val_str), "%d", (int)scene->lead_d_rel1);
    } else {
       snprintf(val_str, sizeof(val_str), "-");
    }
    snprintf(uom_str, sizeof(uom_str), "m   ");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "REL DIST",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

  //add visual radar relative speed
  if( true )
  {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    if (scene->lead_status1) {
      //show Orange if negative speed (approaching)
      //show Orange if negative speed faster than 5mph (approaching fast)
      if((int)(scene->lead_v_rel1) < 0) {
        val_color = nvgRGBA(255, 188, 3, 200);
      }
      if((int)(scene->lead_v_rel1) < -5) {
        val_color = nvgRGBA(255, 0, 0, 200);
      }
      // lead car relative speed is always in meters
      if (s->is_metric) {
         snprintf(val_str, sizeof(val_str), "%d", (int)(scene->lead_v_rel1 * 3.6 + 0.5));
      } else {
         snprintf(val_str, sizeof(val_str), "%d", (int)(scene->lead_v_rel1 * 2.2374144 + 0.5));
      }
    } else {
       snprintf(val_str, sizeof(val_str), "-");
    }
    if (s->is_metric) {
      snprintf(uom_str, sizeof(uom_str), "km/h");;
    } else {
      snprintf(uom_str, sizeof(uom_str), "mph");
    }
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "REL SPEED",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

  //add  steering angle
  if( true )
  {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(0, 255, 0, 200);
      //show Orange if more than 30 degrees
      //show red if  more than 50 degrees
      if(((int)(scene->angleSteers) < -30) || ((int)(scene->angleSteers) > 30)) {
        val_color = nvgRGBA(255, 175, 3, 200);
      }
      if(((int)(scene->angleSteers) < -55) || ((int)(scene->angleSteers) > 55)) {
        val_color = nvgRGBA(255, 0, 0, 200);
      }
      // steering is in degrees
      snprintf(val_str, sizeof(val_str), "%.1f",(scene->angleSteers));

      snprintf(uom_str, sizeof(uom_str), "");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "REAL STEER",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

  //add  desired steering angle
  if( true )
  {
    char val_str[50];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    if( scene->controls_state.getEnabled() ) {
      //show Orange if more than 6 degrees
      //show red if  more than 12 degrees
      if(((int)(scene->angleSteersDes) < -30) || ((int)(scene->angleSteersDes) > 30)) 
      {
        val_color = nvgRGBA(255, 255, 255, 200);
      }
      if( ((int)(scene->angleSteersDes) < -50) || ((int)(scene->angleSteersDes) > 50) ) 
      {
        val_color = nvgRGBA(255, 255, 255, 200);
      }
      // steering is in degrees
      snprintf(val_str, sizeof(val_str), "%.1f",(scene->angleSteersDes));
    } else {
       snprintf(val_str, sizeof(val_str), "-");
    }
    snprintf(uom_str, sizeof(uom_str), "");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "DESIR STEER",
      bb_rx, bb_ry, bb_uom_dx,
      val_color, lab_color, uom_color,
      value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }


  //finally draw the frame
  bb_h += 20;
  nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, bb_x, bb_y, bb_w, bb_h, 20);
    nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
}


static void bb_ui_draw_UI(UIState *s)
{
  const UIScene *scene = &s->scene;
  const int bb_dml_w = 180;
  const int bb_dml_x = (scene->viz_rect.x + (bdr_is * 2));
  const int bb_dml_y = (box_y + (bdr_is * 1.5)) + 220;

  const int bb_dmr_w = 180;
  const int bb_dmr_x = scene->viz_rect.x + scene->viz_rect.w - bb_dmr_w - (bdr_is * 2);
  const int bb_dmr_y = (box_y + (bdr_is * 1.5)) + 220;

  bb_ui_draw_measures_right(s, bb_dml_x, bb_dml_y, bb_dml_w);
  bb_ui_draw_measures_left(s, bb_dmr_x, bb_dmr_y, bb_dmr_w);
}
//BB END: functions added for the display of various items


static void ui_draw_vision_header(UIState *s) 
{
  const Rect &viz_rect = s->scene.viz_rect;
  NVGpaint gradient = nvgLinearGradient(s->vg, viz_rect.x,
                        viz_rect.y+(header_h-(header_h/2.5)),
                        viz_rect.x, viz_rect.y+header_h,
                        nvgRGBAf(0,0,0,0.45), nvgRGBAf(0,0,0,0));

  ui_draw_rect(s->vg, viz_rect.x, viz_rect.y, viz_rect.w, header_h, gradient);

  ui_draw_vision_maxspeed(s);

#ifdef SHOW_SPEEDLIMIT
  ui_draw_vision_speedlimit(s);
#endif  
  ui_draw_vision_speed(s);
  ui_draw_vision_event(s);

   //BB START: add new measures panel  const int bb_dml_w = 180;
  bb_ui_draw_UI(s);
  //BB END: add new measures panel       
}

static void ui_draw_vision_footer(UIState *s) 
{
  ui_draw_vision_face(s);
}

static void ui_draw_vision_alert(UIState *s) {
  static std::map<cereal::ControlsState::AlertSize, const int> alert_size_map = {
      {cereal::ControlsState::AlertSize::NONE, 0},
      {cereal::ControlsState::AlertSize::SMALL, 241},
      {cereal::ControlsState::AlertSize::MID, 390},
      {cereal::ControlsState::AlertSize::FULL, s->fb_h}};
  const UIScene *scene = &s->scene;
  bool longAlert1 = scene->alert_text1.length() > 15;

  NVGcolor color = bg_colors[s->status];
  color.a *= s->alert_blinking_alpha;
  int alr_s = alert_size_map[scene->alert_size];

  const int alr_x = scene->viz_rect.x - bdr_s;
  const int alr_w = scene->viz_rect.w + (bdr_s*2);
  const int alr_h = alr_s+(scene->alert_size==cereal::ControlsState::AlertSize::NONE?0:bdr_s);
  const int alr_y = s->fb_h-alr_h;

  ui_draw_rect(s->vg, alr_x, alr_y, alr_w, alr_h, color);

  NVGpaint gradient = nvgLinearGradient(s->vg, alr_x, alr_y, alr_x, alr_y+alr_h,
                                        nvgRGBAf(0.0,0.0,0.0,0.05), nvgRGBAf(0.0,0.0,0.0,0.35));
  ui_draw_rect(s->vg, alr_x, alr_y, alr_w, alr_h, gradient);

  nvgFillColor(s->vg, COLOR_WHITE);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

  if (scene->alert_size == cereal::ControlsState::AlertSize::SMALL) {
    ui_draw_text(s->vg, alr_x+alr_w/2, alr_y+alr_h/2+15, scene->alert_text1.c_str(), 40*2.5, COLOR_WHITE, s->font_sans_semibold);
  } else if (scene->alert_size == cereal::ControlsState::AlertSize::MID) {
    ui_draw_text(s->vg, alr_x+alr_w/2, alr_y+alr_h/2-45, scene->alert_text1.c_str(), 48*2.5, COLOR_WHITE, s->font_sans_bold);
    ui_draw_text(s->vg, alr_x+alr_w/2, alr_y+alr_h/2+75, scene->alert_text2.c_str(), 36*2.5, COLOR_WHITE, s->font_sans_regular);
  } else if (scene->alert_size == cereal::ControlsState::AlertSize::FULL) {
    nvgFontSize(s->vg, (longAlert1?72:96)*2.5*fFontSize);
    nvgFontFaceId(s->vg, s->font_sans_bold);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgTextBox(s->vg, alr_x, alr_y+(longAlert1?360:420), alr_w-60, scene->alert_text1.c_str(), NULL);
    nvgFontSize(s->vg, 48*2.5*fFontSize);
    nvgFontFaceId(s->vg,  s->font_sans_regular);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
    nvgTextBox(s->vg, alr_x, alr_h-(longAlert1?300:360), alr_w-60, scene->alert_text2.c_str(), NULL);
  }
}

static void ui_draw_vision_frame(UIState *s) {
  const UIScene *scene = &s->scene;
  const Rect &viz_rect = scene->viz_rect;

  // Draw video frames
  glEnable(GL_SCISSOR_TEST);
  glViewport(s->video_rect.x, s->video_rect.y, s->video_rect.w, s->video_rect.h);
  glScissor(viz_rect.x, viz_rect.y, viz_rect.w, viz_rect.h);
  draw_frame(s);
  glDisable(GL_SCISSOR_TEST);

  glViewport(0, 0, s->fb_w, s->fb_h);
}

static void ui_draw_vision(UIState *s) {
  const UIScene *scene = &s->scene;
  if (!scene->frontview) {
    // Draw augmented elements
    if (scene->world_objects_visible) {
      ui_draw_world(s);
    }
    // Set Speed, Current Speed, Status/Events
    ui_draw_vision_header(s);
    if (scene->alert_size == cereal::ControlsState::AlertSize::NONE) {
      ui_draw_vision_footer(s);
    }
  } else {
    ui_draw_driver_view(s);
  }
}

static void ui_draw_background(UIState *s) {
  const NVGcolor color = bg_colors[s->status];
  glClearColor(color.r, color.g, color.b, 1.0);
  glClear(GL_STENCIL_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
}

void ui_draw(UIState *s) {
  s->scene.viz_rect = Rect{bdr_s, bdr_s, s->fb_w - 2 * bdr_s, s->fb_h - 2 * bdr_s};
  if (!s->scene.uilayout_sidebarcollapsed) {
    s->scene.viz_rect.x += sbr_w;
    s->scene.viz_rect.w -= sbr_w;
  }

  const bool draw_vision = s->started && s->status != STATUS_OFFROAD &&
                           s->active_app == cereal::UiLayoutState::App::NONE;

  // GL drawing functions
  ui_draw_background(s);
  if (draw_vision && s->vision_connected) {
    ui_draw_vision_frame(s);
  }
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glViewport(0, 0, s->fb_w, s->fb_h);

  // NVG drawing functions - should be no GL inside NVG frame
  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  ui_draw_sidebar(s);
  if (draw_vision && s->vision_connected) {
    ui_draw_vision(s);
  }

  if (draw_vision && s->scene.alert_size != cereal::ControlsState::AlertSize::NONE) {
    ui_draw_vision_alert(s);
  }
  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}

void ui_draw_image(NVGcontext *vg, float x, float y, float w, float h, int image, float alpha){
  nvgBeginPath(vg);
  NVGpaint imgPaint = nvgImagePattern(vg, x, y, w, h, 0, image, alpha);
  nvgRect(vg, x, y, w, h);
  nvgFillPaint(vg, imgPaint);
  nvgFill(vg);
}

void ui_draw_rect(NVGcontext *vg, float x, float y, float w, float h, NVGcolor color, float r, int width) {
  nvgBeginPath(vg);
  r > 0 ? nvgRoundedRect(vg, x, y, w, h, r) : nvgRect(vg, x, y, w, h);
  if (width) {
    nvgStrokeColor(vg, color);
    nvgStrokeWidth(vg, width);
    nvgStroke(vg);
  } else {
    nvgFillColor(vg, color);
    nvgFill(vg);
  }
}

void ui_draw_rect(NVGcontext *vg, float x, float y, float w, float h, NVGpaint &paint, float r){
  nvgBeginPath(vg);
  r > 0? nvgRoundedRect(vg, x, y, w, h, r) : nvgRect(vg, x, y, w, h);
  nvgFillPaint(vg, paint);
  nvgFill(vg);
}

static const char frame_vertex_shader[] =
#ifdef NANOVG_GL3_IMPLEMENTATION
  "#version 150 core\n"
#else
  "#version 300 es\n"
#endif
  "in vec4 aPosition;\n"
  "in vec4 aTexCoord;\n"
  "uniform mat4 uTransform;\n"
  "out vec4 vTexCoord;\n"
  "void main() {\n"
  "  gl_Position = uTransform * aPosition;\n"
  "  vTexCoord = aTexCoord;\n"
  "}\n";

static const char frame_fragment_shader[] =
#ifdef NANOVG_GL3_IMPLEMENTATION
  "#version 150 core\n"
#else
  "#version 300 es\n"
#endif
  "precision mediump float;\n"
  "uniform sampler2D uTexture;\n"
  "in vec4 vTexCoord;\n"
  "out vec4 colorOut;\n"
  "void main() {\n"
  "  colorOut = texture(uTexture, vTexCoord.xy);\n"
  "}\n";

static const mat4 device_transform = {{
  1.0,  0.0, 0.0, 0.0,
  0.0,  1.0, 0.0, 0.0,
  0.0,  0.0, 1.0, 0.0,
  0.0,  0.0, 0.0, 1.0,
}};

// frame from 4/3 to 16/9 display
static const mat4 full_to_wide_frame_transform = {{
  .75,  0.0, 0.0, 0.0,
  0.0,  1.0, 0.0, 0.0,
  0.0,  0.0, 1.0, 0.0,
  0.0,  0.0, 0.0, 1.0,
}};

void ui_nvg_init(UIState *s) {
  // init drawing
#ifdef QCOM
  // on QCOM, we enable MSAA
  s->vg = nvgCreate(0);
#else
  s->vg = nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
#endif

  assert(s->vg);

  s->font_sans_regular = nvgCreateFont(s->vg, "sans-regular", "../assets/fonts/opensans_regular.ttf");
  assert(s->font_sans_regular >= 0);
  s->font_sans_semibold = nvgCreateFont(s->vg, "sans-semibold", "../assets/fonts/opensans_semibold.ttf");
  assert(s->font_sans_semibold >= 0);
  s->font_sans_bold = nvgCreateFont(s->vg, "sans-bold", "../assets/fonts/opensans_bold.ttf");
  assert(s->font_sans_bold >= 0);

  s->img_wheel = nvgCreateImage(s->vg, "../assets/img_chffr_wheel.png", 1);
  assert(s->img_wheel != 0);
  s->img_turn = nvgCreateImage(s->vg, "../assets/img_trafficSign_turn.png", 1);
  assert(s->img_turn != 0);
  s->img_face = nvgCreateImage(s->vg, "../assets/img_driver_face.png", 1);
  assert(s->img_face != 0);
  s->img_button_settings = nvgCreateImage(s->vg, "../assets/images/button_settings.png", 1);
  assert(s->img_button_settings != 0);
  s->img_button_home = nvgCreateImage(s->vg, "../assets/images/button_home.png", 1);
  assert(s->img_button_home != 0);
  s->img_battery = nvgCreateImage(s->vg, "../assets/images/battery.png", 1);
  assert(s->img_battery != 0);
  s->img_battery_charging = nvgCreateImage(s->vg, "../assets/images/battery_charging.png", 1);
  assert(s->img_battery_charging != 0);

  for(int i=0;i<=5;++i) {
    char network_asset[32];
    snprintf(network_asset, sizeof(network_asset), "../assets/images/network_%d.png", i);
    s->img_network[i] = nvgCreateImage(s->vg, network_asset, 1);
    assert(s->img_network[i] != 0);
  }

  // init gl
  s->frame_program = load_program(frame_vertex_shader, frame_fragment_shader);
  assert(s->frame_program);

  s->frame_pos_loc = glGetAttribLocation(s->frame_program, "aPosition");
  s->frame_texcoord_loc = glGetAttribLocation(s->frame_program, "aTexCoord");

  s->frame_texture_loc = glGetUniformLocation(s->frame_program, "uTexture");
  s->frame_transform_loc = glGetUniformLocation(s->frame_program, "uTransform");

  glViewport(0, 0, s->fb_w, s->fb_h);

  glDisable(GL_DEPTH_TEST);

  assert(glGetError() == GL_NO_ERROR);

  for(int i = 0; i < 2; i++) {
    float x1, x2, y1, y2;
    if (i == 1) {
      // flip horizontally so it looks like a mirror
      x1 = 0.0;
      x2 = 1.0;
      y1 = 1.0;
      y2 = 0.0;
    } else {
      x1 = 1.0;
      x2 = 0.0;
      y1 = 1.0;
      y2 = 0.0;
    }
    const uint8_t frame_indicies[] = {0, 1, 2, 0, 2, 3};
    const float frame_coords[4][4] = {
      {-1.0, -1.0, x2, y1}, //bl
      {-1.0,  1.0, x2, y2}, //tl
      { 1.0,  1.0, x1, y2}, //tr
      { 1.0, -1.0, x1, y1}, //br
    };

    glGenVertexArrays(1, &s->frame_vao[i]);
    glBindVertexArray(s->frame_vao[i]);
    glGenBuffers(1, &s->frame_vbo[i]);
    glBindBuffer(GL_ARRAY_BUFFER, s->frame_vbo[i]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(frame_coords), frame_coords, GL_STATIC_DRAW);
    glEnableVertexAttribArray(s->frame_pos_loc);
    glVertexAttribPointer(s->frame_pos_loc, 2, GL_FLOAT, GL_FALSE,
                          sizeof(frame_coords[0]), (const void *)0);
    glEnableVertexAttribArray(s->frame_texcoord_loc);
    glVertexAttribPointer(s->frame_texcoord_loc, 2, GL_FLOAT, GL_FALSE,
                          sizeof(frame_coords[0]), (const void *)(sizeof(float) * 2));
    glGenBuffers(1, &s->frame_ibo[i]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s->frame_ibo[i]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(frame_indicies), frame_indicies, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,0);
    glBindVertexArray(0);
  }

  s->video_rect = Rect{bdr_s, bdr_s, s->fb_w - 2 * bdr_s, s->fb_h - 2 * bdr_s};
  float zx = zoom * 2 * intrinsic_matrix.v[2] / s->video_rect.w;
  float zy = zoom * 2 * intrinsic_matrix.v[5] / s->video_rect.h;

  const mat4 frame_transform = {{
    zx, 0.0, 0.0, 0.0,
    0.0, zy, 0.0, -y_offset / s->video_rect.h * 2,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
  }};

  s->front_frame_mat = matmul(device_transform, full_to_wide_frame_transform);
  s->rear_frame_mat = matmul(device_transform, frame_transform);

  for(int i = 0; i < UI_BUF_COUNT; i++) {
    s->khr[i] = 0;
    s->priv_hnds[i] = NULL;
  }
}
