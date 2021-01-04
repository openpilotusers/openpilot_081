#include <stdlib.h>
#include <time.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "common/params.h"
#include "ui.hpp"


bool control_button_clicked1(int touch_x, int touch_y) {
  if (touch_x >= 1585 && touch_x <= 1725) {
    if (touch_y >= 905 && touch_y <= 1045) {
      return true;
    }
  }
  return false;
}

bool control_button_clicked2(int touch_x, int touch_y) {
  if (touch_x >= 1425 && touch_x <= 1565) {
    if (touch_y >= 905 && touch_y <= 1045) {
      return true;
    }
  }
  return false;
}

bool control_button_clicked3(int touch_x, int touch_y) {
  if (touch_x >= 1265 && touch_x <= 1405) {
    if (touch_y >= 905 && touch_y <= 1045) {
      return true;
    }
  }
  return false;
}

bool control_button_clicked4(int touch_x, int touch_y) {
  if (touch_x >= 1105 && touch_x <= 1245) {
    if (touch_y >= 905 && touch_y <= 1045) {
      return true;
    }
  }
  return false;
}

bool control_button_clicked5(int touch_x, int touch_y) {
  if (touch_x >= 945 && touch_x <= 1085) {
    if (touch_y >= 905 && touch_y <= 1045) {
      return true;
    }
  }
  return false;
}

bool control_button_screenshot(int touch_x, int touch_y) {
  if (touch_x >= 0 && touch_x <= 800) {
    if (touch_y >= 600 && touch_y <= 1080) {
      return true;
    }
  }
  return false;
}

static void draw_control_button1(UIState *s, int touch_x, int touch_y) {
  if (s->vision_connected){
    int btn_w = 140;
    int btn_h = 140;
    int btn_x1 = 1920 - btn_w - 195;
    int btn_y = 1080 - btn_h - 35;
    int btn_xc1 = btn_x1 + (btn_w/2);
    int btn_yc = btn_y + (btn_h/2);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, 100);
    nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
    
    nvgFontSize(s->vg, 45);
    
    if (s->lat_mode == 0) {
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc,"OPA",NULL);
    } else if (s->lat_mode == 1) {
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc,"CITY",NULL);
    } else if (s->lat_mode == 2) {
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc,"HIGW",NULL);
    } else if (s->lat_mode == 3) {
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc,"ONEW",NULL);
    }
  }
}

static void draw_control_button2(UIState *s, int touch_x, int touch_y) {
  if (s->vision_connected){
    int btn_w = 140;
    int btn_h = 140;
    int btn_x2 = 1920 - btn_w - 355;
    int btn_y = 1080 - btn_h - 35;
    int btn_xc2 = btn_x2 + (btn_w/2);
    int btn_yc = btn_y + (btn_h/2);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x2, btn_y, btn_w, btn_h, 100);
    if (s->acc_mode == 0) {
      nvgStrokeColor(s->vg, nvgRGBA(55, 184, 104, 150));
    } else if (s->acc_mode == 1) {
      nvgStrokeColor(s->vg, nvgRGBA(255, 175, 3, 150));
    } else if (s->acc_mode == 2) {
      nvgStrokeColor(s->vg, nvgRGBA(135, 206, 235, 175));
    }
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
    
    nvgFontSize(s->vg, 45);

    if (s->acc_mode == 0) {
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc2,btn_yc,"NORM",NULL);
    } else if (s->acc_mode == 1) {
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc2,btn_yc,"SPRT",NULL);
    } else if (s->acc_mode == 2) {
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc2,btn_yc,"ECO",NULL);
    }
  }
}

static void draw_control_button3(UIState *s, int touch_x, int touch_y) {
  if (s->vision_connected){
    int btn_w = 140;
    int btn_h = 140;
    int btn_x3 = 1920 - btn_w - 515;
    int btn_y = 1080 - btn_h - 35;
    int btn_xc3 = btn_x3 + (btn_w/2);
    int btn_yc = btn_y + (btn_h/2);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x3, btn_y, btn_w, btn_h, 100);
    if (s->limit_set_speed_curv == 1) {
      nvgStrokeColor(s->vg, nvgRGBA(55,104,200,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      nvgFillColor(s->vg, nvgRGBA(10,10,230,80));
      nvgFill(s->vg);
    } else {
      nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
    }
    nvgFontSize(s->vg, 45);
    nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
    nvgText(s->vg,btn_xc3,btn_yc,"CURV",NULL);
  }
}

static void draw_control_button4(UIState *s, int touch_x, int touch_y) {
  if (s->vision_connected){
    int btn_w = 140;
    int btn_h = 140;
    int btn_x4 = 1920 - btn_w - 675;
    int btn_y = 1080 - btn_h - 35;
    int btn_xc4 = btn_x4 + (btn_w/2);
    int btn_yc = btn_y + (btn_h/2);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x4, btn_y, btn_w, btn_h, 100);
    if (s->limit_set_speed_camera == 1) {
      nvgStrokeColor(s->vg, nvgRGBA(184,80,70,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      nvgFillColor(s->vg, nvgRGBA(200,20,20,80));
      nvgFill(s->vg);
    } else {
      nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
    }
    nvgFontSize(s->vg, 45);
    nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
    nvgText(s->vg,btn_xc4,btn_yc,"CAM",NULL);
  }
}

static void draw_control_button5(UIState *s, int touch_x, int touch_y) {
  if (s->vision_connected){
    int btn_w = 140;
    int btn_h = 140;
    int btn_x5 = 1920 - btn_w - 835;
    int btn_y = 1080 - btn_h - 35;
    int btn_xc5 = btn_x5 + (btn_w/2);
    int btn_yc = btn_y + (btn_h/2);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x5, btn_y, btn_w, btn_h, 100);
    if (s->limit_set_speed == 1) {
      nvgStrokeColor(s->vg, nvgRGBA(55,184,104,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      nvgFillColor(s->vg, nvgRGBA(0,255,0,80));
      nvgFill(s->vg);
    } else {
      nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
    }
    nvgFontSize(s->vg, 45);
    nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
    nvgText(s->vg,btn_xc5,btn_yc,"LIM",NULL);
  }
}

bool latcontrol( UIState *s, int touch_x, int touch_y ) {

  bool touched = false;
  
  draw_control_button1(s, touch_x, touch_y);
  draw_control_button2(s, touch_x, touch_y);
  draw_control_button3(s, touch_x, touch_y);
  draw_control_button4(s, touch_x, touch_y);
  draw_control_button5(s, touch_x, touch_y);

  if ((control_button_clicked1(touch_x,touch_y)) && (s->scene.uilayout_sidebarcollapsed == true)) {
    s->lat_mode = s->lat_mode + 1;
    if (s->lat_mode > 3) {
      s->lat_mode = 0;
    }
    if (s->lat_mode == 0) {
      Params().write_db_value("OpkrLatMode", "0", 1);
    } else if (s->lat_mode == 1) {
      Params().write_db_value("OpkrLatMode", "1", 1);
    } else if (s->lat_mode == 2) {
      Params().write_db_value("OpkrLatMode", "2", 1);
    } else if (s->lat_mode == 3) {
      Params().write_db_value("OpkrLatMode", "3", 1);
    }
    touched = true;
  }
  if ((control_button_clicked2(touch_x,touch_y)) && (s->scene.uilayout_sidebarcollapsed == true)) {
    s->acc_mode = s->acc_mode + 1;
    if (s->acc_mode > 2) {
      s->acc_mode = 0;
    }
    if (s->acc_mode == 0) {
      Params().write_db_value("OpkrAccMode", "0", 1);
    } else if (s->acc_mode == 1) {
      Params().write_db_value("OpkrAccMode", "1", 1);
    } else if (s->acc_mode == 2) {
      Params().write_db_value("OpkrAccMode", "2", 1);
    }
    touched = true;
  }

  if ((control_button_clicked3(touch_x,touch_y)) && (s->scene.uilayout_sidebarcollapsed == true)) {
    if (s->limit_set_speed_curv == false) {
      s->limit_set_speed_curv = true;
      Params().write_db_value("LimitSetSpeedCurv", "1", 1);
    } else if (s->limit_set_speed_curv == true) {
      s->limit_set_speed_curv = false;
      Params().write_db_value("LimitSetSpeedCurv", "0", 1);
    }
    touched = true;
  }

  if ((control_button_clicked4(touch_x,touch_y)) && (s->scene.uilayout_sidebarcollapsed == true)) {
    if (s->limit_set_speed_camera == false) {
      s->limit_set_speed_camera = true;
      Params().write_db_value("LimitSetSpeedCamera", "1", 1);
    } else if (s->limit_set_speed_camera == true) {
      s->limit_set_speed_camera = false;
      Params().write_db_value("LimitSetSpeedCamera", "0", 1);
    }
    touched = true;
  }

  if ((control_button_clicked5(touch_x,touch_y)) && (s->scene.uilayout_sidebarcollapsed == true)) {
    if (s->limit_set_speed == false) {
      s->limit_set_speed = true;
      Params().write_db_value("LimitSetSpeed", "1", 1);
    } else if (s->limit_set_speed == true) {
      s->limit_set_speed = false;
      Params().write_db_value("LimitSetSpeed", "0", 1);
    }
    touched = true;
  }

  if ((control_button_screenshot(touch_x,touch_y)) && (s->scene.uilayout_sidebarcollapsed == true)) {
    system("su -c 'mkdir -p /data/screenshots; screencap -p /data/screenshots/sc_$(date '+%Y-%m-%d_%H%M%S').png'");
    system("su -c 'touch /data/screenshots/camdetect'");
    s->sound->play(AudibleAlert::CHIME_WARNING1);
    touched = true;
  }
  
  return touched;
}