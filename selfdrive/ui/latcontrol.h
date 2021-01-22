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

bool control_button_screenshot(int touch_x, int touch_y) {
  if (touch_x >= 0 && touch_x <= 150) {
    if (touch_y >= 465 && touch_y <= 615) {
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
    nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
    nvgText(s->vg,btn_xc1,btn_yc,"NAVI",NULL);
  }
}

bool latcontrol( UIState *s, int touch_x, int touch_y ) {

  bool touched = false;

  draw_control_button1(s, touch_x, touch_y);

  if ((control_button_clicked1(touch_x,touch_y)) && (s->status != STATUS_OFFROAD) && (s->limit_set_speed == 0)) {
    Params().write_db_value("LimitSetSpeed", "1", 1);
    //system("su -c 'am start -n com.gmd.hidesoftkeys/com.gmd.hidesoftkeys.MainActivity'");
    system("su -c 'am start --activity-task-on-home com.skt.tmap.ku/com.skt.tmap.activity.TmapNaviActivity'");
    touched = true;
  }

  if ((control_button_screenshot(touch_x,touch_y)) && (s->status != STATUS_OFFROAD)) {
    Params().write_db_value("LimitSetSpeed", "0", 1);
    system("su -c 'am start --activity-task-on-home ai.comma.plus.offroad/.MainActivity'");
    s->sound->play(AudibleAlert::CHIME_WARNING1);
    touched = true;
  }

  return touched;
}