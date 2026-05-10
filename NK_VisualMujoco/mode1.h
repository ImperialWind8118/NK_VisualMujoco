#pragma once
#include "app_common.h"

void mode1_init(mjModel* m, mjData* d);
void mode1_step(mjModel* m, mjData* d);
void mode1_inject_geoms(mjvScene& scn);
void mode1_render_ui();
void mode1_cleanup();

// 供 main.cpp 的鼠标回调调用
void mode1_ctrl_click(GLFWwindow* window, int button, int act, double cx, double cy);
void mode1_ctrl_drag(double dx, double dy, int height);
void mode1_release_perturb();