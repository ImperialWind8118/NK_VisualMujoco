#pragma once
#include "app_common.h"

void mode2_init(mjModel* m, mjData* d);
void mode2_step(mjModel* m, mjData* d);
void mode2_render_ui();
void mode2_cleanup();