#pragma once
#include <mujoco/mujoco.h>

void controller_init(mjModel* model, mjData* data);

void controller_step(mjModel* model, mjData* data);