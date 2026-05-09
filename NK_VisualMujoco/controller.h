#pragma once
#include <mujoco/mujoco.h>

// 控制器初始化（程序启动时调用一次）
void controller_init(mjModel* model, mjData* data);

// 控制器每帧更新（在 mj_step 之前调用）
void controller_step(mjModel* model, mjData* data);