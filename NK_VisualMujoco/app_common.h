#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

enum class AppMode { MENU, MODE1, MODE2 };

// 所有全局量在 main.cpp 中定义，其他文件通过 extern 访问
extern mjModel* m;
extern mjData* d;
extern mjvScene   scn;
extern mjvCamera  cam;
extern mjvOption  opt;
extern mjrContext con;
extern mjvPerturb pert;

extern AppMode currentMode;
extern float   timeScale;
extern double  baseTimestep;