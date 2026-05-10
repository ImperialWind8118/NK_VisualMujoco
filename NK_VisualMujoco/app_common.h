#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

enum class AppMode { MENU, MODE1, MODE2 };	// 应用模式：主菜单、模式1（交互）和模式2（辨识）

// 所有全局量在 main.cpp 中定义，其他文件通过 extern 访问
extern mjModel* m;       // MuJoCo 模型（几何、关节、执行器定义）
extern mjData* d;       // MuJoCo 数据（每帧的状态：位置、速度、力）
extern mjvScene   scn;   // 渲染场景（存储所有几何体）
extern mjvCamera  cam;   // 相机参数
extern mjvOption  opt;   // 渲染选项
extern mjrContext con;   // OpenGL渲染上下文
extern mjvPerturb pert;  // 外力扰动对象（Ctrl+拖拽用）

extern AppMode currentMode;
extern float   timeScale;    // 时间流速倍率（0～3x）
extern double  baseTimestep; // 模型原始 timestep（0.002s）