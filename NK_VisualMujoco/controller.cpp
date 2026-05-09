#include "controller.h"

void controller_init(mjModel* model, mjData* data)
{
    // 初始化：目前什么都不做
    // 以后可以在这里设置初始关节角度、加载轨迹文件等
}

void controller_step(mjModel* model, mjData* data)
{
    // 每帧调用，在物理步进之前写入控制量
    // d->ctrl[i] 是第 i 个执行器的控制输入
    // d->qpos[i] 是第 i 个关节的当前角度（只读，用于读取状态）

    // 目前是纯被动仿真，暂时留空
    // 后续在这里加入 PD 控制、轨迹跟踪等算法
}