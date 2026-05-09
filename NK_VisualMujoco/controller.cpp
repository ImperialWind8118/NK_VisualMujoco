#include "controller.h"
#include <cmath>
#include <cstring>
#include <cstdio>

static constexpr double PI = 3.14159265358979323846;
static constexpr int MAX_ACT = 30;
static constexpr double BLEND_TIME = 1.0;  // 姿态过渡时间（秒）

// ===================== 执行器索引缓存 =====================
// 在 controller_init 中通过名字查找并记录每个执行器的编号
// 这样后续代码不需要硬编码数字索引
static struct {
    int WRJ1 = -1, WRJ0 = -1;
    int FFJ3 = -1, FFJ2 = -1, FFJ1 = -1, FFJ0 = -1;   // 食指
    int MFJ3 = -1, MFJ2 = -1, MFJ1 = -1, MFJ0 = -1;   // 中指
    int RFJ3 = -1, RFJ2 = -1, RFJ1 = -1, RFJ0 = -1;   // 无名指
    int LFJ4 = -1, LFJ3 = -1, LFJ2 = -1, LFJ1 = -1, LFJ0 = -1; // 小指
    int THJ4 = -1, THJ3 = -1, THJ2 = -1, THJ1 = -1, THJ0 = -1;  // 拇指
} A;

static int nu = 0;  // 执行器总数

// ===================== 姿态系统 =====================
struct Pose {
    const char* name;          // 姿态名称（调试用）
    double ctrl[MAX_ACT];      // 每个执行器的目标角度
    double holdTime;           // 到达后保持时间（秒）
};

static Pose poses[10];
static int numPoses = 0;
static int poseIdx = 0;
static double elapsed = 0.0;
static double prevCtrl[MAX_ACT] = {};

// ===================== 工具函数 =====================

// 通过名字查找执行器编号
static int findAct(mjModel* m, const char* name) {
    int id = mj_name2id(m, mjOBJ_ACTUATOR, name);
    if (id < 0) printf("[Controller] Warning: actuator '%s' not found\n", name);
    return id;
}

// 余弦插值：输入 t∈[0,1]，输出平滑的 0→1
static double cosInterp(double t) {
    t = fmax(0.0, fmin(1.0, t));
    return 0.5 * (1.0 - cos(PI * t));
}

// 设置四指（食中无小）的弯曲程度，每根手指的 J2/J1/J0 设为相同角度
static void setFingers(Pose& p, double ff, double mf, double rf, double lf) {
    auto set3 = [&](int j2, int j1, int j0, double v) {
        if (j2 >= 0) p.ctrl[j2] = v;
        if (j1 >= 0) p.ctrl[j1] = v;
        if (j0 >= 0) p.ctrl[j0] = v;
        };
    set3(A.FFJ2, A.FFJ1, A.FFJ0, ff);
    set3(A.MFJ2, A.MFJ1, A.MFJ0, mf);
    set3(A.RFJ2, A.RFJ1, A.RFJ0, rf);
    set3(A.LFJ2, A.LFJ1, A.LFJ0, lf);
    if (A.LFJ4 >= 0 && lf > 0.1) p.ctrl[A.LFJ4] = 0.3;
}

// 设置拇指五个关节
static void setThumb(Pose& p, double j4, double j3, double j2, double j1, double j0) {
    if (A.THJ4 >= 0) p.ctrl[A.THJ4] = j4;
    if (A.THJ3 >= 0) p.ctrl[A.THJ3] = j3;
    if (A.THJ2 >= 0) p.ctrl[A.THJ2] = j2;
    if (A.THJ1 >= 0) p.ctrl[A.THJ1] = j1;
    if (A.THJ0 >= 0) p.ctrl[A.THJ0] = j0;
}

// ===================== 姿态定义 =====================
// 每个姿态就是一组目标角度 + 保持时间
// 程序运行时会按顺序循环播放，姿态之间用余弦曲线平滑过渡

static void buildPoseSequence() {

    /* 0: 张开手掌
    {
        Pose& p = poses[numPoses++];
        p.name = "Open"; p.holdTime = 1.5;
        memset(p.ctrl, 0, sizeof(p.ctrl));
    }   */

    // 1: 握拳
    {
        Pose& p = poses[numPoses++];
        p.name = "Fist"; p.holdTime = 1.5;
        memset(p.ctrl, 0, sizeof(p.ctrl));
        setFingers(p, 1.5, 1.5, 1.5, 1.5);
        setThumb(p, 0.5, 1.0, 0.0, -0.4, -1.0);
    }

    // 2: 张开
    {
        Pose& p = poses[numPoses++];
        p.name = "Open"; p.holdTime = 1.0;
        memset(p.ctrl, 0, sizeof(p.ctrl));
    }

    // 3: 剪刀手
    {
        Pose& p = poses[numPoses++];
        p.name = "Peace"; p.holdTime = 2.0;
        memset(p.ctrl, 0, sizeof(p.ctrl));
        setFingers(p, 0.0, 0.0, 1.5, 1.5);
        setThumb(p, 0.5, 1.0, 0.0, -0.4, -1.0);
    }

    // 4: 指向
    {
        Pose& p = poses[numPoses++];
        p.name = "Point"; p.holdTime = 1.5;
        memset(p.ctrl, 0, sizeof(p.ctrl));
        setFingers(p, 0.0, 1.5, 1.5, 1.5);
        setThumb(p, 0.5, 1.0, 0.0, -0.4, -1.0);
    }

    // 5: 竖大拇指
    {
        Pose& p = poses[numPoses++];
        p.name = "ThumbsUp"; p.holdTime = 2.0;
        memset(p.ctrl, 0, sizeof(p.ctrl));
        setFingers(p, 1.5, 1.5, 1.5, 1.5);
        setThumb(p, -0.5, 0.0, 0.0, 0.0, 0.0);
    }

    // 6: 回到张开
    {
        Pose& p = poses[numPoses++];
        p.name = "Open"; p.holdTime = 1.5;
        memset(p.ctrl, 0, sizeof(p.ctrl));
    }
}

// ===================== 公开接口 =====================

void controller_init(mjModel* m, mjData* d)
{
    nu = m->nu;

    // 通过名字查找所有执行器编号
    A.WRJ1 = findAct(m, "A_WRJ1"); A.WRJ0 = findAct(m, "A_WRJ0");
    A.FFJ3 = findAct(m, "A_FFJ3"); A.FFJ2 = findAct(m, "A_FFJ2");
    A.FFJ1 = findAct(m, "A_FFJ1"); A.FFJ0 = findAct(m, "A_FFJ0");
    A.MFJ3 = findAct(m, "A_MFJ3"); A.MFJ2 = findAct(m, "A_MFJ2");
    A.MFJ1 = findAct(m, "A_MFJ1"); A.MFJ0 = findAct(m, "A_MFJ0");
    A.RFJ3 = findAct(m, "A_RFJ3"); A.RFJ2 = findAct(m, "A_RFJ2");
    A.RFJ1 = findAct(m, "A_RFJ1"); A.RFJ0 = findAct(m, "A_RFJ0");
    A.LFJ4 = findAct(m, "A_LFJ4"); A.LFJ3 = findAct(m, "A_LFJ3");
    A.LFJ2 = findAct(m, "A_LFJ2"); A.LFJ1 = findAct(m, "A_LFJ1");
    A.LFJ0 = findAct(m, "A_LFJ0");
    A.THJ4 = findAct(m, "A_THJ4"); A.THJ3 = findAct(m, "A_THJ3");
    A.THJ2 = findAct(m, "A_THJ2"); A.THJ1 = findAct(m, "A_THJ1");
    A.THJ0 = findAct(m, "A_THJ0");

    buildPoseSequence();
    memset(prevCtrl, 0, sizeof(prevCtrl));
    poseIdx = 0;
    elapsed = 0.0;

    printf("[Controller] Initialized: %d actuators, %d poses\n", nu, numPoses);
}

void controller_step(mjModel* m, mjData* d)
{
    if (numPoses == 0) return;

    elapsed += m->opt.timestep;

    Pose& cur = poses[poseIdx];
    double totalTime = BLEND_TIME + cur.holdTime;

    // 当前姿态播放完毕，切换到下一个
    if (elapsed >= totalTime) {
        memcpy(prevCtrl, cur.ctrl, sizeof(double) * nu);
        poseIdx = (poseIdx + 1) % numPoses;
        elapsed = 0.0;
        printf("[Controller] -> %s\n", poses[poseIdx].name);
    }

    // 计算过渡系数：0（还在上一个姿态）→ 1（完全到达当前姿态）
    double alpha = cosInterp(elapsed / BLEND_TIME);
    Pose& target = poses[poseIdx];

    // 对每个执行器做插值，写入控制量
    for (int i = 0; i < nu; i++) {
        d->ctrl[i] = prevCtrl[i] + alpha * (target.ctrl[i] - prevCtrl[i]);
    }
}