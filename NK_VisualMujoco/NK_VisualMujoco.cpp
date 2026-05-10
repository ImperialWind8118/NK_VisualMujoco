#if 0


#include "controller.h"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstring>
#include <cmath>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

// ===================== MuJoCo 全局对象 =====================
mjModel* m = nullptr;
mjData* d = nullptr;
mjvScene   scn;
mjvCamera  cam;
mjvOption  opt;
mjrContext con;
mjvPerturb pert;

// ===================== 时间缩放 =====================
static float  timeScale = 1.0f;
static double baseTimestep = 0.002;

// ===================== 相机鼠标状态 =====================
static bool   button_left = false;
static bool   button_right = false;
static double lastx = 0, lasty = 0;

// =========================================================
// 路径4(1)：关节状态监测
// =========================================================
static constexpr int N_MON_JOINTS = 5;
static const char* MON_JOINT_NAMES[N_MON_JOINTS] = { "FFJ2","MFJ2","RFJ2","LFJ2","THJ3" };
static const char* MON_JOINT_LABELS[N_MON_JOINTS] = {
    "食指 FFJ2", "中指 MFJ2", "无名 RFJ2", "小指 LFJ2", "拇指 THJ3"
};
static int    monJointId[N_MON_JOINTS];   // joint id（mj_name2id结果）
static int    monQposAdr[N_MON_JOINTS];   // 在 qpos 数组里的下标
static double monJntMin[N_MON_JOINTS];    // 关节下限
static double monJntMax[N_MON_JOINTS];    // 关节上限

// =========================================================
// 路径4(2)：指尖轨迹拖尾
// =========================================================
static constexpr int N_TIPS = 5;
static constexpr int TRAIL_LEN = 100;

static const char* TIP_SITE_NAMES[N_TIPS] = {
    "S_fftip", "S_mftip", "S_rftip", "S_lftip", "S_thtip"
};
// 每根手指的轨迹颜色 RGBA
static const float TIP_COLORS[N_TIPS][4] = {
    {1.0f, 0.35f, 0.35f, 1.0f},   // 食指 红
    {0.35f, 1.0f, 0.35f, 1.0f},   // 中指 绿
    {0.4f,  0.6f, 1.0f,  1.0f},   // 无名 蓝
    {1.0f,  1.0f, 0.35f, 1.0f},   // 小指 黄
    {0.35f, 1.0f, 1.0f,  1.0f},   // 拇指 青
};

static int   tipSiteId[N_TIPS];                     // site id
static float trailPos[N_TIPS][TRAIL_LEN][3];         // 环形缓冲
static int   trailHead = 0;                         // 下一个写入位置
static int   trailCount = 0;                         // 已有帧数（上限 TRAIL_LEN）

// =========================================================
// 工具：向场景注入一条线段
// =========================================================
static void injectLine(mjvScene& s,
    float ax, float ay, float az,
    float bx, float by, float bz,
    const float col[4], float alpha)
{
    if (s.ngeom >= s.maxgeom) return;
    mjvGeom& g = s.geoms[s.ngeom++];
    mjv_initGeom(&g, mjGEOM_LINE, nullptr, nullptr, nullptr, nullptr);

    mjtNum from[3] = { ax, ay, az };
    mjtNum to[3] = { bx, by, bz };
    mjv_connector(&g, mjGEOM_LINE, 2.0, from, to);

    g.rgba[0] = col[0];
    g.rgba[1] = col[1];
    g.rgba[2] = col[2];
    g.rgba[3] = col[3] * alpha;
}

// =========================================================
// 鼠标按键回调
// =========================================================
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    ImGui_ImplGlfw_MouseButtonCallback(window, button, act, mods);
    if (ImGui::GetIO().WantCaptureMouse) return;

    bool ctrl = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);

    glfwGetCursorPos(window, &lastx, &lasty);

    if (ctrl) {
        if (act == GLFW_PRESS) {
            pert.active = 0; pert.select = 0;

            int width, height;
            glfwGetWindowSize(window, &width, &height);

            // 屏幕坐标归一化到 [-0.5, 0.5]，Y 轴翻转
            double relx = lastx / width - 0.5;
            double rely = 0.5 - lasty / height;

            // 从 scn.camera 取相机位置和朝向
            mjvGLCamera& gc = scn.camera[0];
            mjtNum camPos[3] = { gc.pos[0],     gc.pos[1],     gc.pos[2] };
            mjtNum fwd[3] = { gc.forward[0], gc.forward[1], gc.forward[2] };
            mjtNum upv[3] = { gc.up[0],      gc.up[1],      gc.up[2] };
            mjtNum rgt[3];
            mju_cross(rgt, fwd, upv);   // right = forward × up

            // 用 cam.fovy（垂直视角，角度）算射线方向，不依赖视锥体参数
            double tanHalf = (double)gc.frustum_top / (double)gc.frustum_near;
            double aspect = (double)width / height;
            double sx = relx * 2.0 * aspect * tanHalf;
            double sy = rely * 2.0 * tanHalf;

            mjtNum ray[3] = {
                fwd[0] + rgt[0] * sx + upv[0] * sy,
                fwd[1] + rgt[1] * sx + upv[1] * sy,
                fwd[2] + rgt[2] * sx + upv[2] * sy
            };
            mju_normalize3(ray);

            // 投射射线
            mjtByte geomgroup[6]; memset(geomgroup, 1, sizeof(geomgroup));
            int hitGeom[1] = { -1 };
            mjtNum dist = mj_ray(m, d, camPos, ray, geomgroup, 1, -1, hitGeom);

            if (dist >= 0 && hitGeom[0] >= 0) {
                int selBody = m->geom_bodyid[hitGeom[0]];
                if (selBody > 0) {   // 排除 world body（地板）
                    pert.select = selBody;

                    mjtNum selpnt[3] = {
                        camPos[0] + dist * ray[0],
                        camPos[1] + dist * ray[1],
                        camPos[2] + dist * ray[2]
                    };
                    mjtNum worldOff[3];
                    mju_sub3(worldOff, selpnt, d->xpos + 3 * selBody);
                    mjtNum tmp[3];
                    mju_mulMatTVec(tmp, d->xmat + 9 * selBody, worldOff, 3, 3);
                    mju_copy3(pert.localpos, tmp);

                    mjv_initPerturb(m, d, &scn, &pert);
                    pert.active = (button == GLFW_MOUSE_BUTTON_LEFT)
                        ? mjPERT_TRANSLATE : mjPERT_ROTATE;
                }
            }
        }
        else if (act == GLFW_RELEASE) {
            pert.active = 0; pert.select = 0;
        }
        return;
    }

    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    if (act == GLFW_RELEASE) { pert.active = 0; pert.select = 0; }
}

// =========================================================
// 鼠标移动回调
// =========================================================
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    ImGui_ImplGlfw_CursorPosCallback(window, xpos, ypos);
    if (ImGui::GetIO().WantCaptureMouse) return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos; lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool ctrl = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);

    if (ctrl) {
        if (pert.select > 0 && pert.active > 0) {
            mjtMouse action = (pert.active == mjPERT_TRANSLATE)
                ? mjMOUSE_MOVE_H : mjMOUSE_ROTATE_H;
            mjv_movePerturb(m, d, action, dx / height * 3.0, dy / height * 3.0, &scn, &pert);
        }
        return;
    }

    if (!button_left && !button_right) return;
    mjtMouse action = button_right ? mjMOUSE_MOVE_V : mjMOUSE_ROTATE_V;
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// =========================================================
// 滚轮回调
// =========================================================
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
    if (ImGui::GetIO().WantCaptureMouse) return;
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// =========================================================
// main
// =========================================================
int main(void)
{
    // ---- 加载模型 ----
    char error[1000];
    m = mj_loadXML("model_formal/Adroit/Adroit_hand.xml", nullptr, error, 1000);
    if (!m) { printf("模型加载失败: %s\n", error); return 1; }
    d = mj_makeData(m);
    baseTimestep = m->opt.timestep;

    controller_init(m, d);

    // ---- 路径4(1)：查找监测关节 ----
    for (int i = 0; i < N_MON_JOINTS; i++) {
        int jid = mj_name2id(m, mjOBJ_JOINT, MON_JOINT_NAMES[i]);
        monJointId[i] = jid;
        monQposAdr[i] = (jid >= 0) ? m->jnt_qposadr[jid] : -1;
        monJntMin[i] = (jid >= 0) ? m->jnt_range[2 * jid] : 0.0;
        monJntMax[i] = (jid >= 0) ? m->jnt_range[2 * jid + 1] : 1.0;
        if (jid < 0) printf("[P4] Warning: joint '%s' not found\n", MON_JOINT_NAMES[i]);
    }

    // ---- 路径4(2)：查找指尖 site ----
    for (int i = 0; i < N_TIPS; i++) {
        tipSiteId[i] = mj_name2id(m, mjOBJ_SITE, TIP_SITE_NAMES[i]);
        if (tipSiteId[i] < 0)
            printf("[P4] Warning: site '%s' not found\n", TIP_SITE_NAMES[i]);
    }
    memset(trailPos, 0, sizeof(trailPos));

    // ---- 初始化GLFW ----
    if (!glfwInit()) return 1;
    GLFWwindow* window = glfwCreateWindow(1280, 960, "MuJoCo 灵巧手仿真", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);

    // ---- 初始化MuJoCo渲染器 ----
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);
    mjv_makeScene(m, &scn, 4000);   // 留足空间给轨迹线段
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // ---- 初始化ImGui ----
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/msyh.ttc", 16.0f, nullptr,
        io.Fonts->GetGlyphRangesChineseFull());
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, false);
    ImGui_ImplOpenGL3_Init("#version 130");

    // ===================== 主循环 =====================
    while (!glfwWindowShouldClose(window))
    {
        // ---- 时间缩放 ----
        m->opt.timestep = baseTimestep * (double)timeScale;

        // ---- 物理步进 ----
        for (int i = 0; i < 1; i++) {
            mju_zero(d->xfrc_applied, 6 * m->nbody);
            mjv_applyPerturbForce(m, d, &pert);
            controller_step(m, d);
            mj_step(m, d);
        }

        // ---- 路径4(2)：把当前指尖坐标写入环形缓冲 ----
        for (int f = 0; f < N_TIPS; f++) {
            int sid = tipSiteId[f];
            if (sid < 0) continue;
            const mjtNum* p = d->site_xpos + 3 * sid;
            trailPos[f][trailHead][0] = (float)p[0];
            trailPos[f][trailHead][1] = (float)p[1];
            trailPos[f][trailHead][2] = (float)p[2];
        }
        trailHead = (trailHead + 1) % TRAIL_LEN;
        if (trailCount < TRAIL_LEN) trailCount++;

        // ---- MuJoCo场景更新 ----
        int W, H;
        glfwGetFramebufferSize(window, &W, &H);
        mjrRect viewport = { 0, 0, W, H };

        mjv_updateScene(m, d, &opt, &pert, &cam, mjCAT_ALL, &scn);

        // ---- 路径4(2)：注入指尖轨迹线段（updateScene之后，render之前）----
        if (trailCount >= 2) {
            for (int f = 0; f < N_TIPS; f++) {
                if (tipSiteId[f] < 0) continue;

                // 从最旧帧到最新帧依次画线段
                // 环形缓冲：最旧帧在 trailHead（当 count==TRAIL_LEN 时）
                int total = trailCount;
                for (int seg = 0; seg < total - 1; seg++) {
                    // seg=0 是最旧的线段，seg=total-2 是最新的
                    int idxA = (trailHead - total + seg + TRAIL_LEN * 2) % TRAIL_LEN;
                    int idxB = (trailHead - total + seg + 1 + TRAIL_LEN * 2) % TRAIL_LEN;

                    // alpha：越新越不透明
                    float alpha = (float)(seg + 1) / (float)(total - 1);
                    alpha = alpha * alpha;   // 平方让衰减更陡，旧端消散更快

                    injectLine(scn,
                        trailPos[f][idxA][0], trailPos[f][idxA][1], trailPos[f][idxA][2],
                        trailPos[f][idxB][0], trailPos[f][idxB][1], trailPos[f][idxB][2],
                        TIP_COLORS[f], alpha);
                }
            }
        }

        mjr_render(viewport, &scn, &con);

        // ---- ImGui ----
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 窗口1：仿真控制（路径3，保持不变）
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(320, 90), ImGuiCond_Always);
        ImGui::Begin("仿真控制", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
        ImGui::TextDisabled("Ctrl + 左键拖拽：移动指节");
        ImGui::SliderFloat("Time Scale", &timeScale, 0.0f, 3.0f, "%.2fx");
        ImGui::End();

        // 窗口2：路径4(1) 关节状态监测面板
        ImGui::SetNextWindowPos(ImVec2(10, 110), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(320, 210), ImGuiCond_Always);
        ImGui::Begin("关节状态监测", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

        for (int i = 0; i < N_MON_JOINTS; i++) {
            int adr = monQposAdr[i];
            if (adr < 0) { ImGui::Text("%s: N/A", MON_JOINT_LABELS[i]); continue; }

            double qval = d->qpos[adr];
            double qmin = monJntMin[i];
            double qmax = monJntMax[i];
            double range = qmax - qmin;
            float  frac = (range > 1e-6) ? (float)((qval - qmin) / range) : 0.0f;
            frac = fmaxf(0.0f, fminf(1.0f, frac));

            // 距限位不足10%时变红，否则绿
            bool nearLimit = (frac < 0.10f || frac > 0.90f);
            ImVec4 barColor = nearLimit
                ? ImVec4(1.0f, 0.25f, 0.25f, 1.0f)   // 红色警告
                : ImVec4(0.25f, 0.85f, 0.40f, 1.0f);  // 正常绿

            ImGui::Text("%s", MON_JOINT_LABELS[i]);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, barColor);
            char barId[32]; snprintf(barId, sizeof(barId), "##jbar%d", i);
            ImGui::ProgressBar(frac, ImVec2(-1, 14), barId);
            ImGui::PopStyleColor();

            // 在进度条右边显示实际角度值（弧度）
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetItemRectSize().x);
            ImGui::TextDisabled("  %.2f rad", qval);
        }

        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // ---- 清理 ----
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}


#endif