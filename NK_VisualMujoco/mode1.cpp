#include "mode1.h"
#include "controller.h"
#include <cstring>
#include <cmath>
#include <cstdio>

// ===== 路径4(1): 关节监测 =====
static constexpr int N_MON = 5;
static const char* MON_NAMES[N_MON] = { "FFJ2","MFJ2","RFJ2","LFJ2","THJ3" };
static const char* MON_LABELS[N_MON] = { "食指 FFJ2","中指 MFJ2","无名 RFJ2","小指 LFJ2","拇指 THJ3" };
static int    monJointId[N_MON];
static int    monQposAdr[N_MON];
static double monJntMin[N_MON];
static double monJntMax[N_MON];

// ===== 路径4(2): 指尖轨迹 =====
static constexpr int N_TIPS = 5;
static constexpr int TRAIL_LEN = 100;
static const char* TIP_NAMES[N_TIPS] = { "S_fftip","S_mftip","S_rftip","S_lftip","S_thtip" };
static const float TIP_COLORS[N_TIPS][4] = {
    {1.0f, 0.35f, 0.35f, 1.0f},
    {0.35f, 1.0f, 0.35f, 1.0f},
    {0.4f,  0.6f, 1.0f,  1.0f},
    {1.0f,  1.0f, 0.35f, 1.0f},
    {0.35f, 1.0f, 1.0f,  1.0f},
};
static int   tipSiteId[N_TIPS];
static float trailPos[N_TIPS][TRAIL_LEN][3];
static int   trailHead = 0, trailCount = 0;

// ===== 工具函数 =====
static void injectLine(mjvScene& s,
    float ax, float ay, float az,
    float bx, float by, float bz,
    const float col[4], float alpha)
{
    if (s.ngeom >= s.maxgeom) return;
    mjvGeom& g = s.geoms[s.ngeom++];
    mjv_initGeom(&g, mjGEOM_LINE, nullptr, nullptr, nullptr, nullptr);
    mjtNum from[3] = { ax, ay, az }, to[3] = { bx, by, bz };
    mjv_connector(&g, mjGEOM_LINE, 2.0, from, to);
    g.rgba[0] = col[0]; g.rgba[1] = col[1];
    g.rgba[2] = col[2]; g.rgba[3] = col[3] * alpha;
}

// ===== 对外接口 =====
void mode1_init(mjModel* m, mjData* d)
{
    mj_resetData(m, d);

    for (int i = 0; i < N_MON; i++) {
        int jid = mj_name2id(m, mjOBJ_JOINT, MON_NAMES[i]);
        monJointId[i] = jid;
        monQposAdr[i] = (jid >= 0) ? m->jnt_qposadr[jid] : -1;
        monJntMin[i] = (jid >= 0) ? m->jnt_range[2 * jid] : 0.0;
        monJntMax[i] = (jid >= 0) ? m->jnt_range[2 * jid + 1] : 1.0;
    }
    for (int i = 0; i < N_TIPS; i++)
        tipSiteId[i] = mj_name2id(m, mjOBJ_SITE, TIP_NAMES[i]);

    memset(trailPos, 0, sizeof(trailPos));
    trailHead = 0; trailCount = 0;
    mjv_defaultPerturb(&pert);
    controller_init(m, d);
}

void mode1_step(mjModel* m, mjData* d)
{
    mju_zero(d->xfrc_applied, 6 * m->nbody);
    mjv_applyPerturbForce(m, d, &pert);
    controller_step(m, d);
    mj_step(m, d);

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
}

void mode1_inject_geoms(mjvScene& scn)
{
    if (trailCount < 2) return;
    for (int f = 0; f < N_TIPS; f++) {
        if (tipSiteId[f] < 0) continue;
        int total = trailCount;
        for (int seg = 0; seg < total - 1; seg++) {
            int idxA = (trailHead - total + seg + TRAIL_LEN * 2) % TRAIL_LEN;
            int idxB = (trailHead - total + seg + 1 + TRAIL_LEN * 2) % TRAIL_LEN;
            float alpha = (float)(seg + 1) / (float)(total - 1);
            alpha = alpha * alpha;
            injectLine(scn,
                trailPos[f][idxA][0], trailPos[f][idxA][1], trailPos[f][idxA][2],
                trailPos[f][idxB][0], trailPos[f][idxB][1], trailPos[f][idxB][2],
                TIP_COLORS[f], alpha);
        }
    }
}

void mode1_render_ui()
{
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(320, 115), ImGuiCond_Always);
    ImGui::Begin("仿真控制", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
    ImGui::TextDisabled("Ctrl + 左键拖拽：移动指节");
    ImGui::SliderFloat("Time Scale", &timeScale, 0.0f, 3.0f, "%.2fx");
    if (ImGui::Button("返回主菜单")) { mode1_cleanup(); currentMode = AppMode::MENU; }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(10, 135), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(320, 210), ImGuiCond_Always);
    ImGui::Begin("关节状态监测", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
    for (int i = 0; i < N_MON; i++) {
        int adr = monQposAdr[i];
        if (adr < 0) { ImGui::Text("%s: N/A", MON_LABELS[i]); continue; }
        double qval = d->qpos[adr];
        double range = monJntMax[i] - monJntMin[i];
        float  frac = (range > 1e-6) ? (float)((qval - monJntMin[i]) / range) : 0.0f;
        frac = fmaxf(0.0f, fminf(1.0f, frac));
        bool nearLimit = (frac < 0.10f || frac > 0.90f);
        ImVec4 barColor = nearLimit
            ? ImVec4(1.0f, 0.25f, 0.25f, 1.0f)
            : ImVec4(0.25f, 0.85f, 0.40f, 1.0f);
        ImGui::Text("%s", MON_LABELS[i]);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, barColor);
        char barId[32]; snprintf(barId, sizeof(barId), "##jbar%d", i);
        ImGui::ProgressBar(frac, ImVec2(-1, 14), barId);
        ImGui::PopStyleColor();
    }
    ImGui::End();
}

void mode1_cleanup()
{
    pert.active = 0; pert.select = 0;
    trailHead = 0;   trailCount = 0;
}

void mode1_ctrl_click(GLFWwindow* window, int button, int act, double cx, double cy)
{
    if (act == GLFW_PRESS) {
        pert.active = 0; pert.select = 0;
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        double relx = cx / width - 0.5;
        double rely = 0.5 - cy / height;

        mjvGLCamera& gc = scn.camera[0];
        mjtNum camPos[3] = { gc.pos[0], gc.pos[1], gc.pos[2] };
        mjtNum fwd[3] = { gc.forward[0], gc.forward[1], gc.forward[2] };
        mjtNum upv[3] = { gc.up[0], gc.up[1], gc.up[2] };
        mjtNum rgt[3];
        mju_cross(rgt, fwd, upv);

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

        mjtByte geomgroup[6]; memset(geomgroup, 1, sizeof(geomgroup));
        int hitGeom[1] = { -1 };
        mjtNum dist = mj_ray(m, d, camPos, ray, geomgroup, 1, -1, hitGeom);

        if (dist >= 0 && hitGeom[0] >= 0) {
            int selBody = m->geom_bodyid[hitGeom[0]];
            if (selBody > 0) {
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
}

void mode1_ctrl_drag(double dx, double dy, int height)
{
    if (pert.select > 0 && pert.active > 0) {
        mjtMouse action = (pert.active == mjPERT_TRANSLATE)
            ? mjMOUSE_MOVE_H : mjMOUSE_ROTATE_H;
        mjv_movePerturb(m, d, action, dx / height * 3.0, dy / height * 3.0, &scn, &pert);
    }
}

void mode1_release_perturb()
{
    pert.active = 0; pert.select = 0;
}