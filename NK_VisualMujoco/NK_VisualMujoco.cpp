#include "controller.h"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstring>

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
mjvPerturb pert;                    // 鼠标扰动对象

// ===================== 时间缩放 =====================
static float  timeScale = 1.0f; // ImGui滑条读取值
static double baseTimestep = 0.002;// 原始timestep，程序启动时保存

// ===================== 相机鼠标状态 =====================
static bool   button_left = false;
static bool   button_right = false;
static double lastx = 0, lasty = 0;

// ===================== 鼠标按键回调 =====================
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    ImGui_ImplGlfw_MouseButtonCallback(window, button, act, mods);
    if (ImGui::GetIO().WantCaptureMouse) return;

    bool ctrl = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);

    glfwGetCursorPos(window, &lastx, &lasty);

    if (ctrl) {
        // Ctrl模式：只做扰动，不碰相机状态
        if (act == GLFW_PRESS) {
            pert.active = 0;
            pert.select = 0;

			int width, height;
            glfwGetWindowSize(window, &width, &height);

            // 从场景相机获取位置和方向
            mjvGLCamera& glcam = scn.camera[0];
            mjtNum cam_pos[3] = { glcam.pos[0], glcam.pos[1], glcam.pos[2] };

            // 计算相机右方向 = forward × up
            float right[3] = {
                glcam.forward[1] * glcam.up[2] - glcam.forward[2] * glcam.up[1],
                glcam.forward[2] * glcam.up[0] - glcam.forward[0] * glcam.up[2],
                glcam.forward[0] * glcam.up[1] - glcam.forward[1] * glcam.up[0]
            };

            // 光标归一化坐标
            double relx = lastx / width - 0.5;
            double rely = 0.5 - lasty / height;

            // 利用视锥体参数计算射线方向
            float fn = glcam.frustum_near;
            float fy = (glcam.frustum_top - glcam.frustum_bottom) * 0.5f;
            float fx = fy * (float)width / (float)height;
            float xw = glcam.frustum_center + (float)relx * 2.0f * fx;
            float yw = (glcam.frustum_top + glcam.frustum_bottom) * 0.5f
                + (float)rely * 2.0f * fy;

            mjtNum rayDir[3] = {
                glcam.forward[0] * fn + right[0] * xw + glcam.up[0] * yw,
                glcam.forward[1] * fn + right[1] * xw + glcam.up[1] * yw,
                glcam.forward[2] * fn + right[2] * xw + glcam.up[2] * yw
            };
            mju_normalize3(rayDir);

            // 射线检测，bodyexclude=0 跳过世界body（地面）
            mjtByte geomgroup[6];
            memset(geomgroup, 1, sizeof(geomgroup));
            int hitGeom[1] = { -1 };
            mjtNum selpnt[3];
            mjtNum dist = mj_ray(m, d, cam_pos, rayDir, geomgroup, 1, 0, hitGeom);

            printf("[Ray] dist=%.3f hitGeom=%d\n", (float)dist, hitGeom[0]);

            if (dist >= 0 && hitGeom[0] >= 0) {
                pert.select = m->geom_bodyid[hitGeom[0]];
                selpnt[0] = cam_pos[0] + dist * rayDir[0];
                selpnt[1] = cam_pos[1] + dist * rayDir[1];
                selpnt[2] = cam_pos[2] + dist * rayDir[2];
                printf("[Ray] hit body=%d\n", pert.select);
            }

            if (pert.select > 0) {
                mju_sub3(pert.localpos, selpnt, d->xpos + 3 * pert.select);
                mjtNum tmp[3];
                mju_mulMatTVec(tmp, d->xmat + 9 * pert.select, pert.localpos, 3, 3);
                mju_copy3(pert.localpos, tmp);
                mjv_initPerturb(m, d, &scn, &pert);
                pert.active = (button == GLFW_MOUSE_BUTTON_LEFT)
                    ? mjPERT_TRANSLATE : mjPERT_ROTATE;
            }
        }
        else if (act == GLFW_RELEASE) {
            pert.active = 0;
            pert.select = 0;
        }
        return; // 关键：Ctrl模式下直接返回，不更新button_left/right
    }

    // 非Ctrl模式：相机控制
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    if (act == GLFW_RELEASE) {
        pert.active = 0;
        pert.select = 0;
    }
}

// ===================== 鼠标移动回调 =====================
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    ImGui_ImplGlfw_CursorPosCallback(window, xpos, ypos);
    if (ImGui::GetIO().WantCaptureMouse) return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool ctrl = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);

    if (ctrl) {
        // Ctrl模式：只处理扰动，绝不移相机
        if (pert.select > 0 && pert.active > 0) {
            mjtMouse action = (pert.active == mjPERT_TRANSLATE)
                ? mjMOUSE_MOVE_H : mjMOUSE_ROTATE_H;
            mjv_movePerturb(m, d, action, dx / height, dy / height, &scn, &pert);
        }
        return;
    }

    // 非Ctrl模式：相机控制
    if (!button_left && !button_right) return;
    mjtMouse action = button_right ? mjMOUSE_MOVE_V : mjMOUSE_ROTATE_V;
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// ===================== 滚轮回调 =====================
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
    if (ImGui::GetIO().WantCaptureMouse) return;
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// ===================== main =====================
int main(void)
{
    // ---- 加载模型 ----
    char error[1000];
    m = mj_loadXML("model_formal/Adroit/Adroit_hand.xml", nullptr, error, 1000);
    if (!m) { printf("模型加载失败: %s\n", error); return 1; }
    d = mj_makeData(m);
    baseTimestep = m->opt.timestep;  // 保存原始timestep

    controller_init(m, d);

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
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // ---- 初始化ImGui ----
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/msyh.ttc", 16.0f, nullptr,
        io.Fonts->GetGlyphRangesChineseFull());
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, false); // false：我们自己转发回调
    ImGui_ImplOpenGL3_Init("#version 130");

    // ===================== 主循环 =====================
    while (!glfwWindowShouldClose(window))
    {
        // ---- 应用时间缩放 ----
        m->opt.timestep = baseTimestep * (double)timeScale;

        // ---- 物理步进（8步/帧）----
        for (int i = 0; i < 8; i++) {
            // 每步前清零外力，再施加鼠标扰动力
            mju_zero(d->xfrc_applied, 6 * m->nbody);
            mjv_applyPerturbForce(m, d, &pert);

            //controller_step(m, d);
            mj_step(m, d);
        }

        // ---- MuJoCo渲染 ----
        int W, H;
        glfwGetFramebufferSize(window, &W, &H);
        mjrRect viewport = { 0, 0, W, H };

        mjv_updateScene(m, d, &opt, &pert, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // ---- ImGui渲染 ----
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(320, 90), ImGuiCond_Always);
        ImGui::Begin("仿真控制", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse);

        ImGui::TextDisabled("Ctrl + 左键拖拽：移动指节");
        ImGui::SliderFloat("Time Scale", &timeScale, 0.05f, 2.0f, "%.2fx");

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