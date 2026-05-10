#include "app_common.h"
#include "mode1.h"
#include "mode2.h"
#include <cstdio>
#include <cstring>

// MuJoCo全局变量定义
mjModel* m = nullptr;
mjData* d = nullptr;
mjvScene   scn;
mjvCamera  cam;
mjvOption  opt;
mjrContext con;
mjvPerturb pert;

AppMode currentMode = AppMode::MENU;
float   timeScale = 1.0f;
double  baseTimestep = 0.002;

// 相机状态
static bool   button_left = false;
static bool   button_right = false;
static double lastx = 0, lasty = 0;

// GLFW 处理鼠标键盘输入
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    ImGui_ImplGlfw_MouseButtonCallback(window, button, act, mods);
    if (ImGui::GetIO().WantCaptureMouse) return;

    glfwGetCursorPos(window, &lastx, &lasty);

    bool ctrl = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);

    if (currentMode == AppMode::MODE1 && ctrl) {
        mode1_ctrl_click(window, button, act, lastx, lasty);
        return;
    }

    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    if (act == GLFW_RELEASE && currentMode == AppMode::MODE1)
        mode1_release_perturb();
}

void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    ImGui_ImplGlfw_CursorPosCallback(window, xpos, ypos);
    if (ImGui::GetIO().WantCaptureMouse) return;

    double dx = xpos - lastx, dy = ypos - lasty;
    lastx = xpos; lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool ctrl = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);

    if (currentMode == AppMode::MODE1 && ctrl) {
        mode1_ctrl_drag(dx, dy, height);
        return;
    }

    if (!button_left && !button_right) return;
    mjtMouse action = button_right ? mjMOUSE_MOVE_V : mjMOUSE_ROTATE_V;
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
    if (ImGui::GetIO().WantCaptureMouse) return;
	mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);     // 这里其实应该是-0.05，但反过来更符合用户习惯
}

// 优化imgui默认ui主题
static void apply_dark_theme()
{
    ImGuiStyle& s = ImGui::GetStyle();

    // 圆角与边框
    s.WindowRounding = 6.0f;
    s.FrameRounding = 4.0f;
    s.GrabRounding = 4.0f;
    s.ScrollbarRounding = 4.0f;
    s.WindowBorderSize = 1.0f;
    s.FrameBorderSize = 0.5f;

    // 间距
    s.WindowPadding = ImVec2(10, 8);
    s.FramePadding = ImVec2(6, 3);
    s.ItemSpacing = ImVec2(6, 4);

    // 颜色
    ImVec4* c = s.Colors;

    // 背景
    c[ImGuiCol_WindowBg] = ImVec4(0.06f, 0.08f, 0.12f, 0.92f);
    c[ImGuiCol_ChildBg] = ImVec4(0.04f, 0.06f, 0.10f, 0.80f);
    c[ImGuiCol_PopupBg] = ImVec4(0.08f, 0.10f, 0.16f, 0.95f);

    // 文字
    c[ImGuiCol_Text] = ImVec4(0.85f, 0.92f, 1.00f, 1.00f);
    c[ImGuiCol_TextDisabled] = ImVec4(0.35f, 0.48f, 0.60f, 1.00f);

    // 边框
    c[ImGuiCol_Border] = ImVec4(0.12f, 0.32f, 0.52f, 0.70f);
    c[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

    // 标题栏
    c[ImGuiCol_TitleBg] = ImVec4(0.04f, 0.08f, 0.14f, 1.00f);
    c[ImGuiCol_TitleBgActive] = ImVec4(0.05f, 0.16f, 0.30f, 1.00f);
    c[ImGuiCol_TitleBgCollapsed] = ImVec4(0.02f, 0.05f, 0.10f, 0.80f);

    // Frame（输入框/进度条底色）
    c[ImGuiCol_FrameBg] = ImVec4(0.10f, 0.14f, 0.20f, 0.80f);
    c[ImGuiCol_FrameBgHovered] = ImVec4(0.14f, 0.22f, 0.32f, 0.80f);
    c[ImGuiCol_FrameBgActive] = ImVec4(0.16f, 0.26f, 0.40f, 0.80f);

    // 滑块（赛博蓝高亮）
    c[ImGuiCol_SliderGrab] = ImVec4(0.00f, 0.72f, 1.00f, 1.00f);
    c[ImGuiCol_SliderGrabActive] = ImVec4(0.20f, 0.88f, 1.00f, 1.00f);
    c[ImGuiCol_CheckMark] = ImVec4(0.00f, 0.85f, 1.00f, 1.00f);

    // 按钮
    c[ImGuiCol_Button] = ImVec4(0.05f, 0.22f, 0.42f, 0.85f);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.00f, 0.52f, 0.85f, 0.90f);
    c[ImGuiCol_ButtonActive] = ImVec4(0.00f, 0.68f, 1.00f, 1.00f);

    // Header（Columns表头/可折叠项）
    c[ImGuiCol_Header] = ImVec4(0.00f, 0.45f, 0.75f, 0.50f);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.00f, 0.60f, 0.90f, 0.70f);
    c[ImGuiCol_HeaderActive] = ImVec4(0.00f, 0.72f, 1.00f, 0.80f);

    // 分隔线
    c[ImGuiCol_Separator] = ImVec4(0.12f, 0.32f, 0.52f, 0.80f);
    c[ImGuiCol_SeparatorHovered] = ImVec4(0.20f, 0.55f, 0.80f, 0.90f);
    c[ImGuiCol_SeparatorActive] = ImVec4(0.25f, 0.70f, 1.00f, 1.00f);

    // 滚动条
    c[ImGuiCol_ScrollbarBg] = ImVec4(0.04f, 0.06f, 0.10f, 0.80f);
    c[ImGuiCol_ScrollbarGrab] = ImVec4(0.14f, 0.32f, 0.52f, 0.80f);
    c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.18f, 0.48f, 0.72f, 0.90f);
    c[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.22f, 0.62f, 0.90f, 1.00f);

    // 进度条填充色（mode1 关节监测用到）
    c[ImGuiCol_PlotHistogram] = ImVec4(0.00f, 0.72f, 1.00f, 0.90f);
    c[ImGuiCol_PlotHistogramHovered] = ImVec4(0.20f, 0.88f, 1.00f, 1.00f);

    // 曲线（PlotLines，mode2 收敛曲线用到）
    c[ImGuiCol_PlotLines] = ImVec4(0.00f, 0.72f, 1.00f, 1.00f);
    c[ImGuiCol_PlotLinesHovered] = ImVec4(0.20f, 0.88f, 1.00f, 1.00f);

    // 表格（mode2 参数对比用到）
    c[ImGuiCol_TableHeaderBg] = ImVec4(0.06f, 0.12f, 0.22f, 1.00f);
    c[ImGuiCol_TableBorderStrong] = ImVec4(0.14f, 0.34f, 0.54f, 1.00f);
    c[ImGuiCol_TableBorderLight] = ImVec4(0.08f, 0.20f, 0.36f, 1.00f);
    c[ImGuiCol_TableRowBgAlt] = ImVec4(0.06f, 0.10f, 0.16f, 0.50f);

    // 拖拽
    c[ImGuiCol_DragDropTarget] = ImVec4(0.00f, 0.85f, 1.00f, 0.90f);

    // 导航高亮
    c[ImGuiCol_NavHighlight] = ImVec4(0.00f, 0.72f, 1.00f, 1.00f);
}

// 起始菜单
static void render_menu()
{
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(
        ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f),
        ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(420, 240), ImGuiCond_Always);
    ImGui::Begin("##menu", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

    float textW = ImGui::CalcTextSize("灵巧手仿真平台").x;
    ImGui::SetCursorPosX((420.0f - textW) * 0.5f);
    ImGui::Text("灵巧手仿真平台");
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::TextWrapped("模式一：交互可视化");
    ImGui::TextDisabled("  鼠标拖拽 · 时间缩放 · 关节监测 · 轨迹留影");
    if (ImGui::Button("进入模式一", ImVec2(-1, 42))) {
        mode1_init(m, d);
        currentMode = AppMode::MODE1;
    }
    ImGui::Spacing();
    ImGui::TextWrapped("模式二：参数辨识");
    ImGui::TextDisabled("  单指简谐运动 · 实时最小二乘 · 误差收敛曲线");
    if (ImGui::Button("进入模式二", ImVec2(-1, 42))) {
        mode2_init(m, d);
        currentMode = AppMode::MODE2;
    }
    ImGui::End();
}

// ===== MAIN =====
int main(void)
{
    // 加载模型
    char error[1000];
    m = mj_loadXML("model_formal/Adroit/Adroit_hand.xml", nullptr, error, 1000);
    if (!m) { printf("模型加载失败: %s\n", error); return 1; }
    d = mj_makeData(m);
    baseTimestep = m->opt.timestep;


	// 初始化GLFW和窗口
    if (!glfwInit()) return 1;
    GLFWwindow* window = glfwCreateWindow(1280, 960, "灵巧手仿真平台", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);


	// 初始化MuJoCo渲染器
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);
    mjv_makeScene(m, &scn, 4000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    // 世界质感增强
    m->vis.headlight.ambient[0] = 0.05f; m->vis.headlight.ambient[1] = 0.08f; m->vis.headlight.ambient[2] = 0.18f;
    m->vis.headlight.diffuse[0] = 0.65f; m->vis.headlight.diffuse[1] = 0.72f; m->vis.headlight.diffuse[2] = 0.92f;
    m->vis.headlight.specular[0] = 0.4f; m->vis.headlight.specular[1] = 0.45f; m->vis.headlight.specular[2] = 0.6f;

    con.fogRGBA[0] = 0.03f;
    con.fogRGBA[1] = 0.05f;
    con.fogRGBA[2] = 0.12f;
    con.fogRGBA[3] = 1.0f;
    con.fogStart = 2.5f * m->stat.extent;  
    con.fogEnd = 8.0f * m->stat.extent;  

    scn.flags[mjRND_FOG] = 1;
    scn.flags[mjRND_HAZE] = 1;
    scn.flags[mjRND_SHADOW] = 1;
    scn.flags[mjRND_REFLECTION] = 1;


	// 初始化ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    // 加载字体
    ImFontConfig baseCfg;
    baseCfg.OversampleH = 2;
    baseCfg.OversampleV = 2;
    io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/consola.ttf", 15.5f, &baseCfg);

    ImFontConfig mergeCfg;
    mergeCfg.MergeMode = true;
    mergeCfg.OversampleH = 1;
    mergeCfg.OversampleV = 1;
    io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/msyh.ttc", 15.5f, &mergeCfg,
        io.Fonts->GetGlyphRangesChineseFull());

    // 应用优化后暗黑主题
    apply_dark_theme();

    ImGui_ImplGlfw_InitForOpenGL(window, false);
    ImGui_ImplOpenGL3_Init("#version 130");


    // 为菜单背景做一次初始场景更新
    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);


	// 主循环
    while (!glfwWindowShouldClose(window))
    {
        m->opt.timestep = baseTimestep * (double)timeScale;

        int W, H;
        glfwGetFramebufferSize(window, &W, &H);
        mjrRect viewport = { 0, 0, W, H };

        if (currentMode == AppMode::MENU) {
            mjr_render(viewport, &scn, &con);   // 渲染3D场景到OpenGL
        }
        else {
            if (currentMode == AppMode::MODE1) mode1_step(m, d);
            else if (currentMode == AppMode::MODE2) mode2_step(m, d);

            mjvPerturb* pertPtr = (currentMode == AppMode::MODE1) ? &pert : nullptr;    // MODE 2 禁用ctrl扰动
            mjv_updateScene(m, d, &opt, pertPtr, &cam, mjCAT_ALL, &scn);

            if (currentMode == AppMode::MODE1) mode1_inject_geoms(scn);

            mjr_render(viewport, &scn, &con);
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (currentMode == AppMode::MENU)  render_menu();
        else if (currentMode == AppMode::MODE1) mode1_render_ui();
        else if (currentMode == AppMode::MODE2) mode2_render_ui();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    if (currentMode == AppMode::MODE1) mode1_cleanup();
    else if (currentMode == AppMode::MODE2) mode2_cleanup();

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