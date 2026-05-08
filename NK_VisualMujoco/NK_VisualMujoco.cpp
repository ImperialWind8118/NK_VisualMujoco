#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstring>

mjModel* m = nullptr;
mjData* d = nullptr;
mjvScene     scn;
mjvCamera    cam;
mjvOption    opt;
mjrContext   con;

// 鼠标状态
bool  button_left = false;
bool  button_right = false;
double lastx = 0, lasty = 0;

// 鼠标按键回调
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

// 鼠标移动回调（拖拽旋转/平移）
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_right) return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    mjtMouse action;
    if (button_right)
        action = mjMOUSE_MOVE_V;        // 右键：上下平移
    else if (button_left)
        action = mjMOUSE_ROTATE_V;      // 左键：旋转

    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// 滚轮回调（缩放）
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

int main() {
    char error[1000];
    m = mj_loadXML("model_formal/Adroit/Adroit_hand.xml", nullptr, error, 1000);
    if (!m) { printf("模型加载失败: %s\n", error); return 1; }
    d = mj_makeData(m);

    if (!glfwInit()) return 1;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo 灵巧手仿真", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // 注册鼠标回调
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    while (!glfwWindowShouldClose(window)) {
        mj_step(m, d);

        int W, H;
        glfwGetFramebufferSize(window, &W, &H);
        mjrRect viewport = { 0, 0, W, H };

        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}