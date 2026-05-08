#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstring>

// MuJoCo 全局对象
mjModel* m = nullptr;
mjData* d = nullptr;
mjvScene     scn;
mjvCamera    cam;
mjvOption    opt;
mjrContext   con;

int main()
{
    // 1. 加载模型（先用一个内置的简单测试模型）
    const char* xml = R"(
        <mujoco>
          <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
            <geom type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>
            <body pos="0 0 1">
              <joint type="free"/>
              <geom type="sphere" size="0.1" rgba="1 0 0 1"/>
            </body>
          </worldbody>
        </mujoco>
    )";

    char error[1000];
    m = mj_loadXML(nullptr, nullptr, error, 1000);
    // 用字符串加载
    m = mj_loadXML(nullptr, nullptr, error, 1000);

    // 直接从字符串加载
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    mj_addBufferVFS(&vfs, "test.xml", xml, strlen(xml));
    m = mj_loadXML("test.xml", &vfs, error, 1000);

    if (!m) {
        printf("模型加载失败: %s\n", error);
        return 1;
    }
    d = mj_makeData(m);

    // 2. 初始化 GLFW 窗口
    if (!glfwInit()) {
        printf("GLFW 初始化失败\n");
        return 1;
    }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo 灵巧手仿真", nullptr, nullptr);
    if (!window) {
        printf("窗口创建失败\n");
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // 3. 初始化 MuJoCo 渲染器
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 4. 主循环
    while (!glfwWindowShouldClose(window)) {
        // 物理步进
        mj_step(m, d);

        // 获取窗口尺寸
        int W, H;
        glfwGetFramebufferSize(window, &W, &H);
        mjrRect viewport = { 0, 0, W, H };

        // 渲染
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 5. 清理
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}