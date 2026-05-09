#include "controller.h"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstring>

mjModel* model = nullptr;
mjData* data = nullptr;
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

	mjv_moveCamera(model, action, dx / height, dy / height, &scn, &cam);
}

// 滚轮回调（缩放）
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
	mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

int main(void) {
	// 加载模型
	char error[1000];
	model = mj_loadXML("model_formal/Adroit/Adroit_hand.xml", nullptr, error, 1000);
	if (!model) { printf("模型加载失败: %s\n", error); return 1; }
	data = mj_makeData(model);

	// 初始化控制器
	controller_init(model, data);


	// 初始化GLFW窗口
	if (!glfwInit()) return 1;
	GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo 灵巧手仿真", nullptr, nullptr);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// 注册鼠标回调
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetScrollCallback(window, scroll);

	// 初始化Mujoco渲染器
	mjv_defaultCamera(&cam);
	mjv_defaultOption(&opt);
	mjv_makeScene(model, &scn, 2000);
	mjr_makeContext(model, &con, mjFONTSCALE_150);

	// 主循环
	while (!glfwWindowShouldClose(window)) {
		for (int i = 0; i < 2; i++) {
			controller_step(model, data);
			mj_step(model, data);
		}

		int windowWidth, windowHeight;
		glfwGetFramebufferSize(window, &windowWidth, &windowHeight);
		mjrRect viewport = { 0, 0, windowWidth, windowHeight };

		mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	// 清理资源
	mjr_freeContext(&con);
	mjv_freeScene(&scn);
	mj_deleteData(data);
	mj_deleteModel(model);
	glfwTerminate();
	return 0;
}