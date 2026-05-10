#include "mode2.h"
#include <cstring>
#include <cmath>
#include <cstdio>
#include <vector>

// 可调参数
static const char* TARGET_JOINT = "FFJ2";
static const char* TARGET_ACT = "A_FFJ2";
static constexpr double EXCITE_AMP = 0.6;   // rad幅度（0.8±0.6 在 [0,1.6] 内）
static constexpr double EXCITE_FREQ = 1.0;   // Hz
static constexpr double WARMUP_TIME = 0.5;   // s
static constexpr int    WINDOW_SIZE = 50;   // 滑动窗口大小
static constexpr int    LS_INTERVAL = 3;    // 每3个新样本求解一次
static constexpr int    HIST_LEN = 200;

// 运行状态
static int    targetDof = -1;
static int    targetAct = -1;
static double J_true = 1.0;
static double B_true = 0.0;
static double J_hat = 0.0;
static double B_hat = 0.0;
static double elapsed = 0.0;
static bool   collecting = true;  // 未达MAX_SAMPLES时持续采样

// LS（最小二乘法） 数据缓冲
static double lsA[WINDOW_SIZE][2];  // 循环缓冲，只保留最近50个样本
static double lsT[WINDOW_SIZE];
static int    lsCount = 0;          // 累计总样本数（可超过WINDOW_SIZE）
static int    lsSinceLastSolve = 0;

// 收敛历史（环形缓冲）
static float  J_errHist[HIST_LEN];
static float  B_errHist[HIST_LEN];
static int    histHead = 0;
static int    histCount = 0;

// 参数调节
static float  editArmature = 0.0f;  // 附加惯量 dof_armature，直接影响 J_true
static float  editDamping = 0.0f;  // 阻尼 dof_damping，即 B_true


// 内部函数
// 2×2 法方程解析求解
static void solve_ls()
{
    int n = (lsCount < WINDOW_SIZE) ? lsCount : WINDOW_SIZE;
    if (n < 4) return;

    double ATA00 = 0, ATA01 = 0, ATA11 = 0;
    double ATb0 = 0, ATb1 = 0;

    for (int i = 0; i < n; i++) {
        double a0 = lsA[i][0], a1 = lsA[i][1], b = lsT[i];
        ATA00 += a0 * a0;
        ATA01 += a0 * a1;
        ATA11 += a1 * a1;
        ATb0 += a0 * b;
        ATb1 += a1 * b;
    }
    double det = ATA00 * ATA11 - ATA01 * ATA01;
    if (fabs(det) < 1e-14) return;
    J_hat = (ATA11 * ATb0 - ATA01 * ATb1) / det;
    B_hat = (ATA00 * ATb1 - ATA01 * ATb0) / det;
}

static void push_history()
{
    J_errHist[histHead] = (float)fabs(J_hat - J_true);
    B_errHist[histHead] = (float)fabs(B_hat - B_true);
    histHead = (histHead + 1) % HIST_LEN;
    if (histCount < HIST_LEN) histCount++;
}

static void reset_estimation()
{
    lsCount = 0;
    lsSinceLastSolve = 0;
    histHead = 0;
    histCount = 0;
    J_hat = B_hat = 0.0;
    collecting = true;
    elapsed = 0.0;
    memset(J_errHist, 0, sizeof(J_errHist));
    memset(B_errHist, 0, sizeof(B_errHist));
}

static void apply_params(mjModel* m, mjData* d)
{
    if (targetDof < 0) return;
    if (editArmature < 0.0f) editArmature = 0.0f;
    if (editDamping < 0.0f) editDamping = 0.0f;

    // 写入模型
    m->dof_armature[targetDof] = (mjtNum)editArmature;
    m->dof_damping[targetDof] = (mjtNum)editDamping;
    B_true = editDamping;

    // 重新计算 J_true（armature 已计入质量矩阵对角元）
    mj_resetData(m, d);
    mj_forward(m, d);
    int nv = m->nv;
    std::vector<mjtNum> M((size_t)nv * nv, 0.0);
    mj_fullM(m, M.data(), d->qM);
    J_true = M[(size_t)targetDof * nv + targetDof];

    reset_estimation();
}



// 对外接口，由 main.cpp 调用

void mode2_init(mjModel* m, mjData* d)
{
    mj_resetData(m, d);
    reset_estimation();

    // 查找目标关节 DOF
    int jid = mj_name2id(m, mjOBJ_JOINT, TARGET_JOINT);
    if (jid >= 0) {
        targetDof = m->jnt_dofadr[jid];
    }
    else {
        targetDof = -1;
        printf("[Mode2] Warning: joint '%s' not found\n", TARGET_JOINT);
    }

    // 查找执行器
    targetAct = mj_name2id(m, mjOBJ_ACTUATOR, TARGET_ACT);
    if (targetAct < 0)
        printf("[Mode2] Warning: actuator '%s' not found\n", TARGET_ACT);

    // 真实阻尼
    B_true = (targetDof >= 0) ? m->dof_damping[targetDof] : 0.0;

    // 真实惯量：质量矩阵对角元
    mj_forward(m, d);
    if (targetDof >= 0) {
        int nv = m->nv;
        std::vector<mjtNum> M((size_t)nv * nv, 0.0);
        mj_fullM(m, M.data(), d->qM);
        J_true = M[(size_t)targetDof * nv + targetDof];
    }
    else {
        J_true = 1.0;
    }

    printf("[Mode2] J_true = %.6f   B_true = %.6f\n", J_true, B_true);

    // 初始化编辑器为当前模型值（新增）
    editArmature = (targetDof >= 0) ? (float)m->dof_armature[targetDof] : 0.0f;
    editDamping = (targetDof >= 0) ? (float)m->dof_damping[targetDof] : 0.0f;
}

void mode2_step(mjModel* m, mjData* d)
{
    elapsed += m->opt.timestep;

    // 所有关节归零，仅激励目标关节
    mju_zero(d->ctrl, m->nu);
    if (targetAct >= 0)
        d->ctrl[targetAct] = 0.8 + EXCITE_AMP * sin(2.0 * mjPI * EXCITE_FREQ * elapsed);

    mj_step(m, d);

    // 热身期结束后采样
    if (elapsed <= WARMUP_TIME || targetDof < 0 || !collecting) return;

    double q_ddot = d->qacc[targetDof];
    double q_dot = d->qvel[targetDof];
    double tau = d->qfrc_actuator[targetDof] - d->qfrc_bias[targetDof];

    int idx = lsCount % WINDOW_SIZE;   // 循环写入
    lsA[idx][0] = q_ddot;
    lsA[idx][1] = q_dot;
    lsT[idx] = tau;
    lsCount++;
    lsSinceLastSolve++;

    // 去掉 collecting = false 的条件，让采样持续进行

    if (lsSinceLastSolve >= LS_INTERVAL) {
        solve_ls();
        push_history();
        lsSinceLastSolve = 0;
    }
}

void mode2_render_ui()
{
    // 窗口1：控制 + 参数对比
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(340, 300), ImGuiCond_Always);
    ImGui::Begin("参数辨识控制", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

    ImGui::SliderFloat("Time Scale", &timeScale, 0.0f, 3.0f, "%.2fx");
    ImGui::Spacing();
    ImGui::TextDisabled("激励：%s   %.2f rad @ %.1f Hz", TARGET_JOINT, EXCITE_AMP, EXCITE_FREQ);
    int inWindow = (lsCount < WINDOW_SIZE) ? lsCount : WINDOW_SIZE;
    ImGui::Text("滑动窗口：%d / %d    累计：%d", inWindow, WINDOW_SIZE, lsCount);
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Columns(3, "params", true);
    ImGui::TextDisabled("参数");   ImGui::NextColumn();
    ImGui::TextDisabled("真实值"); ImGui::NextColumn();
    ImGui::TextDisabled("估计值"); ImGui::NextColumn();
    ImGui::Separator();

    ImGui::Text("J (惯量)"); ImGui::NextColumn();
    ImGui::Text("%.5f", J_true); ImGui::NextColumn();
    {
        bool bad = (lsCount > 10) && (fabs(J_hat - J_true) > 0.25 * (fabs(J_true) + 1e-12));
        ImGui::PushStyleColor(ImGuiCol_Text, bad ? ImVec4(1.f, 0.4f, 0.4f, 1.f) : ImVec4(0.4f, 1.f, 0.4f, 1.f));
        ImGui::Text("%.5f", J_hat);
        ImGui::PopStyleColor();
    }
    ImGui::NextColumn();

    ImGui::Text("B (阻尼)"); ImGui::NextColumn();
    ImGui::Text("%.5f", B_true); ImGui::NextColumn();
    {
        double ref = fabs(B_true) > 1e-10 ? fabs(B_true) : 1e-4;
        bool bad = (lsCount > 10) && (fabs(B_hat - B_true) > 0.25 * ref);
        ImGui::PushStyleColor(ImGuiCol_Text, bad ? ImVec4(1.f, 0.4f, 0.4f, 1.f) : ImVec4(0.4f, 1.f, 0.4f, 1.f));
        ImGui::Text("%.5f", B_hat);
        ImGui::PopStyleColor();
    }
    ImGui::NextColumn();

    ImGui::Columns(1);
    ImGui::Spacing();
    if (ImGui::Button("重置估计", ImVec2(150, 0))) {
        mj_resetData(m, d);
        reset_estimation();
    }
    ImGui::SameLine();
    if (ImGui::Button("返回主菜单", ImVec2(-1, 0))) {
        mode2_cleanup();
        currentMode = AppMode::MENU;
    }
    ImGui::End();

    // 窗口2：收敛曲线
    ImGui::SetNextWindowPos(ImVec2(10, 320), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(340, 270), ImGuiCond_Always);
    ImGui::Begin("误差收敛曲线", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

    if (histCount < 2) {
        ImGui::TextDisabled("等待热身期结束并积累足够样本...");
    }
    else {
        static float tmpJ[HIST_LEN], tmpB[HIST_LEN];
        float maxJ = 1e-8f, maxB = 1e-8f;
        for (int i = 0; i < histCount; i++) {
            int idx = (histHead - histCount + i + HIST_LEN) % HIST_LEN;
            tmpJ[i] = J_errHist[idx];
            tmpB[i] = B_errHist[idx];
            if (tmpJ[i] > maxJ) maxJ = tmpJ[i];
            if (tmpB[i] > maxB) maxB = tmpB[i];
        }
        char ovlJ[48], ovlB[48];
        snprintf(ovlJ, sizeof(ovlJ), "cur=%.5f", tmpJ[histCount - 1]);
        snprintf(ovlB, sizeof(ovlB), "cur=%.5f", tmpB[histCount - 1]);

        ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "|J_hat - J_true|");
        ImGui::PushStyleColor(ImGuiCol_PlotLines, ImVec4(0.4f, 1.0f, 0.4f, 1.0f));
        ImGui::PlotLines("##jerr", tmpJ, histCount, 0, ovlJ, 0.0f, maxJ * 1.3f, ImVec2(-1, 95));
        ImGui::PopStyleColor();
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "|B_hat - B_true|");
        ImGui::PushStyleColor(ImGuiCol_PlotLines, ImVec4(1.0f, 0.8f, 0.3f, 1.0f));
        ImGui::PlotLines("##berr", tmpB, histCount, 0, ovlB, 0.0f, maxB * 1.3f, ImVec2(-1, 95));
        ImGui::PopStyleColor();
    }
    ImGui::End();

    // 窗口3：参数调节
    ImGui::SetNextWindowPos(ImVec2(10, 600), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(340, 155), ImGuiCond_Always);
    ImGui::Begin("参数调节 & 重新实验", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

    ImGui::TextDisabled("修改参数后点击「应用并重置」开始新一轮实验");
    ImGui::Spacing();
    ImGui::Text("附加惯量 armature");
    ImGui::SameLine(); ImGui::SetNextItemWidth(-1);
    ImGui::DragFloat("##armature", &editArmature, 0.0001f, 0.0f, 1.0f, "%.4f");
    ImGui::Text("阻    尼 damping ");
    ImGui::SameLine(); ImGui::SetNextItemWidth(-1);
    ImGui::DragFloat("##damping", &editDamping, 0.001f, 0.0f, 5.0f, "%.4f");
    ImGui::Spacing();
    ImGui::TextDisabled("当前  J_true = %.5f    B_true = %.5f", J_true, B_true);
    ImGui::Spacing();
    if (ImGui::Button("应用并重置", ImVec2(-1, 28)))
        apply_params(m, d);

    ImGui::End(); 
}


void mode2_cleanup()
{
    lsCount = 0;
    elapsed = 0.0;
    histHead = 0;
    histCount = 0;
    collecting = true;
}