#include "fuzzy_pid.h"
/*
步骤 1：确立基础值 (Base)
先别管模糊控制，把 weight 都设为 0.0f（这样它就是一个普通PID）。
利用你以前调普通PID的经验，填好 Kp_base, Ki_base, Kd_base，只要能让系统差不多稳定就行。

步骤 2：划定边界 (Max)
你需要观察你的温控系统：
e_max：你加热时，通常起始温度和目标温度最大差多少度？如果室温20度，加热到70度，最大温差是50，那么 e_max = 50.0f。
ec_max：在10ms的周期里，你的温度传感器数值最大跳动/变化大概是多少度？比如加热块升温很慢，10ms最多变0.5度，那你最好设置 ec_max = 0.5f。（如果在10ms内温度几乎不变，可以考虑累积10次误差再算 ec，或者把 ec_max 设得非常小）。

步骤 3：赋予“老司机”权力 (Weight)
现在开始开启模糊补偿，调整 weight（权重）：
如果你觉得温度上升太慢，就把 Kp_weight 给到 5.0 甚至 10.0。这样在温差大的时候，模糊规则会自动把 Kp 顶得非常高。
如果你觉得温度在快达到目标时总是超调（冲过头），就把 Kd_weight 调大。
如果你觉得稳态总是差那么一两度上不去，就把 Ki_weight 给一点（比如 0.1或0.2）。

总结：
你不需要写任何复杂的逻辑判断了，只要看机器运行的曲线：
曲线太肉 -> 加 Kp_weight
曲线震荡 -> 加 Kd_weight 或减小 Kp_weight
*/


// 模糊规则表 (横轴:误差变化率ec, 纵轴:误差e)
// 范围从 NB(负大), NM(负中), NS(负小), ZO(零), PS(正小), PM(正中), PB(正大) (对应索引 0~6)

// Kp 规则表：误差大时Kp要大(快速响应)；误差小时Kp要小(防止超调)
static const float rule_Kp[7][7] = {
    { 3.0,  3.0,  2.0,  2.0,  1.0,  0.0,  0.0},
    { 3.0,  3.0,  2.0,  1.0,  1.0,  0.0, -1.0},
    { 2.0,  2.0,  2.0,  1.0,  0.0, -1.0, -1.0},
    { 2.0,  2.0,  1.0,  0.0, -1.0, -2.0, -2.0},
    { 1.0,  1.0,  0.0, -1.0, -1.0, -2.0, -2.0},
    { 1.0,  0.0, -1.0, -2.0, -2.0, -3.0, -3.0},
    { 0.0,  0.0, -1.0, -2.0, -2.0, -3.0, -3.0}
};

// Ki 规则表：误差大时Ki要小(防止积分饱和)；误差小时Ki要大(消除静差)
static const float rule_Ki[7][7] = {
    {-3.0, -3.0, -2.0, -2.0, -1.0,  0.0,  0.0},
    {-3.0, -3.0, -2.0, -1.0, -1.0,  0.0,  0.0},
    {-3.0, -2.0, -1.0, -1.0,  0.0,  1.0,  1.0},
    {-2.0, -2.0, -1.0,  0.0,  1.0,  2.0,  2.0},
    {-2.0, -1.0,  0.0,  1.0,  1.0,  2.0,  3.0},
    { 0.0,  0.0,  1.0,  1.0,  2.0,  3.0,  3.0},
    { 0.0,  0.0,  1.0,  2.0,  2.0,  3.0,  3.0}
};

// Kd 规则表：误差大时Kd要小(避免高频噪声)；误差适中时Kd要大(抑制超调)
static const float rule_Kd[7][7] = {
    { 1.0, -1.0, -3.0, -3.0, -3.0, -2.0,  1.0},
    { 1.0, -1.0, -3.0, -2.0, -2.0, -1.0,  0.0},
    { 0.0, -1.0, -2.0, -2.0, -1.0, -1.0,  0.0},
    { 0.0, -1.0, -2.0, -1.0, -1.0, -1.0,  0.0},
    { 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
    { 3.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0},
    { 3.0,  2.0,  1.0,  1.0,  1.0,  1.0,  1.0}
};

void FuzzyPID_init(FuzzyPID_struct* fuzzy_pid) {
    fuzzy_pid->v_target = 0.0f;
    fuzzy_pid->v_current = 0.0f;
    fuzzy_pid->err = 0.0f;
    fuzzy_pid->err_prev_1 = 0.0f;
    fuzzy_pid->err_prev_2 = 0.0f;
    fuzzy_pid->speed[0] = fuzzy_pid->speed[1] = fuzzy_pid->speed[2] = 0.0f;
    
    // 初始化你原本的普通PID参数 (参考你之前设定的40, 0.8, 125)
    fuzzy_pid->Kp_base = 40.0f;
    fuzzy_pid->Ki_base = 0.8f;
    fuzzy_pid->Kd_base = 125.0f;

    // ----- 需要你根据实际情况微调的边界参数 -----
    fuzzy_pid->e_max = 50.0f;     // 默认：温差50度算“极大误差”
    fuzzy_pid->ec_max = 5.0f;     // 默认：单次采样温差变5度算“极大突变”
    
    fuzzy_pid->Kp_weight = 5.0f;  // 默认：Kp最大允许增减 5.0 * 3.0 = 15.0
    fuzzy_pid->Ki_weight = 0.1f;  // 默认：Ki最大允许增减 0.1 * 3.0 = 0.3
    fuzzy_pid->Kd_weight = 10.0f; // 默认：Kd最大允许增减 10.0 * 3.0 = 30.0
}

static void Fuzzy_Bound_Fix(float* value, float min_val, float max_val) {
    if (*value < min_val) *value = min_val;
    if (*value > max_val) *value = max_val;
}

float FuzzyPID_Calculate(FuzzyPID_struct* fuzzy_pid, float value_thres, float value_current) {
    fuzzy_pid->v_target = value_thres;
    fuzzy_pid->v_current = value_current;
    
    // 1. 计算误差 e 和 误差变化率 ec
    fuzzy_pid->err = fuzzy_pid->v_target - fuzzy_pid->v_current;
    float ec = fuzzy_pid->err - fuzzy_pid->err_prev_1;
    
    // 2. 映射到论域 [-3, +3] (归一化模糊处理)
    float e_fuzzy = (fuzzy_pid->err / fuzzy_pid->e_max) * 3.0f;
    float ec_fuzzy = (ec / fuzzy_pid->ec_max) * 3.0f;
    
    // 限制在 -3 到 +3 之间
    Fuzzy_Bound_Fix(&e_fuzzy, -3.0f, 3.0f);
    Fuzzy_Bound_Fix(&ec_fuzzy, -3.0f, 3.0f);
    
    // 3. 将浮点数转换为 0~6 的整数索引 (四舍五入)
    int e_index = (int)(e_fuzzy + 3.5f); 
    int ec_index = (int)(ec_fuzzy + 3.5f);
    
    if(e_index > 6) e_index = 6; if(e_index < 0) e_index = 0;
    if(ec_index > 6) ec_index = 6; if(ec_index < 0) ec_index = 0;
    
    // 4. 查表获取补偿值，并计算实际的 PID
    float delta_Kp = rule_Kp[e_index][ec_index];
    float delta_Ki = rule_Ki[e_index][ec_index];
    float delta_Kd = rule_Kd[e_index][ec_index];
    
    fuzzy_pid->Kp = fuzzy_pid->Kp_base + (delta_Kp * fuzzy_pid->Kp_weight);
    fuzzy_pid->Ki = fuzzy_pid->Ki_base + (delta_Ki * fuzzy_pid->Ki_weight);
    fuzzy_pid->Kd = fuzzy_pid->Kd_base + (delta_Kd * fuzzy_pid->Kd_weight);
    
    // 保证PID不为负数
    if(fuzzy_pid->Kp < 0.0f) fuzzy_pid->Kp = 0.0f;
    if(fuzzy_pid->Ki < 0.0f) fuzzy_pid->Ki = 0.0f;
    if(fuzzy_pid->Kd < 0.0f) fuzzy_pid->Kd = 0.0f;

    // 5. 执行标准PID运算
    float speed_p = fuzzy_pid->Kp * fuzzy_pid->err;
    fuzzy_pid->err_prev_2 = fuzzy_pid->err_prev_2 + fuzzy_pid->err;
    float speed_i = fuzzy_pid->Ki * fuzzy_pid->err_prev_2;
    float speed_d = fuzzy_pid->Kd * ec;
    
    fuzzy_pid->speed[0] = speed_p;
    fuzzy_pid->speed[1] = speed_i;
    fuzzy_pid->speed[2] = speed_d;
    fuzzy_pid->err_prev_1 = fuzzy_pid->err;
    
    float speed = speed_p + speed_i + speed_d;
    
    // 输出限幅 (跟你的普通PID一致)
    if (speed > 1000.0f) return 1000.0f;
    if (speed < 1.0f) return 1.0f;
    
    return speed;
}
