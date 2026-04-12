#include "thermocouple.h"
#include "ads1118.h"

// NIST T型热电偶多项式系数 (0℃ ~ 400℃ 范围)
// 温度(℃) 转 电压(mV) 的多项式系数
const float c_T2V[7] = {
    0.0f,
    3.8748106e-2f,
    3.3292227e-5f,
    2.0618243e-7f,
    -2.1882256e-10f,
    1.0996880e-12f,
    -1.5682758e-15f
};

// 电压(mV) 转 温度(℃) 的多项式系数
const float c_V2T[7] = {
    0.0f,
    25.928f,
    -0.7602961f,
    0.04637791f,
    -0.002165394f,
    0.00006048144f,
    -7.293422e-7f
};

// 冷端补偿电压缓存 (单位: mV)，初始值设为常温 25℃ 对应电压
static float cached_cold_junction_vol = 1.0f;

// 将冷端温度(环境温度)转换为对应冷端补偿电压 (mV)
float Temp_To_Voltage_T_Type(float temp) {
    float voltage = 0.0f;
    float temp_pow = temp;

    // 使用多项式公式: V = c1*T + c2*T^2 + c3*T^3 ...
    // 0℃ 时电压为 0，所以跳过 c_T2V[0]
    for (int i = 1; i < 7; i++) {
        voltage += c_T2V[i] * temp_pow;
        temp_pow *= temp;
    }
    return voltage;
}

// 将总电压(mV)转换为实际温度 (℃)
float Voltage_To_Temp_T_Type(float voltage) {
    float temp = 0.0f;
    float vol_pow = voltage;

    // 使用多项式公式: T = d1*V + d2*V^2 + d3*V^3 ...
    for (int i = 1; i < 7; i++) {
        temp += c_V2T[i] * vol_pow;
        vol_pow *= voltage;
    }
    return temp;
}

// 更新冷端补偿电压缓存 (200ms 调用一次)
void Thermocouple_UpdateColdJunction(void) {
    // 获取芯片内部温度 (环境温度)
    float cold_junction_temp = ADS1118_GetInternalTemp();
    // 将环境温度转换为对应的冷端补偿电压并缓存
    cached_cold_junction_vol = Temp_To_Voltage_T_Type(cold_junction_temp);
}

// 获取热电偶实际温度 (10ms 高频调用)
float Get_Actual_Temperature(void) {
    // 获取热电偶产生的差分电压 (mV)
    float tc_voltage = ADS1118_GetVoltage_mV();
    // 加上冷端补偿电压得到总电压
    float total_voltage = tc_voltage + cached_cold_junction_vol;
    // 将总电压转换为最终实际温度
    float final_temp = Voltage_To_Temp_T_Type(total_voltage);
    return final_temp;
}
