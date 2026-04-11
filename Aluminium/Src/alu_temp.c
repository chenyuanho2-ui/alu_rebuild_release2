#include "alu_temp.h"
#include "temp_filter.h"  // 新增：引入我们的双通道测温接口


// 劫持原有的测温接口
double alu_SPI_gettemp(void)
{
    // 直接获取【通道A】的 20ms 实时平滑数据，转为 double 给 PID 用
    // 这样原来的控制任务只要调用这个函数，拿到的就是 ADS1118 的最新数据
    return (double)TempFilter_GetLatestAvgTemp();
}
