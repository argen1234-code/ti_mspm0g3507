#include "delay.h"
#include "ti_msp_dl_config.h"  // 确保包含这个头文件

// SysTick计数器（每1ms增加）
static volatile uint32_t systick_count = 0;

// 定义系统时钟频率（根据实际配置修改）
#define SYSTEM_CLOCK_FREQ 32000000  // 32MHz

// SysTick中断处理函数
void SysTick_Handler(void)
{
    systick_count++;
}

// 初始化延时系统
void Delay_Init(void)
{
    // 配置SysTick每1ms中断一次
    SysTick_Config(SYSTEM_CLOCK_FREQ / 1000);
    
    // 设置SysTick中断优先级为最低
    NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
}

// 毫秒级精确延时
void Delay_ms(uint32_t ms)
{
    uint32_t start = systick_count;
    
    // 处理长时间延时（防止计数器溢出）
    while (ms > 500) {
        while ((systick_count - start) < 500);
        ms -= 500;
        start += 500;
    }
    
    // 等待剩余时间
    while ((systick_count - start) < ms);
}

// 微秒级精确延时
void Delay_us(uint32_t us)
{
    // 对于大于1ms的延时，使用毫秒延时函数
    if (us > 1000) {
        Delay_ms(us / 1000);
        us %= 1000;
    }
    
    // 计算需要的循环次数
    // 每个循环约4个时钟周期（根据编译器优化）
    volatile uint32_t cycles = (us * (SYSTEM_CLOCK_FREQ / 1000000)) / 4;
    
    // 精确循环延时
    while (cycles--) {
        __NOP(); // 空操作指令
    }
}