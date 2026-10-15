#ifndef __LED_H
#define __LED_H
#include <stdio.h>
#include "mm32_device.h"
#include "hal_conf.h"

/*
 * V366 只有 PA10/COM3 经 R63、Q15(2N7002) 驱动的单路照明。
 * PA13 是 EN 供电保持，PA2/PA4 是 ADC，不能恢复旧六灯复用矩阵。
 * 单灯适配：机械开关 ON 正常照明，ON 故障闪码，OFF 或掉电时熄灭。
 * 原三灯电量/档位组合无法在本原理图上直接显示，不复用测量引脚模拟它。
 */
#define LED_LIGHT_RCC_CLOCKGPIO     (RCC_AHBENR_GPIOA_Msk)
#define LED_LIGHT_PORT              GPIOA
#define LED_LIGHT_PIN               GPIO_Pin_10
#define LED_ERROR_ON()              GPIO_SetBits(LED_LIGHT_PORT, LED_LIGHT_PIN)
#define LED_ERROR_OFF()             GPIO_ResetBits(LED_LIGHT_PORT, LED_LIGHT_PIN)

/* 沿用旧产品故障闪烁编号及优先级，只把三灯同时闪适配成 PA10 单灯闪。 */
#define LED_FAULT_HW_OCP            (1)     /* 硬件/ADC 快速过流 */
#define LED_FAULT_BAT_TEMP          (2)     /* 电池温度；本板功能关闭 */
#define LED_FAULT_MOS_TEMP          (3)     /* MOS 温度/过载 */
#define LED_FAULT_BLOCK             (4)     /* 堵转或失步 */
#define LED_FAULT_LOSE_STEP         (4)
#define LED_FAULT_IBUS_OCP1         (5)     /* 峰值积分与平均电流一至三级 */
#define LED_FAULT_IBUS_OCP2         (5)
#define LED_FAULT_PHASE_OCP_AVG     (5)
#define LED_FAULT_PHASE_OCP_HW      (11)    /* 平均电流第四级沿用旧其它故障码 */
#define LED_FAULT_UVP               (6)
#define LED_FAULT_FLY_MACHINE       (7)
#define LED_FAULT_OVP               (8)
#define LED_FAULT_MOS_BRIDGE        (9)
#define LED_FAULT_LACK_PHASE        (10)
#define LED_FAULT_OTHER             (11)
#define LED_FAULT_BAT_COM           (11)    /* 通信关闭；若有锁存则按其它故障显示 */

typedef struct
{
	uint8_t     LedDeal;        // 新一批故障闪烁请求
	uint8_t     LedRunMode;     // 1 为旧产品单段亮灭节奏
	uint16_t    LedRunTimes;    // 剩余显示轮数，每批十轮
	uint16_t    LedRunCnt_Once; // 每轮闪烁次数，即故障编号
} User_LED_Control_t;

typedef enum
{
	LIGHT_OFF=0,
	LIGHT_ON,
	LIGHT_WAIT_10S, // 保留旧接口枚举；V366 不使用延时照明
	LIGHT_WAIT_30S,
	LIGHT_FAULT
} Light_State_e;

typedef struct
{
	Light_State_e state;
	uint32_t cnt;
} Light_Ctrl_t;

extern User_LED_Control_t user_led_control;
extern Light_Ctrl_t light_ctrl;
void led_init(void);
void UO_Led_Handler(void);
uint8_t LedError_Control(void);
void LedError_Run(void);
void Light_Handler(void);

#endif
