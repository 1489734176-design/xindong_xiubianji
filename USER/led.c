#include "board.h"
#include "led.h"
#include "user_control.h"
#include "parameter.h"

User_LED_Control_t user_led_control;
Light_Ctrl_t light_ctrl;

/** @brief 只初始化 PA10 照明，不操作 PA13/EN 或 ADC 复用引脚。 */
void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHBPeriphClockCmd(LED_LIGHT_RCC_CLOCKGPIO, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin=LED_LIGHT_PIN;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_High;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(LED_LIGHT_PORT, &GPIO_InitStruct);
	LED_ERROR_OFF();
	light_ctrl.state=LIGHT_OFF;
	light_ctrl.cnt=0;
	user_led_control.LedDeal=0;
	user_led_control.LedRunTimes=0;
	user_led_control.LedRunCnt_Once=0;
}

/**
 * @brief V366 单灯适配。正常 ON 照明，故障 ON 闪码，OFF 立即灭。
 *        旧独立照明代码未启用，不引入未经需求确认的 10s/30s 延时；
 *        供电保持由工具状态机按旧 5s 空闲规则处理。
 */
void Light_Handler(void)
{
	light_ctrl.cnt=0;
	if(user_list.user_state==TOOL_POWER_DOWN || !user_list.flag.bits.gToolTrigger)
	{
		light_ctrl.state=LIGHT_OFF;
	}
	else if(machine_error_hold.userErr_hold.word || machine_error_hold.MC_error_hold.word)
	{
		light_ctrl.state=LIGHT_FAULT;
	}
	else
	{
		light_ctrl.state=LIGHT_ON;
	}
}

void UO_Led_Handler(void)
{
	Light_Handler();
	if(light_ctrl.state==LIGHT_FAULT && LedError_Control())
	{
		LedError_Run();
	}
	else
	{
		/* 松开开关后重置故障闪烁节拍，下次按下从一整轮开始。 */
		user_led_control.LedDeal=0;
		user_led_control.LedRunTimes=0;
		user_led_control.LedRunCnt_Once=0;
		LedError_Run();
		if(light_ctrl.state==LIGHT_ON)
		{
			LED_ERROR_ON();
		}
	}
}

/** @brief 旧故障优先级采用结构位名翻译，不搬旧错误字的数值掩码。 */
uint8_t LedError_Control(void)
{
	if(machine_error_hold.userErr_hold.word==0 && machine_error_hold.MC_error_hold.word==0)
	{
		user_led_control.LedDeal=0;
		user_led_control.LedRunTimes=0;
		user_led_control.LedRunCnt_Once=0;
		return 0;
	}
	if(!user_led_control.LedDeal && !user_led_control.LedRunTimes)
	{
		user_led_control.LedDeal=1;
		user_led_control.LedRunTimes=10;
		user_led_control.LedRunMode=1;
		if(machine_error_hold.userErr_hold.bits.gIBusAcmpOCP)
		{
			user_led_control.LedRunCnt_Once=LED_FAULT_HW_OCP;
		}
		else if(machine_error_hold.userErr_hold.bits.gNtcBat)
		{
			user_led_control.LedRunCnt_Once=LED_FAULT_BAT_TEMP;
		}
		else if(machine_error_hold.userErr_hold.bits.gNtcMos || machine_error_hold.userErr_hold.bits.gMos_overload)
		{
			user_led_control.LedRunCnt_Once=LED_FAULT_MOS_TEMP;
		}
		else if(machine_error_hold.MC_error_hold.bits.gBlock || machine_error_hold.MC_error_hold.bits.gLosStep)
		{
			user_led_control.LedRunCnt_Once=LED_FAULT_BLOCK;
		}
		else if(machine_error_hold.userErr_hold.bits.gIBusPeakOCP1 ||
		        machine_error_hold.userErr_hold.bits.gIBusAvgOCP1 ||
		        machine_error_hold.userErr_hold.bits.gIBusAvgOCP2 ||
		        machine_error_hold.userErr_hold.bits.gIBusAvgOCP3)
		{
			user_led_control.LedRunCnt_Once=LED_FAULT_IBUS_OCP1;
		}
		else if(machine_error_hold.userErr_hold.bits.gVBusStbUVP ||
		        machine_error_hold.userErr_hold.bits.gVBusRunUVP1 ||
		        machine_error_hold.userErr_hold.bits.gVBusRunUVP2)
		{
			user_led_control.LedRunCnt_Once=LED_FAULT_UVP;
		}
		else if(machine_error_hold.userErr_hold.bits.fly_machine)
		{
			user_led_control.LedRunCnt_Once=LED_FAULT_FLY_MACHINE;
		}
		else if(machine_error_hold.userErr_hold.bits.gVBusOVP)
		{
			user_led_control.LedRunCnt_Once=LED_FAULT_OVP;
		}
		else if(machine_error_hold.userErr_hold.bits.gMosBridgeH || machine_error_hold.userErr_hold.bits.gMosBridgeL)
		{
			user_led_control.LedRunCnt_Once=LED_FAULT_MOS_BRIDGE;
		}
		else if(machine_error_hold.MC_error_hold.bits.gLackPhase)
		{
			user_led_control.LedRunCnt_Once=LED_FAULT_LACK_PHASE;
		}
		else
		{
			user_led_control.LedRunCnt_Once=LED_FAULT_OTHER;
		}
	}
	return user_led_control.LedRunCnt_Once ? 1 : 0;
}

/** @brief 继承旧闪码节奏：灭 500ms、亮 200ms；每轮间隔 1500ms。 */
void LedError_Run(void)
{
	static u16 flash_count=0;
	static u16 flash_time=0;

	if(user_led_control.LedDeal)
	{
		LED_ERROR_OFF();
		flash_count=user_led_control.LedRunCnt_Once;
		flash_time=0;
		user_led_control.LedDeal=0;
	}
	if(!user_led_control.LedRunTimes)
	{
		LED_ERROR_OFF();
		flash_count=0;
		flash_time=0;
		return;
	}
	if(flash_count)
	{
		flash_time++;
		if(flash_time<=500)
		{
			LED_ERROR_OFF();
		}
		else if(flash_time<=700)
		{
			LED_ERROR_ON();
		}
		else
		{
			LED_ERROR_OFF();
			flash_time=0;
			flash_count--;
		}
	}
	else
	{
		LED_ERROR_OFF();
		if(++flash_time>=1500)
		{
			flash_time=0;
			flash_count=user_led_control.LedRunCnt_Once;
			user_led_control.LedRunTimes--;
		}
	}
}
