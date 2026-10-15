/**
 * @file     main.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    V366 修边机初始化与 1ms 应用任务，保留新框架电机/限速内核。
 *
 * @attention
 *
 * THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 * CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 * TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 * HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 * CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 *
 * <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
 */

#define _MAIN_C_

#include "main.h"
#include "drv_inc.h"
#include "board.h"
#include "user_control.h"
#include "drv_adc.h"
#include "drv_pwm.h"
#include "parameter.h"
#include "mc_core.h"
#include "systick.h"
#include "motor_control.h"
#include "flash_data_save.h"
#include "led.h"
#include "Bat_com.h"

/** @brief 翻转调试 GPIO；不能在正常运行时占用 PA2 母线电压输入。 */
void GPIO_Toggle(GPIO_TypeDef *GPIOn, uint16_t PINn)
{
	if(Bit_RESET==GPIO_ReadOutputDataBit(GPIOn, PINn))
	{
		GPIO_SetBits(GPIOn, PINn);
	}
	else
	{
		GPIO_ResetBits(GPIOn, PINn);
	}
}

/** @brief 示波器调试端口；默认不调用，PA2 正常用于 VBus ADC。 */
void io_debug_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA_Msk, ENABLE);
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_High;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

int main(void)
{
	u8 i;

	Systick_Init(0x00ffffff);
	/* 防飞车电路有上电瞬态，必须保留旧工程的早期检测顺序。
	 * 不能挪到 100ms 校准延时之后，也不能先将 PA13/EN 置高。
	 */
	systick_delay_us(500);
	Bsp_Adc_Init();
	Board_ADC_Init();
	power_on_fly_machine_handle();
	systick_delay_us(10000);

	Bsp_Gpio_Init();
	Peripheral_Init();
	user_para_init(); // 只清实时状态，不清刚检测到的上电故障锁存。
	systick_delay_us(100000);
	Vcc_5V_CAL();
	CurrentOffsetCalibration();
	mose_check();
	systick_delay_us(1000);
	/* 保留旧存储兼容；物理旋钮档位随后由 VSP 重建，不新增运行 Flash 写入。 */
	read_user_data();
	mc_init();

	En_Port_init();
	GPIO_SetBits(EN_PORT, EN_PIN); // PA13 仅保持驱动供电，不代表允许电机启动。
#if LED_EN
	led_init(); // PA10/COM3 单灯，不恢复旧复用矩阵。
#endif
#if BAT_COM_EN
	Bsp_Com_init();
	memset(&BatData, 0, sizeof(BATDATA));
	Bat_Com_Restart();
#endif

	Interrupt_Init();
	/* V366 ADC 独立采样，不能再依赖旧 LED 扫描的温度/电压采样窗口。 */
	user_list.led_reuse_adc_flag=0;
	user_list.led_scan_start_flag=0;
	systick_delay_us(5000);

	/* 与旧工程一致，上电用稳定 ADC 快速建立开关状态；不是运行消抖时基。 */
	for(i=0;i<20;i++)
	{
		user_direction_handle();
		motor_on_off_control();
	}
	Volt_Handler_fast();
	Volt_Handler();
	Volt_Handler();
	for(i=0;i<10;i++)
	{
		User_Temperature_Handler_Fast();
		/* 档位确认在下面的真实1ms任务完成，快速循环不能当成10ms消抖。 */
	}
	/* ADC已稳定，先按减档边界覆盖旧Flash档位，再建立本次启动的转速/PWM目标。 */
	user_gears_init(user_list.vsp_adc);
	pwm_duty_aim_deal();
	user_gears_handle(); // 初始化也使用固定档位目标，不能以未缩放的满量程目标接入PI。
	error_hold_updata();
#if LIMIT_SPEED_EN
	Speed_PI_init();
	Speed_PI_reset();
	Speed_Pid.refValue=user_list.speed_ref_erpm;
	motor_control_list.pwm_duty_aim2=motor_control_list.pwm_duty_aim;
#endif
	/* 恢复旧主入口的看门狗初始化，校准及启动检测结束后才进入喂狗循环。 */
	Drv_Iwdg_Init();

	while(1)
	{
		IWDG_RELOAD_COUNT();
#if BAT_COM_EN
		Bat_Com();
#endif
		if(user_list.u8_1ms_flag)
		{
			user_list.u8_1ms_flag=0;
			user_direction_handle();
			Volt_Handler();
			motor_on_off_control();
			user_oc_handle();
			User_Temperature_Handler();
			/* ADC每1ms只判档一次；同一档内ADC变化不得直接影响PWM目标。 */
			get_gear_from_adc(user_list.vsp_adc);
			pwm_duty_aim_deal(); // 重建固定基准，不再做ADC连续调速。
			user_gears_handle(); // 按确认档位选择六档目标，不在上次结果上重复乘比例。

			/* 先锁存本次保护结果，同一拍撤销许可；故障恢复必须释放机械开关。 */
			error_hold_updata();
			user_state_control();
			error_hold_updata();

#if LIMIT_SPEED_EN
			motor_speed_control_1ms(); // 独立处理软启动参考和小步速度PI，原档位ADC不变。
#endif
#if LED_EN
			UO_Led_Handler();
#endif
			/* 工具开关、电池许可、实时故障与锁存故障共同门控，不直接写 CCR。 */
			if(user_list.flag.bits.gToolEn && Bat_Com_CanRun() &&
			   user_list.userErr.word==0 && mc_core_list.MC_error.word==0 &&
			   machine_error_hold.userErr_hold.word==0 && machine_error_hold.MC_error_hold.word==0)
			{
				mc_core_list.motor_en=1;
			}
			else
			{
				mc_core_list.motor_en=0;
			}

			/* IPD 检测与运行使用同一套许可，取消启动时丢弃旧检测请求。 */
			if(mc_core_list.ipd_star_flag==1)
			{
				if(mc_core_list.motor_en && Bat_Com_CanRun() &&
				   motor_control_list.bldc_state==MOTOR_POSITION &&
				   user_list.userErr.word==0 && mc_core_list.MC_error.word==0 &&
				   machine_error_hold.userErr_hold.word==0 && machine_error_hold.MC_error_hold.word==0)
				{
					ipd_detect();
				}
				mc_core_list.ipd_star_flag=0;
			}
		}
	}
}
