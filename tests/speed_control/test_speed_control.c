/* Included after production functions by run_tests.py; no motor model or MMIO. */
static u32 checks;
static u32 cases;
static u32 failure_line;

#define CHECK(condition) do { ++checks; if(!(condition)) { failure_line=__LINE__; return 0; } } while(0)

static __asm int semihost(int operation, const void *argument)
{
	BKPT 0xAB
	BX lr
}

static void print_number(u32 value)
{
	char buffer[12];
	u32 pos=sizeof(buffer)-1;
	buffer[pos]=0;
	do
	{
		buffer[--pos]=(char)('0'+value%10u);
		value/=10u;
	}while(value);
	semihost(4, &buffer[pos]);
}

static void select_gear(u8 gear)
{
	user_list.gears_level=gear;
	pwm_duty_aim_deal();
	user_gears_handle();
}

static void sample_speed(u32 speed)
{
	mc_core_list.time_count_60_degree=75000000u/speed;
	motor_speed_control_1ms();
}

static void prepare(u8 gear, u16 actual, u32 speed)
{
	memset(&motor_control_list, 0, sizeof(motor_control_list));
	memset(&mc_core_list, 0, sizeof(mc_core_list));
	memset(&user_list, 0, sizeof(user_list));
	memset(&machine_error_hold, 0, sizeof(machine_error_hold));
	Speed_PI_init();
	motor_speed_control_1ms(); /* Reset function-static transition history. */
	user_list.flag.bits.gToolEn=1;
	user_list.Vcc_cal_gain=32768u;
	select_gear(gear);
	motor_control_list.bldc_state=MOTOR_RUN;
	motor_control_list.u8Calibration_OK_flag=1;
	motor_control_list.startup_complete=1;
	motor_control_list.pwm_duty_virtual=motor_control_list.pwm_duty_aim;
	mc_core_list.pwm_actual_runing_data=actual;
	mc_core_list.bldc_bemf_detected=1;
	mc_core_list.CurrentOffset_ad_data=2048;
	mc_core_list.Board_ADC_DATA.u16I_SUM_Avg=2048;
	mc_core_list.Board_ADC_DATA.u16I_Peak=2048;
	sample_speed(speed);
}

static int test_gear_targets_and_reference(void)
{
	static const u32 targets[6]={20000, 28000, 36000, 44000, 52000, 60000};
	static const u16 ceilings[6]={937, 1124, 1311, 1499, 1686, 1874};
	u32 tick;
	u32 previous;
	u32 step;
	u8 gear;
	u8 tapered;
	++cases;
	for(gear=0;gear<6;++gear)
	{
		prepare(gear, 600, 10000);
		CHECK(user_list.speed_ref_erpm==targets[gear]);
		CHECK(motor_control_list.pwm_duty_aim==ceilings[gear]);
		CHECK(Speed_Pid.output==(600<<15));
		CHECK(motor_control_list.pwm_duty_aim2==600);
		CHECK(Speed_Pid.refValue==10000);
		tapered=0;
		for(tick=0;tick<2000;++tick)
		{
			previous=(u32)Speed_Pid.refValue;
			sample_speed(10000);
			CHECK(Speed_Pid.refValue>=(s32)previous);
			CHECK(Speed_Pid.refValue<=(s32)targets[gear]);
			step=(u32)Speed_Pid.refValue-previous;
			CHECK(step<=ROUTER_SPEED_REF_STEP_ERPM);
			if(step>0 && step<ROUTER_SPEED_REF_STEP_ERPM)
			{
				tapered=1;
			}
			CHECK(motor_control_list.pwm_duty_aim2<=600+ROUTER_SPEED_PWM_LEAD_COUNTS);
			CHECK(Speed_Pid.output<=(Speed_Pid.Limit_max_data<<15));
		}
		CHECK(tapered);
		CHECK(Speed_Pid.refValue==(s32)targets[gear]);
	}
	return 1;
}

static int test_all_gear_changes(void)
{
	u8 from;
	u8 to;
	u32 tick;
	u32 previous;
	++cases;
	for(from=0;from<6;++from)
	{
		for(to=0;to<6;++to)
		{
			prepare(from, 900, 20000u+8000u*from);
			select_gear(to);
			/* Deliberately leave the previous gear's virtual PWM in place. */
			previous=(u32)Speed_Pid.refValue;
			for(tick=0;tick<1500;++tick)
			{
				sample_speed(20000u+8000u*from);
				CHECK(Speed_Pid.Limit_max_data<=motor_control_list.pwm_duty_aim);
				CHECK(Speed_Pid.Limit_max_data<=motor_control_list.pwm_duty_virtual);
				CHECK(motor_control_list.pwm_duty_aim2<=900+ROUTER_SPEED_PWM_LEAD_COUNTS);
				CHECK(Speed_Pid.output<=(Speed_Pid.Limit_max_data<<15));
				if(from>to)
				{
					CHECK(Speed_Pid.refValue==(s32)user_list.speed_ref_erpm);
				}
				else
				{
					CHECK((u32)Speed_Pid.refValue>=previous);
					CHECK((u32)Speed_Pid.refValue-previous<=ROUTER_SPEED_REF_STEP_ERPM);
				}
				previous=(u32)Speed_Pid.refValue;
			}
			CHECK(Speed_Pid.refValue==(s32)user_list.speed_ref_erpm);
		}
	}
	/* A high-output downshift must clamp internal Q15 state in the first ms. */
	prepare(5, 1400, 60000);
	select_gear(0);
	sample_speed(60000);
	CHECK(Speed_Pid.Limit_max_data==motor_control_list.pwm_duty_aim);
	CHECK(Speed_Pid.output<=(motor_control_list.pwm_duty_aim<<15));
	return 1;
}

static int test_acceleration_lead_and_steady_state(void)
{
	u32 tick;
	s32 lead;
	u8 anticipates=0;
	++cases;
	prepare(0, 600, 20000);
	/* Establish a lower speed, then accelerate towards the unchanged final target. */
	for(tick=0;tick<200;++tick)
	{
		sample_speed(16000);
	}
	for(tick=0;tick<200;++tick)
	{
		sample_speed(16000+tick*20);
		if((202u+tick)%ROUTER_SPEED_PI_PERIOD_MS==0 && Speed_Pid.error!=0)
		{
			lead=Speed_Pid.refValue-Speed_Pid.error-(s32)motor_control_list.motor_speed;
			CHECK(lead>=0);
			CHECK(lead<=ROUTER_SPEED_LEAD_MAX_ERPM);
			if(motor_control_list.motor_speed<20000 && Speed_Pid.error<0)
			{
				anticipates=1;
			}
		}
	}
	CHECK(anticipates);
	for(tick=0;tick<500;++tick)
	{
		sample_speed(20000);
	}
	CHECK(Speed_Pid.error==0);
	CHECK(motor_control_list.motor_speed>=19999 && motor_control_list.motor_speed<=20001);
	/* Constant measured speed / small period quantization must not cause drift. */
	prepare(0, 600, 20000);
	for(tick=0;tick<1000;++tick)
	{
		mc_core_list.time_count_60_degree=(tick&1u)?3749:3751;
		motor_speed_control_1ms();
	}
	CHECK(Speed_Pid.error==0);
	CHECK(motor_control_list.pwm_duty_aim2==600);
	CHECK(Speed_Pid.output==(600<<15));
	return 1;
}

static int test_tracking_limit_and_fractional_integration(void)
{
	u32 tick;
	s32 reference;
	++cases;
	prepare(5, 600, 10000);
	for(tick=0;tick<1000;++tick)
	{
		sample_speed(10000);
	}
	/* The old high PI demand must not survive a sudden reduction in actual PWM. */
	mc_core_list.pwm_actual_runing_data=400;
	sample_speed(10000);
	CHECK(Speed_Pid.Limit_max_data==400+ROUTER_SPEED_PWM_LEAD_COUNTS);
	CHECK(Speed_Pid.output<=((400+ROUTER_SPEED_PWM_LEAD_COUNTS)<<15));
	reference=Speed_Pid.refValue;
	motor_control_list.startup_current_limited=1;
	for(tick=0;tick<100;++tick)
	{
		sample_speed(10000);
		CHECK(Speed_Pid.Limit_max_data==400);
		CHECK(Speed_Pid.refValue==reference);
		CHECK(Speed_Pid.output<=(400<<15));
	}
	motor_control_list.startup_current_limited=0;
	sample_speed(10000);
	CHECK(Speed_Pid.refValue>reference);
	/* A small >deadband error must still accumulate sub-CCR Q15 corrections. */
	prepare(0, 600, 10000);
	user_list.speed_ref_erpm=10200;
	for(tick=0;tick<1000;++tick)
	{
		sample_speed(10000);
	}
	CHECK(Speed_Pid.output>(600<<15));
	CHECK(motor_control_list.pwm_duty_aim2>600);
	CHECK(motor_control_list.pwm_duty_aim2<=600+ROUTER_SPEED_PWM_LEAD_COUNTS);
	return 1;
}

static int test_invalid_feedback_and_restart(void)
{
	u8 fault;
	++cases;
	for(fault=0;fault<8;++fault)
	{
		prepare(5, 600, 60000);
		switch(fault)
		{
			case 0: user_list.flag.bits.gToolEn=0; break;
			case 1: motor_control_list.bldc_state=MOTOR_STOP; break;
			case 2: mc_core_list.bldc_bemf_detected=0; break;
			case 3: mc_core_list.time_count_60_degree=0; break;
			case 4: mc_core_list.time_count_60_degree=SectorTIme_Max_TIMER_COUNT+1; break;
			case 5: user_list.userErr.word=1; break;
			case 6: mc_core_list.MC_error.word=1; break;
			default: machine_error_hold.userErr_hold.word=1; machine_error_hold.MC_error_hold.word=1; break;
		}
		motor_speed_control_1ms();
		CHECK(Speed_Pid.output==0);
		CHECK(Speed_Pid.error1==0);
		CHECK(motor_control_list.time_count_60_degree_filter_q8==0);
		CHECK(motor_control_list.motor_speed==0);
		user_list.flag.bits.gToolEn=1;
		motor_control_list.bldc_state=MOTOR_RUN;
		mc_core_list.bldc_bemf_detected=1;
		user_list.userErr.word=0;
		mc_core_list.MC_error.word=0;
		machine_error_hold.userErr_hold.word=0;
		machine_error_hold.MC_error_hold.word=0;
		mc_core_list.pwm_actual_runing_data=400;
		sample_speed(10000);
		CHECK(Speed_Pid.refValue==10000);
		CHECK(Speed_Pid.output==(400<<15));
		CHECK(motor_control_list.motor_speed==10000);
	}
	return 1;
}

static int test_pwm_ramp_and_fast_current_limit(void)
{
	u32 tick;
	u32 previous;
	++cases;
	prepare(5, 0, 10000);
	motor_control_list.pwm_duty_virtual=0;
	pwm_set(); /* Also reset pwm_set's static fractional tick counters. */
	motor_control_list.pwm_duty_aim2=1000;
	for(tick=0;tick<100;++tick)
	{
		previous=mc_core_list.pwm_actual_runing_data;
		for(u32 irq=0;irq<SLOWLOOP_1ms_CNT_LOAD;++irq)
		{
			pwm_set();
		}
		CHECK(mc_core_list.pwm_actual_runing_data<=previous+PWM_TRACK_VALUE);
		CHECK(mc_core_list.pwm_actual_runing_data>=LIMIT_MIN_PWN_DUTY);
		CHECK(pwm_last==mc_core_list.pwm_actual_runing_data);
	}
	mc_core_list.pwm_actual_runing_data=700;
	motor_control_list.pwm_duty_virtual=900;
	motor_control_list.pwm_duty_aim2=700;
	mc_core_list.Board_ADC_DATA.u16I_Peak=2048+LIMIT_PEAK_CURRENT_AdcValue+2;
	pwm_set();
	CHECK(mc_core_list.pwm_actual_runing_data==700-PWM_SUB_FAST_VALUE);
	CHECK(motor_control_list.pwm_duty_virtual==700);
	CHECK(pwm_last==mc_core_list.pwm_actual_runing_data);
	return 1;
}

int main(void)
{
	int passed=test_gear_targets_and_reference() && test_all_gear_changes() &&
		test_acceleration_lead_and_steady_state() &&
		test_tracking_limit_and_fractional_integration() &&
		test_invalid_feedback_and_restart() && test_pwm_ramp_and_fast_current_limit();
	semihost(4, passed?"PASS: speed control, ":"FAIL: speed control, ");
	print_number(cases);
	semihost(4, " groups, ");
	print_number(checks);
	semihost(4, " checks; failure line ");
	print_number(failure_line);
	semihost(4, " (real Cortex-M0 C; PWM hardware stubbed, not a bench test)\n");
	semihost(0x18, (const void *)(passed?0x20026:0x20023));
	for(;;) {}
}
