/* Production gear and LED functions plus reference_led_error_control are provided by run_tests.py. */
volatile u32 test_checks;
volatile u32 test_failure_line;
volatile u32 test_case;

#define CHECK(value) do { ++test_checks; if(!(value)) { test_failure_line=__LINE__; return; } } while(0)

static void reset_led(void)
{
	user_led_control.LedDeal=0;
	user_led_control.LedRunTimes=0;
	user_led_control.LedRunCnt_Once=0;
	reference_led_control=user_led_control;
}

void run_tests(void)
{
	static const u16 expected_down[5]={82, 901, 1720, 2498, 3522};
	u32 adc;
	u32 old_gear;
	u32 expected;
	u32 tick;
	u32 a;
	u32 b;
	u32 flags;
	u16 core_flags;
	u8 result;
	u8 reference;

	test_checks=0;
	test_failure_line=0;
	test_case=1;
	/* Every ADC code, independent of all six previous/Flash gear values. */
	for(adc=0;adc<=ADC_REF_VALUE;++adc)
	{
		expected=0;
		for(tick=0;tick<5;++tick)
		{
			if(adc>=expected_down[tick])
			{
				expected++;
			}
		}
		for(old_gear=0;old_gear<VSP_GEAR_COUNT;++old_gear)
		{
			user_list.gears_level=old_gear;
			user_gears_init(adc);
			CHECK(user_list.gears_level==expected);
			/* Keeping the knob still after boot must not drift to another gear. */
			for(tick=0;tick<VSP_GEAR_CONFIRM_MS+1;++tick)
			{
				get_gear_from_adc(adc);
				CHECK(user_list.gears_level==expected);
			}
		}
	}

	test_case=2;
	user_gears_init(0);
	for(tick=0;tick<VSP_GEAR_CONFIRM_MS-1;++tick)
	{
		get_gear_from_adc(VSP_GEAR_MV_TO_ADC(1500));
		CHECK(user_list.gears_level==0);
	}
	get_gear_from_adc(VSP_GEAR_MV_TO_ADC(1500));
	CHECK(user_list.gears_level==1);
	/* A sample in the hold region cancels an unfinished downshift. */
	for(tick=0;tick<VSP_GEAR_CONFIRM_MS-1;++tick)
	{
		get_gear_from_adc(0);
	}
	get_gear_from_adc(VSP_GEAR_MV_TO_ADC(1500));
	for(tick=0;tick<VSP_GEAR_CONFIRM_MS-1;++tick)
	{
		get_gear_from_adc(0);
		CHECK(user_list.gears_level==1);
	}
	get_gear_from_adc(0);
	CHECK(user_list.gears_level==0);
	/* Reinitialization must also clear pending debounce history. */
	for(tick=0;tick<VSP_GEAR_CONFIRM_MS-1;++tick)
	{
		get_gear_from_adc(VSP_GEAR_MV_TO_ADC(1500));
	}
	user_gears_init(0);
	get_gear_from_adc(VSP_GEAR_MV_TO_ADC(1500));
	CHECK(user_list.gears_level==0);

	test_case=3;
	/* Compare single faults and all pairs, including priority and reference 11-flash fallback. */
	for(a=0;a<25;++a)
	{
		for(b=0;b<25;++b)
		{
			machine_error_hold.userErr_hold.word=0;
			machine_error_hold.MC_error_hold.word=0;
			if(a<20) machine_error_hold.userErr_hold.word|=1u<<a;
			else machine_error_hold.MC_error_hold.word|=1u<<(a-20);
			if(b<20) machine_error_hold.userErr_hold.word|=1u<<b;
			else machine_error_hold.MC_error_hold.word|=1u<<(b-20);
			flags=machine_error_hold.userErr_hold.word;
			core_flags=machine_error_hold.MC_error_hold.word;
			reset_led();
			reference=reference_led_error_control();
			result=LedError_Control();
			CHECK(result==reference);
			CHECK(user_led_control.LedRunCnt_Once==reference_led_control.LedRunCnt_Once);
			CHECK(user_led_control.LedRunTimes==10);
			CHECK(user_led_control.LedRunMode==1);
			CHECK(machine_error_hold.userErr_hold.word==flags);
			CHECK(machine_error_hold.MC_error_hold.word==core_flags);
			/* Disabled new faults must not mask an existing reference fault. */
			expected=reference_led_control.LedRunCnt_Once;
			machine_error_hold.userErr_hold.bits.gBatComFault=1;
			flags=machine_error_hold.userErr_hold.word;
			reset_led();
			CHECK(LedError_Control()==1);
			CHECK(user_led_control.LedRunCnt_Once==expected);
			CHECK(machine_error_hold.userErr_hold.word==flags);
		}
	}

	test_case=4;
	machine_error_hold.userErr_hold.word=0;
	machine_error_hold.MC_error_hold.word=0;
	machine_error_hold.userErr_hold.bits.gBatComFault=1;
	flags=machine_error_hold.userErr_hold.word;
	reset_led();
#if LED_FAULT_BAT_COM_EN
	CHECK(LedError_Control()==1);
	CHECK(user_led_control.LedRunCnt_Once==LED_FAULT_BAT_COM);
#else
	CHECK(LedError_Control()==0);
	CHECK(user_led_control.LedRunTimes==0);
	CHECK(user_led_control.LedRunCnt_Once==0);
#endif
	CHECK(machine_error_hold.userErr_hold.word==flags);
	machine_error_hold.userErr_hold.word=0;
	CHECK(LedError_Control()==0);
}
