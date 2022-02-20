#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Mapping
// list of interrupts: https://docs.rs/stm32f1/latest/stm32f1/stm32f103/enum.Interrupt.html
// TIM2 pwm timer (motor direction and rate)
// TIM3 monotonic clock timer
// TIM4 qeimanager (angle encoder)

// help from  henrik_alser at https://github.com/kalkyl/f103-rtic/blob/main/src/mono.rs
// star tracker servo device app

// broken out tasks in different source files
mod pid_update;
mod rate_update;

#[rtic::app(device =  stm32f1xx_hal::pac, peripherals = true, dispatchers = [USART1])]
mod app {

    // code given by henrik_alser for TIM3 based MonoTimer
    use syd::mono::{ExtU32, MonoTimer};

    use stm32f1xx_hal::{
        gpio::gpiob::{PB6, PB7}, // qei pins
        gpio::gpioc::PC13,       // onboard led
        gpio::{
            gpioa::{PA2, PA3}, // pwm control
            Floating,
            Input,
            Output,
            PullUp,
            PushPull,
        },
        pac,
        prelude::*,
        pwm::PwmChannel,
        qei::{Qei, QeiOptions},
        stm32,
        timer::{Tim2NoRemap, Tim4NoRemap, Timer},
    };

    use debouncr::{debounce_2, Debouncer, Repeat2};

    // External crate qei contains a wrapper around quadrature encoder
    // interface module in HAL so it becomes a i64 in stead of u32
    // device; The Qei from the HAL is an argument for the constructor
    // of the manager.
    use qei as qeimanager;
    use qeimanager::QeiManager;

    use crate::pid_update::pid_update_fn;
    use crate::rate_update::rate_update_fn;

    #[monotonic(binds = TIM3, default = true)]
    type MyMono = MonoTimer<pac::TIM3, 10_000>;

    // set up quadrature encoder manager
    type SomePin = PB6<Input<Floating>>;
    type OtherPin = PB7<Input<Floating>>;
    type Qei4 = Qei<stm32::TIM4, Tim4NoRemap, (SomePin, OtherPin)>;
    type QeiManager4 = qeimanager::QeiManager<Qei4>;

    #[shared]
    struct Shared {
        tracking: bool,         // system in tracking state?
        rate_index: usize,      // index in the rating speed table
        setpoint: i64,          // target position for digital servo
        motor_pos: QeiManager4, // motor position interface
    }

    // Local resources
    #[local]
    struct Local {
        onboard_led: PC13<Output<PushPull>>,
        signal_led: PA2<Output<PushPull>>,

        rate_switch: PA3<Input<PullUp>>,
        rate_switch_state: Debouncer<u8, Repeat2>,

        pwm_backward: PwmChannel<stm32f1xx_hal::pac::TIM2, stm32f1xx_hal::pwm::C1>,
        pwm_forward: PwmChannel<stm32f1xx_hal::pac::TIM2, stm32f1xx_hal::pwm::C2>,
        pwm_max_duty: u16,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::println!("init: starting up");

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let mut afio = cx.device.AFIO.constrain();

        // Acquire the GPIOC peripherals
        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split(); // quadrature counter
        let mut gpioc = cx.device.GPIOC.split();

        let clocks = rcc
            .cfgr
            //.use_hse(8.mhz()) // use internal clock since xtal on board seems to be broken.
            .sysclk(48.mhz())
            .hclk(32.mhz())
            .pclk1(36.mhz())
            .pclk2(36.mhz())
            .adcclk(16.mhz())
            .freeze(&mut flash.acr);

        // set up system timer
        let mono = MyMono::new(cx.device.TIM3, &clocks);

        // Set up the onboard LED
        let mut onboard_led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        onboard_led.set_high(); // not sure why unwrap is not implemented.

        // set up signal LED on PA2
        let mut signal_led = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
        signal_led.set_low();

        // set up PWM
        let pwm_pins = (
            gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl), // motor ch 1
            gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl), // motor ch 2
        );
        let pwm = Timer::tim2(cx.device.TIM2, &clocks).pwm::<Tim2NoRemap, _, _, _>(
            pwm_pins,
            &mut afio.mapr,
            20.khz(),
        );

        let max_duty = pwm.get_max_duty() * 4 / 5;

        let mut pwm_channels = pwm.split();
        pwm_channels.0.enable();
        pwm_channels.1.enable();

        let pwm_backward = pwm_channels.0;
        let pwm_forward = pwm_channels.1;

        // set up quadrature angle counter
        // use TIM4 as the qei counter
        // ! ensure a pull up resistor in hall sensor circuit is physically present
        // ! stm32f1xx_hal::pwm_input::Pins does not allow into_pull_up_input ?!
        let qei_pins = (
            gpiob.pb6.into_floating_input(&mut gpiob.crl),
            gpiob.pb7.into_floating_input(&mut gpiob.crl),
        );
        let qei = Timer::tim4(cx.device.TIM4, &clocks).qei(
            qei_pins,
            &mut afio.mapr,
            QeiOptions::default(),
        );
        let motor_pos = QeiManager::new(qei);

        // tracking rate selection control switch on PA4; input Pullup.
        let rate_switch = gpioa.pa3.into_pull_up_input(&mut gpioa.crl);

        // kick off the threads; they recall themselves using monotonic sw timer
        main::spawn().ok();
        pid_update_fn::spawn().ok();
        rate_update_fn::spawn().ok();

        // return the resources and the monotonic timer
        (
            Shared {
                // Initialization of shared resources
                tracking: false,
                rate_index: 0,           // start at siderial rate
                setpoint: -i64::MAX / 2, // initial setpoint (retract)
                motor_pos,
            },
            Local {
                // Initialization of local resources
                onboard_led,
                signal_led,
                rate_switch,
                rate_switch_state: debounce_2(false),
                pwm_backward,
                pwm_forward,
                pwm_max_duty: max_duty,
            },
            // Move the monotonic timer to the RTIC run-time, this enables
            // scheduling
            init::Monotonics(mono),
        )
    }

    // Background task, runs whenever no other tasks are running
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    // main loop, maintaining count setpoint and dealing with retraction
    #[task(
        shared=[
	    tracking,
	    motor_pos,
	    setpoint,
	    rate_index,
	],
        local=[
	    onboard_led,
	    epoch:f32 = 0.0,
            last_position:i64 = 0,
            t_last_position_change:f32 = 0.0,
	    rate_base_angle: f32 = 0.0,
	    rate_change_time: f32 = -1.0,
	    last_rate: usize = 100,
	    angle_setpoint: f32 = 0.0,
	],
    )]
    fn main(cx: main::Context) {
        let main::SharedResources {
            tracking,
            motor_pos,
            setpoint,
            mut rate_index,
        } = cx.shared;

        let main::LocalResources {
            onboard_led,
            epoch,
            last_position,
            t_last_position_change,
            rate_base_angle,
            rate_change_time,
            last_rate,
            angle_setpoint,
        } = cx.local;

        // time since start of timer in s
        let t = as_ms(monotonics::now().duration_since_epoch()) as f32 / 1000.0;

        rate_index.lock(|rate_index| {
            if *rate_index != *last_rate {
                *rate_base_angle = *angle_setpoint;
                *rate_change_time = t;
                *last_rate = *rate_index;
            }
        });

        // speed table
        // 1.08 divider compensates for measured clock speed bias
        const correction: f32 = 1.08;
        const TRACKING_SPEED_mARCS_PER_S: [f32; 3] = [
            15041. / correction,      // Siderial
            14685. / correction,      // Lunar
            15000. * 6. / correction, // Solar
        ];

        // compute setpoint angle (tracking speed * time since start run)
        // compute angle setpoint in milli arcseconds (approach inherited from integer arithmetic)
        *angle_setpoint = rate_index.lock(|rate_index| {
            *rate_base_angle + TRACKING_SPEED_mARCS_PER_S[*rate_index] * (t - *rate_change_time)
        });

        // mount geometry specific coeficients
        // convert angle in  milli-arcseconds to encoder setpoint
        // compute setpoint indepent of whether or not we are tracking.
        let mut target_count: i64 = (6.4
            + *angle_setpoint
                * (2.8938e-3
                    + *angle_setpoint
                        * (382.41e-15
                            + *angle_setpoint * (-20.517e-21 + *angle_setpoint * 268.44e-30))))
            as i64;

        // this lock checks the position and sets the setpoint
        // it can be made smaller but then the logic becomes unwieldy, and the code is small anyway.
        (tracking, motor_pos, setpoint).lock(|tracking, motor_pos, setpoint| {
            // get position of motor
            motor_pos.sample().unwrap();
            let position = motor_pos.count();

            // record last time that position count changed
            // switch on LED if postion changed, off otherwise
            if position != *last_position {
                *t_last_position_change = t;
                onboard_led.set_low()
            } else {
                onboard_led.set_high()
            };
            *last_position = position;

            // if motor is stalled flip tracking state
            if (t - *t_last_position_change) > 3.0 {
                defmt::println!("stalled");
                defmt::println!("tracking: {}", *tracking);

                if *tracking {
                    defmt::println!("start retracting");
                    // stalled and tracking
                    *tracking = false;
                } else {
                    defmt::println!("start tracking");
                    // stalled and not tracking
                    *tracking = true;

                    // record start time of this run and disarm stall detection
                    *epoch = t;
                    *t_last_position_change = t;
                    *rate_base_angle = 0.0;
                    *rate_change_time = t;

                    // set qei wrapper count and target count to 0
                    motor_pos.reset();
                    target_count = 0;
                }
            }

            // set target count
            if *tracking {
                *setpoint = target_count;
            } else {
                // retract
                *setpoint = -i64::MAX / 2;
            }
        }); // lock

        // about 50 ticks per second, so if we have a repeat frequency of 100 Hz we should be tracking tick by tick.
        main::spawn_after(ExtU32::millis(10u32)).ok();
    }

    extern "Rust" {
        // update the PID loop
        // with help from henrik_alser and adamgreig
        #[task(
        shared = [
	    setpoint,  // setpoint position
            motor_pos, // qei encoder
        ],
        local = [
	    respawn_delay_ms: u32  = 10,
            integral_prior:f32 = 0.0,
            error_prior:f32    = 0.0, // PID parameters
            pwm_backward, pwm_forward, // pwm channels
            pwm_max_duty, // constant
        ]
    )]
        fn pid_update_fn(cx: pid_update_fn::Context);
    }

    extern "Rust" {
        // allow user to control the angular rotation rate and control the
        // rate indication signal LED; run 10 times/s, fast enough for
        // tracking human switch control
        #[task(
            shared = [
		rate_index,  // index in rate table
            ],
            local = [rate_switch,
                     rate_switch_state,
                     signal_led,
                     decis: u32 = 0 // cycle counter for LED signal
            ]
	)]
        fn rate_update_fn(cx: rate_update_fn::Context);
    }

    //------------
    // get the length of a Duration in ms
    fn as_ms<const NOM: u32, const DENOM: u32>(
        d: dwt_systick_monotonic::fugit::Duration<u32, NOM, DENOM>,
    ) -> u32 {
        let millis: dwt_systick_monotonic::fugit::MillisDurationU32 = d.convert();
        millis.ticks()
    }
}
