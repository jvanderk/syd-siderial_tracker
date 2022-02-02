#![deny(unsafe_code)]
#![no_main]
#![no_std]

use defmt_rtt as _; // transport layer for defmt logs

use syd as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device =  stm32f1xx_hal::pac, peripherals = true, dispatchers = [USART1])]
mod app {

    //use core::marker::PhantomData;
    use stm32f1xx_hal::gpio::CRL;
    use stm32f1xx_hal::pwm::Pwm;
    use stm32f1xx_hal::pwm::PwmChannel;
    use stm32f1xx_hal::spi::Remap;

    use dwt_systick_monotonic::{DwtSystick, ExtU32};
    use fugit::*;


    use stm32f1xx_hal::{
        gpio::gpiob::{PB6, PB7},
        gpio::gpioc::PC13,
        gpio::{gpioa::PA3, Floating, Input, PullUp},
        gpio::{
            gpioa::{PA0, PA1, PA2},
            Alternate, Output, PushPull,
        },
        prelude::*,
        qei::{Qei, QeiOptions},
        stm32,
        timer::{CountDownTimer, Event, Tim2NoRemap, Tim4NoRemap, Timer},
    };

    // wrapper around quadratue encoder interface so it becomes i64 in stead of u32
    use qei::QeiManager;

    const PID_freq: u32 = 50;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<48_000_000>; // 48 MHz

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        tracking: bool, // is system in tracking state? if not: retracting
        setpoint: i64,  // target position for digital servo
        t: i64,         // discrete time in tenths of seconds
    }

    // Local resources go here
    #[local]
    struct Local {
        onboard_led: PC13<Output<PushPull>>,
        signal_led: PA2<Output<PushPull>>,
        rate_switch: PA3<Input<PullUp>>,

        tmr3: CountDownTimer<stm32::TIM3>,
        motor_pos:
            QeiManager<Qei<stm32::TIM4, Tim4NoRemap, (PB6<Input<Floating>>, PB7<Input<Floating>>)>>,
        error_prior: f64,
        integral_prior: f64,

        pwm_backward: PwmChannel<stm32f1xx_hal::pac::TIM2, stm32f1xx_hal::pwm::C1>,
        pwm_forward: PwmChannel<stm32f1xx_hal::pac::TIM2, stm32f1xx_hal::pwm::C2>,
        pwm_max_duty: u16,

        last_position: i64,
        t_last_position_change: i64,
        t_start_run: i64,
        // pwm: Pwm<stm32f1xx_hal::pac::TIM2, Tim2NoRemap,
        //          (stm32f1xx_hal::pwm::C1, stm32f1xx_hal::pwm::C2),
        //          (stm32f1xx_hal::gpio::Pin<Alternate<stm32f1xx_hal::gpio::PushPull>, CRL, 'A', 0_u8>,
        //           stm32f1xx_hal::gpio::Pin<Alternate<stm32f1xx_hal::gpio::PushPull>, CRL, 'A', 1_u8>)
        //          >,
        // Timer<stm32::TIM2<Tim2NoRemap, _, _, _>>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::println!("init");

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        // Prepare the alternate function I/O registers
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
            .adcclk(4.mhz())
            .freeze(&mut flash.acr);

        // Set up the LED.
        let mut onboard_led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        onboard_led.set_high(); // not sure why unwrap is not implemented.
        let mut signal_led = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
        signal_led.set_low();

        // set up the timer
        let mono = DwtSystick::new(&mut cx.core.DCB, cx.core.DWT, cx.core.SYST, clocks.hclk().0);

        //      let mut motor_ch_1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
        //      let mut motor_ch_2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);

        let pwm_pins = (
            gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl), // motor ch 1
            gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl), // motor ch 2
        );

        // set up PWM
        let pwm = Timer::tim2(cx.device.TIM2, &clocks).pwm::<Tim2NoRemap, _, _, _>(
            pwm_pins,
            &mut afio.mapr,
            20.khz(),
        );

        let max_duty = pwm.get_max_duty();

        let mut pwm_channels = pwm.split();

        pwm_channels.0.enable();
        pwm_channels.1.enable();

        let mut pwm_backward = pwm_channels.0;
        let mut pwm_forward = pwm_channels.1;

        // Quadrature input section
        // TIM4
        // ensure pull up resistor in hall sensor circuit is physically present
        // stm32f1xx_hal::pwm_input::Pins does not allow into_pull_up_input ?!

        let qei_pins = (
            gpiob.pb6.into_floating_input(&mut gpiob.crl),
            gpiob.pb7.into_floating_input(&mut gpiob.crl),
        );

        let qei = Timer::tim4(cx.device.TIM4, &clocks).qei(
            qei_pins,
            &mut afio.mapr,
            QeiOptions::default(),
        );

        let mut motor_pos = QeiManager::new(qei);

        // control switch input. Pullup.
        let rate_switch = gpioa.pa3.into_pull_up_input(&mut gpioa.crl);

        // app setup

        // Use TIM2 for the PID counter task
        let mut tmr3 = Timer::tim3(cx.device.TIM3, &clocks).start_count_down(PID_freq.hz());
        tmr3.listen(Event::Update);

        //let tracking: bool = false;

        main::spawn_after(ExtU32::millis(1)).ok();

        // return the resources (and the monotonic timer?)
        (
            Shared {
                // Initialization of shared resources go here
                tracking: false,
                setpoint: -i64::MAX, // initial setpoint minus infinite
                t: 0,
            },
            Local {
                // Initialization of local resources go here
                onboard_led,
                signal_led,
                tmr3,
                rate_switch,
                motor_pos,
                integral_prior: 0.0,
                error_prior: 0.0,
                pwm_backward: pwm_backward,
                pwm_forward: pwm_forward,
                pwm_max_duty: max_duty,
                last_position: 0,
                t_last_position_change: 0,
                t_start_run: 0,
            },
            init::Monotonics(mono),
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");
        loop {
            continue;
        }
    }

    // TODO: Add tasks
    // &- means that shared resource is 'not locked'
    #[task(shared=[tracking, t], local=[signal_led])]
    fn main(cx: main::Context) {
        let dt: u32 = 100;
        let mut tracking = cx.shared.tracking;

        let signal_led = cx.local.signal_led;
        signal_led.toggle();

        // this is a bad way of finding time in seconds
        defmt::println!("main! {}", monotonics::now().ticks()/48_000_000);

        (tracking).lock(|tracking| {
            ;
        });
        main::spawn_after(ExtU32::millis(dt)).ok();
    }

    // Interrupt task for TIM2, the PID counter timer
    #[task(binds = TIM3, priority = 2, local = [tmr3])]
    fn tim3(cx: tim3::Context) {
        // Delegate the state update to a software task
        pid_update::spawn().unwrap();
        // Restart the timer and clear the interrupt flag
        cx.local.tmr3.start(PID_freq.hz());
        cx.local.tmr3.clear_update_interrupt_flag();
    }

    // update the PID loop; skeleton
    // with help from henrik_alser and adamgreig
    #[task(shared = [t, setpoint, tracking],
           local = [onboard_led,
                    motor_pos, // qei object
                    last_position, t_last_position_change, // stall check data
                    t_start_run,
                    integral_prior, error_prior, // PID parameters
                    pwm_backward, pwm_forward, // pwm channels
                    pwm_max_duty, // constant
           ])]
    fn pid_update(cx: pid_update::Context) {
        let onboard_led = cx.local.onboard_led;
        let mut t = cx.shared.t;
        let mut tracking = cx.shared.tracking;
        let mut last_position = cx.local.last_position;
        let mut t_last_position_change = cx.local.t_last_position_change;
        let mut setpoint = cx.shared.setpoint;
        let mut t_start_run = cx.local.t_start_run;
        let mut motor_pos = cx.local.motor_pos;
        let mut integral_prior = cx.local.integral_prior;
        let mut error_prior = cx.local.error_prior;
        let pwm_forward = cx.local.pwm_forward;
        let pwm_backward = cx.local.pwm_backward;
        let pwm_max_duty = cx.local.pwm_max_duty;

        let Kp: f64 = 16.0;
        let Ki: f64 = 0.0;
        let Kd: f64 = 1.0;
        let bias: f64 = *pwm_max_duty as f64 / 5.0;

        (setpoint, t, tracking).lock(|setpoint, t, tracking| {
            onboard_led.toggle();

            // get position of motor
            motor_pos.sample().unwrap();
            let position = motor_pos.count();
            let position_error: f64 = (*setpoint - position) as f64;

            // check if motor is stalled; assume we are always moving at second scale
            // if stalled, decide what to do next

            if position != *last_position {
                *t_last_position_change = *t;
            }

            if (*t_last_position_change - *t) > 5 {
                // more than x seconds passed since last position change:
                // stalled
                defmt::println!("stalled");
                if *tracking {
                    // stalled and tracking: have to retract
                    *setpoint = -i64::MAX;
                    *tracking = false;
                } else {
                    // stalled and not tracking: start tracking
                    *tracking = true;
                    *t_start_run = *t; // start time of this run
                }
            }

            // pid control
            let integral = *integral_prior + position_error as f64 / PID_freq as f64;
            let derivative = (position_error - *error_prior) * PID_freq as f64;
            let mut pwm_setting = Kp * position_error + Ki * integral + Kd * derivative + bias;

            *integral_prior = integral;
            *error_prior = position_error;

            if pwm_setting > (*pwm_max_duty).into() {
                pwm_setting = (*pwm_max_duty).into();
            };

            // set pwm
            if pwm_setting > 0.0 {
                pwm_forward.set_duty(pwm_setting as u16);
                pwm_backward.set_duty(0);
            } else {
                pwm_forward.set_duty(0);
                pwm_backward.set_duty(-pwm_setting as u16);
            };
        })
    }
}
