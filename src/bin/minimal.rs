#![deny(unsafe_code)]
#![no_main]
#![no_std]


// Mapping
// list of interrupts: https://docs.rs/stm32f1/latest/stm32f1/stm32f103/enum.Interrupt.html
// TIM1 pid loop timer (controlling the plant)
// TIM2 pwm timer (motor direction and rate)
// TIM3 monotonmic clock timer
// TIM4 qeimanager (angle encoder)

// help from  henrik_alser at https://github.com/kalkyl/f103-rtic/blob/main/src/mono.rs

use defmt_rtt as _; // transport layer for defmt logs

use syd as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device =  stm32f1xx_hal::pac, peripherals = true, dispatchers = [USART1])]
mod app {

    use stm32f1xx_hal::pwm::PwmChannel;

//    use dwt_systick_monotonic::{DwtSystick, ExtU32};
    use syd::mono::{ExtU32, MonoTimer};

    use stm32f1xx_hal::{
        gpio::gpiob::{PB6, PB7},
        gpio::gpioc::PC13,
        gpio::{gpioa::PA3, Floating, Input, PullUp},
        gpio::{
            gpioa::{
//                PA0, PA1,
                PA2},
//            Alternate,
            Output, PushPull,
        },
        prelude::*,
        qei::{Qei, QeiOptions},
        stm32,
        pac,
//        time::MonoTimer,
        timer::{CountDownTimer, Event, Tim2NoRemap, Tim4NoRemap, Timer},
    };

    // wrapper around quadratue encoder interface so it becomes i64 in stead of u32
    use qei::QeiManager;

    const PID_freq: u32 = 800;

    #[monotonic(binds = TIM3, default = true)]
    type MyMono = MonoTimer<pac::TIM3, 1_000>;

    // #[monotonic(binds = SysTick, default = true)]
    // type MonoTimer = DwtSystick<48_000_000>; // 48 MHz
    //type MonoTimer = DwtSystick<10_000>;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        tracking: bool, // is system in tracking state? if not: retracting
        setpoint: i64,  // target position for digital servo
        motor_pos:
            QeiManager<Qei<stm32::TIM4, Tim4NoRemap, (PB6<Input<Floating>>, PB7<Input<Floating>>)>>,
    }

    // Local resources go here
    #[local]
    struct Local {
        onboard_led: PC13<Output<PushPull>>,
        signal_led: PA2<Output<PushPull>>,
        rate_switch: PA3<Input<PullUp>>,

        tmr1: CountDownTimer<stm32::TIM1>, // pid loop timer
        error_prior: f64,
        integral_prior: f64,
        epoch: f64,
        last_position: i64,
        t_last_position_change: f64,

        pwm_backward: PwmChannel<stm32f1xx_hal::pac::TIM2, stm32f1xx_hal::pwm::C1>,
        pwm_forward: PwmChannel<stm32f1xx_hal::pac::TIM2, stm32f1xx_hal::pwm::C2>,
        pwm_max_duty: u16,
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

        // Set up the LED
        let mut onboard_led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        onboard_led.set_high(); // not sure why unwrap is not implemented.

        // set up signal LED on PA2
        let mut signal_led = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
        signal_led.set_low();

        // set up system timer
        let mono = MyMono::new(cx.device.TIM3, &clocks);

        //let mono = DwtSystick::new(&mut cx.core.DCB, cx.core.DWT, cx.core.SYST, clocks.hclk().0);
        //let mono = DwtSystick::new(&mut cx.core.DCB, cx.core.DWT, cx.core.SYST, 10_000u32);

        //------------
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


        let max_duty = pwm.get_max_duty();

        let mut pwm_channels = pwm.split();

        pwm_channels.0.enable();
        pwm_channels.1.enable();

        let pwm_backward = pwm_channels.0;
        let pwm_forward = pwm_channels.1;

        //------------
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

        let mut motor_pos = QeiManager::new(qei);

        //------------
        // tracking rate selection control switch input Pullup.
        let rate_switch = gpioa.pa3.into_pull_up_input(&mut gpioa.crl);

        //------------
        // Use TIM1 for the PID controller task
        let mut tmr1 = Timer::tim1(cx.device.TIM1, &clocks).start_count_down(PID_freq.hz());
        tmr1.listen(Event::Update);

        //------------
        // kick off the threads
        main::spawn().ok();
        pid_update::spawn().ok();

        // return the resources (and the monotonic timer)
        (
            Shared {
                // Initialization of shared resources go here
                tracking: false,
                setpoint: -i64::MAX, // initial setpoint minus infinite
                motor_pos,
            },
            Local {
                // Initialization of local resources go here
                onboard_led,
                signal_led,
                tmr1, // PID loop
                rate_switch,
                integral_prior: 0.0,
                error_prior: 0.0,
                epoch: 0.0,
                last_position: 0,
                t_last_position_change: 0.0,
                pwm_backward: pwm_backward,
                pwm_forward: pwm_forward,
                pwm_max_duty: max_duty,
            },
            init::Monotonics(mono), // give the monotonic to RTIC
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
    #[task(
        shared=[tracking, motor_pos, setpoint],
        local=[epoch, signal_led, last_position, t_last_position_change],
    )]
    fn main(cx: main::Context) {

        let mut tracking = cx.shared.tracking;
        let mut motor_pos = cx.shared.motor_pos;
        let mut epoch = cx.local.epoch;
        let mut last_position = cx.local.last_position;
        let mut t_last_position_change = cx.local.t_last_position_change;
        let mut setpoint = cx.shared.setpoint;

        let signal_led = cx.local.signal_led;
        signal_led.toggle();

        let t = as_ms(monotonics::now().duration_since_epoch()) as f64 /1000.0;

        // 1.08 deler compenseert voor foute klok?
        const TRACKING_SPEED_mARCS_PER_S: [f64; 3] = [
            15000./1.08,
            14685./1.08,
            15041./1.08 ];

        // compute angle setpoint in milli arcseconds (tricky, rounding errors galore)
        let angle_setpoint: f64 =
            TRACKING_SPEED_mARCS_PER_S[0] * (t - *epoch);

        // mount geometry specific coeficients

        // convert angle in  milli-arcseconds to encoder count

        //	angle*(angle*(angle * 25.125E-21 - 5.7232E-12) + 3.2282E-3) + 24.73
        //	angle*(angle*(angle * 26.529E-21 - 5.6183E-12) + 3.4131E-3) + 18.1
        //	angle*(angle*(angle * 23.95E-21 - 5.839E-12) + 3.063E-3) + 33.8
        //    angle*(angle*(angle * 22.9E-21 - 5.967E-12) + 2.917E-3) + 46.0

        let target_count = (6.4 +
            angle_setpoint  *
            ( 2.8938e-3 + angle_setpoint *
              ( 382.41e-15 + angle_setpoint *
                ( - 20.517e-21 + angle_setpoint * 268.44e-30  )
              )
            )) as i64;

        (tracking, motor_pos, setpoint)
            .lock(|tracking, motor_pos, setpoint| {
                // get position of motor
                motor_pos.sample().unwrap();
                let position = motor_pos.count();

                // check if motor is stalled; assume we are always moving at second scale
                // if stalled, decide what to do next

                if position != *last_position {
                    *t_last_position_change = t;
                }

                *last_position = position;

                defmt::println!("*reported time since epoch {}", t);
                defmt::println!("last t_last_position_change {}", *t_last_position_change);
                defmt::println!("dt {}", (t-*t_last_position_change));
                defmt::println!("setpoint {}", *setpoint);
                defmt::println!("position {}", *last_position);
                if (t - *t_last_position_change) > 5.0 {
                    // more than x seconds passed since last position change:
                    // stalled
                    defmt::println!("stalled");
                    if *tracking {
                        // stalled and tracking: have to retract
                        *setpoint = -i64::MAX;
                        *tracking = false;
                    } else {
                        // stalled and not tracking: we just retracted: start tracking
                        *tracking = true;
                        *epoch = t; // start time of this run
                        motor_pos.reset(); // set count to 0 at starting point
                        *t_last_position_change = t; // ensure we don't run into this again immediately
                    }
                }
                if *tracking {

                    *setpoint = target_count+3000;
                }
            });
        main::spawn_after(ExtU32::millis(5u32)).ok();
    }


    // update the PID loop; skeleton
    // with help from henrik_alser and adamgreig
    #[task(
        shared = [setpoint,  // setpoint position
                  motor_pos, // qei encoder
        ],
        local = [onboard_led,
                 integral_prior, error_prior, // PID parameters
                 pwm_backward, pwm_forward, // pwm channels
                 pwm_max_duty, // constant
        ]
    )]
    fn pid_update(cx: pid_update::Context) {

        let onboard_led = cx.local.onboard_led;
        // defmt::println!("pid_update");

        let setpoint = cx.shared.setpoint;
        let motor_pos = cx.shared.motor_pos;

        let integral_prior = cx.local.integral_prior;
        let error_prior = cx.local.error_prior;

        let pwm_forward = cx.local.pwm_forward;
        let pwm_backward = cx.local.pwm_backward;
        let pwm_max_duty = cx.local.pwm_max_duty;

        let Kp: f64 = 256.0;
        let Ki: f64 = 0.000;
        let Kd: f64 = 128.0;
        let bias: f64 = *pwm_max_duty as f64 / 100.0;

        (setpoint, motor_pos).lock(|setpoint, motor_pos| {
            onboard_led.toggle();

            // get position of motor
            motor_pos.sample().unwrap();
            let position = motor_pos.count();
            let position_error: f64 = *setpoint as f64 - position as f64;

            // pid control
            let integral = *integral_prior + position_error as f64 / 1000.;
            let derivative = (position_error - *error_prior) * 1000.;
            let mut pwm_setting = Kp * position_error + Ki * integral + Kd * derivative + bias;

            *integral_prior = integral;
            *error_prior = position_error;

            if abs(pwm_setting) > (*pwm_max_duty).into() {
                pwm_setting = *pwm_max_duty as f64 * sign(pwm_setting);
            };

            // set pwm
            if pwm_setting > 0.0 {
                pwm_forward.set_duty(pwm_setting as u16);
                pwm_backward.set_duty(0);
            } else {
                pwm_forward.set_duty(0);
                pwm_backward.set_duty(-pwm_setting as u16);
            };
        });
        pid_update::spawn_after(1.millis()).ok();
    }

    // fn as_s (dt: dwt_systick_monotonic::fugit::Instant<u32, 1, 10_000>) -> i64 {
    //     (dt.ticks()/48_000_000) as i64
    // }

    fn as_s<const NOM: u32, const DENOM: u32>(
        d: dwt_systick_monotonic::fugit::Duration<u32, NOM, DENOM>
    ) -> i64
    {
        let secs: dwt_systick_monotonic::fugit::SecsDurationU32 = d.convert();
        secs.ticks() as i64
    }
    fn as_ms<const NOM: u32, const DENOM: u32>(
        d: dwt_systick_monotonic::fugit::Duration<u32, NOM, DENOM>
    ) -> i64
    {
        let millis: dwt_systick_monotonic::fugit::MillisDurationU32 = d.convert();
        millis.ticks() as i64
    }

    fn sign(f: f64) -> f64 {
        if f < 0.0 { -1.0 } else { 1.0 }
    }

    fn abs(f: f64) -> f64 {
        f * sign(f)
    }

}
