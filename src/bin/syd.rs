#![deny(unsafe_code)]
#![no_main]
#![no_std]


// Mapping
// list of interrupts: https://docs.rs/stm32f1/latest/stm32f1/stm32f103/enum.Interrupt.html
// TIM2 pwm timer (motor direction and rate)
// TIM3 monotonic clock timer
// TIM4 qeimanager (angle encoder)

// help from  henrik_alser at https://github.com/kalkyl/f103-rtic/blob/main/src/mono.rs

use defmt_rtt as _; // transport layer for defmt logs

use syd as _; // ?

#[rtic::app(device =  stm32f1xx_hal::pac, peripherals = true, dispatchers = [USART1])]
mod app {

    use stm32f1xx_hal::pwm::PwmChannel;

    // code given by henrik_alser for TIM3 based MonoTimer
    use syd::mono::{ExtU32, MonoTimer};

    use stm32f1xx_hal::{
        gpio::{gpioa::{PA2, PA3}, Output, PushPull, Floating, Input, PullUp},
        gpio::gpiob::{PB6, PB7},
        gpio::gpioc::PC13,
        prelude::*,
        qei::{Qei, QeiOptions},
        stm32,
        pac,
        timer::{Tim2NoRemap, Tim4NoRemap, Timer},
    };

    // wrapper around quadratue encoder interface so it becomes i64 in stead of u32
    use qei::QeiManager;

    // switch debouncer; used fro rate selection
    use debouncr::{debounce_2, Repeat2, Edge, Debouncer};

    #[monotonic(binds = TIM3, default = true)]
    type MyMono = MonoTimer<pac::TIM3, 1_000>;


    // Shared resources definition
    #[shared]
    struct Shared {
        tracking: bool,                   // system in tracking state?
        rate_index: usize,                // index in the rating speed table
        setpoint: i64,                    // target position for digital servo
        motor_pos:QeiManager<Qei          // quadrature encoder counter
                             <stm32::TIM4,
                              Tim4NoRemap,
                              (PB6<Input<Floating>>, PB7<Input<Floating>>)>
                             >,
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

        // Set up the onboard LED
        let mut onboard_led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        onboard_led.set_high(); // not sure why unwrap is not implemented.

        // set up signal LED on PA2
        let mut signal_led = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
        signal_led.set_low();


        // set up system timer
        let mono = MyMono::new(cx.device.TIM3, &clocks);

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
        let motor_pos = QeiManager::new(qei);

        //------------
        // tracking rate selection control switch input Pullup.
        let rate_switch = gpioa.pa3.into_pull_up_input(&mut gpioa.crl);

        //------------
        // kick off the threads; they are recalled using sw timer
        main::spawn().ok();
        pid_update::spawn().ok();
        rate_update::spawn().ok();

        // return the resources (and the monotonic timer)
        (
            Shared {
                // Initialization of shared resources go here
                tracking: false,
                rate_index: 2, // start at siderial rate
                setpoint: -i64::MAX, // initial setpoint minus infinite
                motor_pos,
            },
            Local {
                // Initialization of local resources go here
                onboard_led,
                signal_led,
                rate_switch,
                rate_switch_state: debounce_2(false),
                pwm_backward: pwm_backward,
                pwm_forward: pwm_forward,
                pwm_max_duty: max_duty,
            },
            init::Monotonics(mono), // give the monotonic to RTIC
        )
    }


    //------------
    // main loop, maintaining count setpoint
    #[task(
        shared=[tracking, motor_pos, setpoint, rate_index],
        local=[epoch:f64 = 0.0,
               last_position:i64 = 0,
               t_last_position_change:f64 = 0.0],
    )]
    fn main(cx: main::Context) {

        let tracking = cx.shared.tracking;   // bool: are we tracking?
        let motor_pos = cx.shared.motor_pos;

        let epoch = cx.local.epoch;
        let last_position = cx.local.last_position;
        let t_last_position_change = cx.local.t_last_position_change;

        let setpoint = cx.shared.setpoint;
        let mut rate_index = cx.shared.rate_index;


        //------------
        // find time since start of timer
        let t = as_ms(monotonics::now().duration_since_epoch()) as f64 /1000.0;

        //------------
        // set speed table
        // 1.08 deler compenseert voor foute klok?
        const TRACKING_SPEED_mARCS_PER_S: [f64; 3] = [
            15000./1.08,
            14685./1.08,
            15041./1.08 ];

        //------------
        // compute setpoint angle (time * tracking speed)
        let mut angle_setpoint: f64 = 0.0;
        // compute angle setpoint in milli arcseconds (tricky, rounding errors galore)
        rate_index.lock(|rate_index| {
            angle_setpoint =
                TRACKING_SPEED_mARCS_PER_S[*rate_index] * (t - *epoch);
        });


        //------------
        // mount geometry specific coeficients
        // convert angle in  milli-arcseconds to encoder setpoint
        let target_count = (6.4 +
            angle_setpoint  *
            ( 2.8938e-3 + angle_setpoint *
              ( 382.41e-15 + angle_setpoint *
                ( - 20.517e-21 + angle_setpoint * 268.44e-30  )
              )
            )) as i64 ;

        (tracking, motor_pos, setpoint)
            .lock(|tracking, motor_pos, setpoint| {
                // get position of motor
                motor_pos.sample().unwrap();
                let position = motor_pos.count();


                // defmt::println!("*reported time since epoch {}", t);
                // defmt::println!("last t_last_position_change {}", *t_last_position_change);
                // defmt::println!("dt {}", (t-*t_last_position_change));
                // defmt::println!("setpoint {}", *setpoint);
                // defmt::println!("position {}", *last_position);

                //------------
                // check if motor is stalled; assume we are always moving at second scale
                // if stalled, decide what to do next

                if position != *last_position {
                    *t_last_position_change = t;
                }

                *last_position = position;

                if (t - *t_last_position_change) > 3.0 {
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
                        *setpoint = 0;
                        *epoch = t; // start time of this run
                        motor_pos.reset(); // set count to 0 at starting point
                        *t_last_position_change = t; // ensure we don't run into this again immediately
                    }
                }

                //------------
                // set target count if we're tracking
                if *tracking {
                    *setpoint = target_count;
                }

            }); // lock

        main::spawn_after(ExtU32::millis(10u32)).ok();
    }


    // update the PID loop; skeleton
    // with help from henrik_alser and adamgreig
    #[task(
        shared = [setpoint,  // setpoint position
                  motor_pos, // qei encoder
        ],
        local = [onboard_led,
                 integral_prior:f64 = 0.0,
                 error_prior:f64    = 0.0, // PID parameters
                 pwm_backward, pwm_forward, // pwm channels
                 pwm_max_duty, // constant
        ]
    )]
    fn pid_update(cx: pid_update::Context) {

        //------------
        // flip the onboard led
        let onboard_led = cx.local.onboard_led;
        onboard_led.set_low();

        //------------
        // this is the setpoint maintained by the main function
        let setpoint = cx.shared.setpoint;

        //------------
        // qei and motor control
        let motor_pos = cx.shared.motor_pos;

        let pwm_forward = cx.local.pwm_forward;
        let pwm_backward = cx.local.pwm_backward;
        let pwm_max_duty = cx.local.pwm_max_duty;

        //------------
        // PID parameters; experimentally determined, probably highly suboptimal
        let Kp: f64 = 320.0;
        let Ki: f64 = 32.0;
        let Kd: f64 = 0.01;
        let bias: f64 = *pwm_max_duty as f64 / 10.0;

        //------------
        // remember from previous iteration
        let integral_prior = cx.local.integral_prior;
        let error_prior = cx.local.error_prior;

        (setpoint, motor_pos).lock(|setpoint, motor_pos| {

            // get position of motor
            motor_pos.sample().unwrap();
            let position = motor_pos.count();
            let position_error: f64 = *setpoint as f64 - position as f64;

            // pid control; assuming 1000 Hz update rate
            let mut integral = *integral_prior + position_error as f64 / 1000.;
            let derivative = (position_error - *error_prior) * 1000.;
            let mut pwm_setting = Kp * position_error + Ki * integral + Kd * derivative + bias;

            //------------
            // this is to limit transition effect after retraction
            if abs(integral) > 10000.0 {integral = 0.0 * sign(integral)};

            //------------
            // save the values for next iteration
            *integral_prior = integral;
            *error_prior = position_error;

            //------------
            // limit the pwm setting to max duty cycle
            if abs(pwm_setting) > (*pwm_max_duty).into() {
                pwm_setting = *pwm_max_duty as f64 * sign(pwm_setting);
            };

            //------------
            // set the pwm
            if pwm_setting > 0.0 {
                pwm_forward.set_duty(pwm_setting as u16);
                pwm_backward.set_duty(0);
            } else {
                pwm_forward.set_duty(0);
                pwm_backward.set_duty(-pwm_setting as u16);
            };
        });

        //------------
        // switch of on board LED
        onboard_led.set_high();

        //------------
        // respawn
        pid_update::spawn_after(1.millis()).ok();

    } // PID update loop


    //------------
    // update the angular rotation rate and actuate signal led
    #[task(
        shared = [
            rate_index,  // index in rate table
        ],
        local = [rate_switch,
                 rate_switch_state,
                 signal_led,
                 d: u32 = 0 // cycle counter for LED signal
        ]
    )]
    fn rate_update(cx: rate_update::Context) {

        // hw bit handle
        let signal_led = cx.local.signal_led;

        // decimal counter for LED pattern
        // assuming 100ms between calls cycle takes 20 * 0.1 = 2.0 s
        let d: &mut u32 = cx.local.d;
        *d = (*d + 1) % 20;

        // rate index in speed table
        let mut rate_index = cx.shared.rate_index;

        // oll button
        let pressed: bool = cx.local.rate_switch.is_low();

        // update state of button
        let edge = cx.local.rate_switch_state.update(pressed);

        //
        rate_index.lock(|rate_index| {

            //  process event
            if edge == Some(Edge::Falling) {
                *rate_index = (*rate_index +1) % 3; // cycle through rates
                defmt::println!("rate_index {}", *rate_index);
            }

            // signals in tenths of seconds, start times
            const TRACK_PATTERN: [u32; 3] = [0b1, 0b1001, 0b1001001];

            // signal led:
            // check if bit# d is on in the pattern of the rate_index
            // if so, switch on signal LED
            if TRACK_PATTERN[*rate_index] & (0b1 << *d) > 0 {
                signal_led.set_high();
            } else {
                signal_led.set_low();
            };
        }
        );

        // respawn
        rate_update::spawn_after(100.millis()).ok();
    }



    //------------
    // get the length of a Duration in ms
    fn as_ms<const NOM: u32, const DENOM: u32>(
        d: dwt_systick_monotonic::fugit::Duration<u32, NOM, DENOM>
    ) -> i64
    {
        let millis: dwt_systick_monotonic::fugit::MillisDurationU32 = d.convert();
        millis.ticks() as i64
    }

    //------------
    // utility functions not present in f64 implementation in no_std
    fn sign(f: f64) -> f64 {
        if f < 0.0 { -1.0 } else { 1.0 }
    }

    fn abs(x: f64) -> f64 {
        f64::from_bits(x.to_bits() & 0x7FFF_FFFF_FFFF_FFFF)
    }

    // fn abs(f: f64) -> f64 {
    //     f * sign(f)
    // }

}
