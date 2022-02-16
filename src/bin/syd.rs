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

    // External crate qei contains a wrapper around quadrature encoder
    // interface module in HAL so it becomes a i64 in stead of u32
    // device; The Qei from the HAL is an argument for the constructor
    // of the manager.
    use qei as qeimanager;
    use qeimanager::QeiManager;

    // switch debouncer; used for tracking rate selection
    use debouncr::{debounce_2, Debouncer, Edge, Repeat2};

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

        let max_duty = pwm.get_max_duty();

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
        pid_update::spawn().ok();
        rate_update::spawn().ok();

        // return the resources (and the monotonic timer)
        (
            Shared {
                // Initialization of shared resources go here
                tracking: false,
                rate_index: 2,       // start at siderial rate
                setpoint: -i64::MAX, // initial setpoint minus infinite
                motor_pos,
            },
            Local {
                // Initialization of local resources go here
                onboard_led,
                signal_led,
                rate_switch,
                rate_switch_state: debounce_2(false),
                pwm_backward,
                pwm_forward,
                pwm_max_duty: max_duty,
            },
            init::Monotonics(mono), // give the monotonic to RTIC
        )
    }

    // main loop, maintaining count setpoint and dealing with retraction
    #[task(
        shared=[tracking,
	    motor_pos,
	    setpoint,
	    rate_index
	],
        local=[epoch:f32 = 0.0,
               last_position:i64 = 0,
               t_last_position_change:f32 = 0.0],
    )]
    fn main(cx: main::Context) {
        let tracking = cx.shared.tracking; // bool: are we tracking?
        let motor_pos = cx.shared.motor_pos;

        let epoch = cx.local.epoch;
        let last_position = cx.local.last_position;
        let t_last_position_change = cx.local.t_last_position_change;

        let setpoint = cx.shared.setpoint;
        let mut rate_index = cx.shared.rate_index;

        //------------
        // find time since start of timer
        let t = as_ms(monotonics::now().duration_since_epoch()) as f32 / 1000.0;

        //------------
        // set speed table
        // 1.08 deler compenseert voor foute klok?
        const correction: f32 = 1.08;
        const TRACKING_SPEED_mARCS_PER_S: [f32; 3] = [
            15000. / correction, // Solar
            14685. / correction, // Lunar
            15041. / correction, // Siderial
        ];

        // compute setpoint angle (tracking speed * time since start run)
        // compute angle setpoint in milli arcseconds (approach inherited from integer arithmetic)
        let angle_setpoint: f32 =
            rate_index.lock(|rate_index| TRACKING_SPEED_mARCS_PER_S[*rate_index] * (t - *epoch));

        // mount geometry specific coeficients
        // convert angle in  milli-arcseconds to encoder setpoint
        // compute setpoint indepent of whether or not we are tracking.
        let mut target_count: i64 = (6.4
            + angle_setpoint
                * (2.8938e-3
                    + angle_setpoint
                        * (382.41e-15
                            + angle_setpoint * (-20.517e-21 + angle_setpoint * 268.44e-30))))
            as i64;

        // this lock checks the position and sets the setpoint
        // it can be made smaller but then the logic becomes unwieldy, and the code is small anyway.
        (tracking, motor_pos, setpoint).lock(|tracking, motor_pos, setpoint| {
            // get position of motor
            motor_pos.sample().unwrap();
            let position = motor_pos.count();

            // record last time that position count changed
            if position != *last_position {
                *t_last_position_change = t;
            }
            *last_position = position;

            // if motor is stalled flip tracking state
            if (t - *t_last_position_change) > 3.0 {
                defmt::println!("stalled");

                if *tracking {
                    // stalled and tracking
                    *tracking = false;
                } else {
                    // stalled and not tracking
                    *tracking = true;

                    // record start time of this run and disarm stalled detection
                    *epoch = t;
                    *t_last_position_change = t;

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
                *setpoint = -i64::MAX;
            }
        }); // lock

        // about 50 ticks per second, so if we have a repeat frequency of 100 Hz we should be tracking tick by tick.
        main::spawn_after(ExtU32::millis(10u32)).ok();
    }

    // update the PID loop
    // with help from henrik_alser and adamgreig
    #[task(
        shared = [setpoint,  // setpoint position
                  motor_pos, // qei encoder
        ],
        local = [onboard_led,
                 integral_prior:f32 = 0.0,
                 error_prior:f32    = 0.0, // PID parameters
                 pwm_backward, pwm_forward, // pwm channels
                 pwm_max_duty, // constant
        ]
    )]
    fn pid_update(cx: pid_update::Context) {
        let pid_update::LocalResources {
            onboard_led,
            integral_prior, // PID parameters
            error_prior,
            pwm_backward, // pwm API
            pwm_forward,
            pwm_max_duty,
        } = cx.local;

        // switch on the onboard led (inverted logic)
        onboard_led.set_low();

        // this is the setpoint maintained by the main() function
        let setpoint = cx.shared.setpoint;

        // qei and motor control
        let motor_pos = cx.shared.motor_pos;

        // PID parameters; experimentally determined, probably suboptimal
        let Kp: f32 = 320.0;
        let Ki: f32 = 32.0;
        let Kd: f32 = 0.01;
        let bias: f32 = *pwm_max_duty as f32 / 10.0;

        // compute position error
        // lock returns the error
        let position_error: f32 = (setpoint, motor_pos).lock(|setpoint, motor_pos| {
            motor_pos.sample().unwrap();
            (*setpoint - motor_pos.count()) as f32
        });

        // pid control; assuming 1000 Hz update rate
        let mut integral = *integral_prior + position_error / 1000.;

        // this is to limit transition effect after retraction
        if abs(integral) > 100_000.0 {
            integral = 0.0
        };

        let derivative = (position_error - *error_prior) * 1000.;

        let mut pwm_setting = Kp * position_error + Ki * integral + Kd * derivative + bias;
        // limit the pwm setting to max duty cycle
        if abs(pwm_setting) > *pwm_max_duty as f32 {
            pwm_setting = *pwm_max_duty as f32 * sign(pwm_setting);
        };

        // save the values for next iteration
        *integral_prior = integral;
        *error_prior = position_error;

        // set the pwm hw
        if pwm_setting > 0.0 {
            // forward
            pwm_forward.set_duty(pwm_setting as u16);
            pwm_backward.set_duty(0);
        } else {
            // retract
            pwm_forward.set_duty(0);
            pwm_backward.set_duty(-pwm_setting as u16);
        };

        // switch of on board LED
        onboard_led.set_high();

        // respawn 1 kHz
        pid_update::spawn_after(1.millis()).ok();
    } // PID update loop

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
    fn rate_update(cx: rate_update::Context) {
        // rate index in speed table
        let mut rate_index = cx.shared.rate_index;

        let rate_update::LocalResources {
            rate_switch,
            rate_switch_state,
            signal_led,
            decis,
        } = cx.local;

        // decimal counter for LED pattern
        // assuming 100ms between calls cycle takes 20 * 0.1 = 2.0 s
        *decis = (*decis + 1) % 20;

        // signals in tenths of seconds, start times
        const TRACK_PATTERN: [u32; 3] = [0b1, 0b1001, 0b1001001];

        // poll button and update state; *rate_index is shared so we need to lock.
        let pressed: bool = rate_switch.is_low();
        let edge = rate_switch_state.update(pressed);
        let r = rate_index.lock(|rate_index| {
            //  process event
            if edge == Some(Edge::Falling) {
                *rate_index = (*rate_index + 1) % 3 // cycle through rates
            }
            *rate_index // return value
        });

        if edge == Some(Edge::Falling) {
            defmt::println!("rate_index {}", r)
        }

        // signal led:
        // switch on if bit# d is on in the pattern of the rate_index
        if TRACK_PATTERN[r] & (0b1 << *decis) > 0 {
            signal_led.set_high();
        } else {
            signal_led.set_low();
        };

        // respawn
        rate_update::spawn_after(100.millis()).ok();
    }

    //------------
    // get the length of a Duration in ms
    fn as_ms<const NOM: u32, const DENOM: u32>(
        d: dwt_systick_monotonic::fugit::Duration<u32, NOM, DENOM>,
    ) -> u32 {
        let millis: dwt_systick_monotonic::fugit::MillisDurationU32 = d.convert();
        millis.ticks()
    }

    //------------
    // utility functions not present in f32 implementation in no_std
    fn sign(f: f32) -> f32 {
        if f < 0.0 {
            -1.0
        } else {
            1.0
        }
    }

    fn abs(x: f32) -> f32 {
        f32::from_bits(x.to_bits() & 0x7FFF_FFFF)
    }
}
