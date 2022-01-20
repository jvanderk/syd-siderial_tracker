#![deny(unsafe_code)]
#![no_main]
#![no_std]

use syd as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device =  stm32f1xx_hal::pac, peripherals = true, dispatchers = [USART1])]
mod app {

    use dwt_systick_monotonic::{DwtSystick, ExtU32};

    use stm32f1xx_hal::{
        gpio::{gpioa::{PA0,PA1,PA2}, Output, PushPull},
        gpio::{gpioa::{PA3}, Input, Floating, PullUp},
        gpio::{gpiob::{PB6,PB7}},
        gpio::{gpioc::{PC13}},
        prelude::*,
        timer::{Event, Timer, CountDownTimer},
        stm32,
    };

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<48_000_000>; // 48 MHz

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        tracking: bool, // is system in tracking state? if not: retracting
        t: i64,         // discrete time in tenths of seconds
    }

    // Local resources go here
    #[local]
    struct Local {
        onboard_led: PC13<Output<PushPull>>,
        signal_led: PA2<Output<PushPull>>,
        last_pos: i64,
        last_pos_change_time: i64,
        tmr2: CountDownTimer<stm32::TIM2>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {

        defmt::info!("init");

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        // Acquire the GPIOC peripherals
        let mut gpioa = cx.device.GPIOA.split();
        // let mut gpiob = dp.GPIOB.split(&mut rcc.apb2); // quadrature counter
        let mut gpioc = cx.device.GPIOC.split();

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(48.mhz())
            .hclk(48.mhz())
            .pclk1(8.mhz())
            .pclk2(8.mhz())
            .adcclk(8.mhz())
            .freeze(&mut flash.acr);

        // Set up the LED.
        let mut onboard_led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        onboard_led.set_high(); // not sure why unwrap is not implemented.
        let mut signal_led = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
        signal_led.set_low();

        let mono = DwtSystick::new(
            &mut cx.core.DCB,
            cx.core.DWT,
            cx.core.SYST,
            clocks.hclk().0,
        );

        // app setup

        // Use TIM2 for the PID counter task
        let mut tmr2 =
            Timer::tim2(cx.device.TIM2, &clocks).start_count_down(4.hz());
        tmr2.listen(Event::Update);

        //let tracking: bool = false;

        main::spawn_after(1.millis()).ok();

        // return the resources (and the monotonic timer?)
        (
            Shared {
                // Initialization of shared resources go here
                tracking: false,
                t: 0
            },
            Local {
                // Initialization of local resources go here
                onboard_led,
                signal_led,
                last_pos: 0,
                last_pos_change_time: 0,
                tmr2
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
        //defmt::info!("main!");

        let tracking = cx.shared.tracking;
        let t = cx.shared.t;
        let signal_led = cx.local.signal_led;

        (tracking, t).lock(|tracking, t| {
            if *tracking {
                // track
                *tracking = false;
                *t += 5;
                signal_led.set_high();
            } else {
                // retract and check if we are stalling
                *tracking = true;
                *t += 5;
                signal_led.set_low();
            }
        });
        main::spawn_after(500.millis()).ok();
    }


    // Interrupt task for TIM2, the PID counter timer
    #[task(binds = TIM2, priority = 1, local = [tmr2])]
    fn tim2(cx: tim2::Context) {
        // Delegate the state update to a software task
        pid_update::spawn().unwrap();
        // Restart the timer and clear the interrupt flag
        cx.local.tmr2.start(4.hz());
        cx.local.tmr2.clear_update_interrupt_flag();
    }


    // update the PID loop
    #[task(shared = [t], local = [onboard_led])]
    fn pid_update(cx: pid_update::Context) {
        let onboard_led = cx.local.onboard_led;
        let mut t = cx.shared.t;
        t.lock(|t| {
            *t += 1;
            onboard_led.toggle();
        })
    }

}
