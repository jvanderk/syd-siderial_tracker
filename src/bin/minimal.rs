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
    };

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<48_000_000>; // 48 MHz

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        tracking: bool, // is system in tracking state? if not: retracting
    }

    // Local resources go here
    #[local]
    struct Local {
        onboard_led: PC13<Output<PushPull>>,
        signal_led: PA2<Output<PushPull>>,
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
            .sysclk(48.mhz())
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
        //let tracking: bool = false;

        main::spawn_after(1.millis()).ok();

        // return the resources (and the monotonic timer?)
        (
            Shared {
                // Initialization of shared resources go here
                tracking: false
            },
            Local {
                // Initialization of local resources go here
                onboard_led,
                signal_led
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
    #[task(shared=[tracking], local=[onboard_led,signal_led])]
    fn main(mut cx: main::Context) {
        //defmt::info!("main!");

        let mut tracking = cx.shared.tracking;
        let mut onboard_led = cx.local.onboard_led;
        let mut signal_led = cx.local.signal_led;

        (tracking).lock(|tracking| {
            if *tracking {
                // track
                *tracking = false;
                onboard_led.set_low();
                signal_led.set_high();
            } else {
                // retract and check if we are stalling
                *tracking = true;
                onboard_led.set_high();
                signal_led.set_low();
            }
        });
        main::spawn_after(500.millis()).ok();
    }
}
