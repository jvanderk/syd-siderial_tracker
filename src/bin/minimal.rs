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
        let onboard_led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        let signal_led = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);

        let mono = DwtSystick::new(
            &mut cx.core.DCB,
            cx.core.DWT,
            cx.core.SYST,
            clocks.hclk().0,
        );

        blink::spawn_after(1.secs()).ok();

        // Setup the monotonic timer
        (
            Shared {
                // Initialization of shared resources go here
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
    #[task(local=[onboard_led,signal_led])]
    fn blink(cx: blink::Context) {
        defmt::info!("Blink!");
        cx.local.onboard_led.toggle();
        cx.local.signal_led.toggle();
        blink::spawn_after(500.millis()).ok();
    }
}
