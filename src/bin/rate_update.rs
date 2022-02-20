use crate::app;
use debouncr::Edge;
use rtic::Mutex;
use syd::mono::ExtU32;

pub(crate) fn rate_update_fn(cx: app::rate_update_fn::Context) {
    // rate index in speed table
    let mut rate_index = cx.shared.rate_index;

    let app::rate_update_fn::LocalResources {
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
    app::rate_update_fn::spawn_after(ExtU32::millis(100u32)).ok();
}
