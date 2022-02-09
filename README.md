# `syd`

## syd: siderial and moon tracker

This is code based on the 'minimal' template ca. 2022Q1 (RTIC 1.0). It drives a
siderial tracker for the northern hemisphere that can run at three
different speeds. It runs on a bluepill (stm32f103).

The geometry of the tracker is a rotating arm connected at the long
end to a nut on a linear thread that is driven by a DC motor
assembly. The DC motor has a quadrature encoder and is driven by an
H-bridge using a PWM signal in a PID loop.

Main functions:
- retract drive at start and when stalled if in tracking mode
- set count based on time and tracking speed
- allow user to select drive speed using on-off switch
- feedback selected drive speed index on indicator LED

Great help for putting this together was given by the great folks at
#rtic:matrix.org .

## feedback on this code:

| comment                                                                                                                      | response                                                                                                                            |
|------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------|
| Move the math from f64 to f32, the MCU has HW support for f32                                                                | The reason I was using f64 is that 268E-30 term in the polynomial; but I may not have thought that through well. Will look into it. |
| Use cargo fmt :)                                                                                                             | Interesting; smart formatter. I'm on emacs. is 'cargo fmt' the same as 'rustfmt'?                                                   |
| Much of the "working code" in each task could be split out to modules, making the app much smaller.                          |                                                                                                                                     |
| You manually de-structure resources quite often, have a look https://rtic.rs/1.0/book/en/by-example/tips_destructureing.html |                                                                                                                                     |
| Some of your locks are quite large, try to reduce locks as much as possible to not hinder concurrency.                       |                                                                                                                                     |
| use 'cargo clippy'                                                                                                           |                                                                                                                                     |
| seems you have a workspace defined with 'testsuite' member but I see no such crate.                                          |                                                                                                                                     |
| Use https://robert.kra.hn/posts/2021-02-07_rust-with-emacs/ (look into rust-analyzer)                                        |                                                                                                                                     |
