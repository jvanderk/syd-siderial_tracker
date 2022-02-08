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

