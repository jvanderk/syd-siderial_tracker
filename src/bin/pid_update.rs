use crate::app;

use rtic::mutex_prelude::TupleExt02;
use stm32f1xx_hal::prelude::_embedded_hal_PwmPin;
use syd::math_embedded::{abs, sign};
use syd::mono::ExtU32;

pub(crate) fn pid_update_fn(cx: app::pid_update_fn::Context) {
    let app::pid_update_fn::LocalResources {
        respawn_delay_ms,
        integral_prior, // PID parameters
        error_prior,
        pwm_backward, // pwm API
        pwm_forward,
        pwm_max_duty,
    } = cx.local;

    // this is the setpoint maintained by the main() function
    let setpoint = cx.shared.setpoint;

    // qeimanager4
    let motor_pos = cx.shared.motor_pos;

    // PID parameters; experimentally determined, probably suboptimal
    let Kp: f32 = 150.0;
    let Ki: f32 = 0.01;
    let Kd: f32 = 16.;
    let bias: f32 = (*pwm_max_duty * 0) as f32;
    let dt: f32 = 1000. / *respawn_delay_ms as f32;

    // compute position error
    // lock returns the error
    let position_error: f32 = (setpoint, motor_pos).lock(|setpoint, motor_pos| {
        motor_pos.sample().unwrap();
        (*setpoint - motor_pos.count()) as f32
    });

    // pid control
    let mut integral = *integral_prior + position_error * dt;

    // this is to limit transition effect after retraction
    if abs(integral) > 100_000.0 {
        integral = 0.0
    };

    let derivative = (position_error - *error_prior) / dt;

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

    // respawn 1 kHz
    app::pid_update_fn::spawn_after(respawn_delay_ms.millis()).ok();
} // PID update loop
