#![allow(unused)]
use crate::{Drivers, Sensors, F};

pub struct Controller {
    last_lin: F,
    last_pole_pos: F,
}

struct NaivePID {
    kp: F,
    ki: F,
    kd: F,
    prev: F,
    sum: F,
}

impl NaivePID {
    fn new(kp: F, ki: F, kd: F) -> Self {
        Self {
            kp,
            ki,
            kd,
            prev: 0.0,
            sum: 0.0,
        }
    }

    fn step(&mut self, error: F) -> F {
        let diff = error - self.prev;
        self.prev = error;
        self.sum += error;
        self.kp * error + self.kd * diff + self.ki * self.sum
    }
}

impl Controller {
    pub fn new() -> Self {
        Controller {
            last_lin: 0.0,
            last_pole_pos: 0.0,
        }
    }

    pub fn control(&mut self, sensors: &Sensors, drivers: &mut Drivers) {
        let pole_pos = sensors.lin + sensors.angle;
        let current_pole_speed = pole_pos - self.last_pole_pos;
        let target_pole_speed = -0.5 * pole_pos;
        let pole_speed_needed = target_pole_speed - current_pole_speed;
        let current_angle = sensors.angle;
        //let target = num::clamp(-0.01 * sensors.lin, -0.02, 0.02);
        let target_angle = 1.0 * pole_speed_needed;
        let angle_needed = target_angle - current_angle;
        let angle_needed = -2.5 * pole_pos + self.last_pole_pos + sensors.lin;
        drivers.lin = -10000.0 * angle_needed * angle_needed.abs();
        self.last_lin = sensors.lin;
        self.last_pole_pos = pole_pos;
        // let angle_needed = 1.0 * ((-0.5 * cur_pole) - (cur_pole - last_pole)) - (cur_pole - cur_base)
        // let angle_needed = 1.0 * ((-0.5 * cur_pole) - cur_vpole) - sensors.angle
    }
}
