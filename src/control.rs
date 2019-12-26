use crate::{Drivers, Sensors};

pub struct Controller {}

impl Controller {
    pub fn new() -> Self {
        Controller {}
    }

    pub fn control(&mut self, _sensors: &Sensors, drivers: &mut Drivers) {
        drivers.lin = 0.0;
    }
}
