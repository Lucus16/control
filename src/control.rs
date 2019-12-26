use crate::{DroneDrivers, DroneSensors};

fn drone(_sensors: &DroneSensors, drivers: &mut DroneDrivers) {
    drivers.fl = 1.0;
    drivers.fr = 1.0;
    drivers.bl = 1.0;
    drivers.br = 1.0;
}
