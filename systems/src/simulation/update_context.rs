use std::time::Duration;
use uom::si::{f64::*, length::foot, thermodynamic_temperature::degree_celsius, velocity::knot};

/// Provides data unowned by any system in the aircraft system simulation
/// for the purpose of handling a simulation tick.
#[derive(Debug)]
pub struct UpdateContext {
    pub delta: Duration,
    pub indicated_airspeed: Velocity,
    pub indicated_altitude: Length,
    pub ambient_temperature: ThermodynamicTemperature,
    pub is_on_ground: bool,
}
impl UpdateContext {
    pub fn new(
        delta: Duration,
        indicated_airspeed: Velocity,
        indicated_altitude: Length,
        ambient_temperature: ThermodynamicTemperature,
        is_on_ground: bool,
    ) -> UpdateContext {
        UpdateContext {
            delta,
            indicated_airspeed,
            indicated_altitude,
            ambient_temperature,
            is_on_ground,
        }
    }

    pub fn is_in_flight(&self) -> bool {
        !self.is_on_ground
    }
}

pub fn context_with() -> UpdateContextBuilder {
    UpdateContextBuilder::new()
}

pub fn context() -> UpdateContext {
    context_with().build()
}

pub struct UpdateContextBuilder {
    delta: Duration,
    indicated_airspeed: Velocity,
    indicated_altitude: Length,
    ambient_temperature: ThermodynamicTemperature,
    is_on_ground: bool,
}
impl UpdateContextBuilder {
    fn new() -> UpdateContextBuilder {
        UpdateContextBuilder {
            delta: Duration::from_secs(1),
            indicated_airspeed: Velocity::new::<knot>(250.),
            indicated_altitude: Length::new::<foot>(5000.),
            ambient_temperature: ThermodynamicTemperature::new::<degree_celsius>(0.),
            is_on_ground: false,
        }
    }

    pub fn build(&self) -> UpdateContext {
        UpdateContext::new(
            self.delta,
            self.indicated_airspeed,
            self.indicated_altitude,
            self.ambient_temperature,
            self.is_on_ground,
        )
    }

    pub fn and(self) -> UpdateContextBuilder {
        self
    }

    pub fn delta(mut self, delta: Duration) -> UpdateContextBuilder {
        self.delta = delta;
        self
    }

    pub fn indicated_airspeed(mut self, indicated_airspeed: Velocity) -> UpdateContextBuilder {
        self.indicated_airspeed = indicated_airspeed;
        self
    }

    pub fn indicated_altitude(mut self, indicated_altitude: Length) -> UpdateContextBuilder {
        self.indicated_altitude = indicated_altitude;
        self
    }

    pub fn ambient_temperature(
        mut self,
        ambient_temperature: ThermodynamicTemperature,
    ) -> UpdateContextBuilder {
        self.ambient_temperature = ambient_temperature;
        self
    }

    pub fn is_on_ground(mut self, is_on_ground: bool) -> UpdateContextBuilder {
        self.is_on_ground = is_on_ground;
        self
    }
}
