#![cfg(any(target_arch = "wasm32", doc))]
use airbus_systems::{
    simulator::{
        from_bool, to_bool, ModelToSimulatorVisitor, SimulatorReadState, SimulatorToModelVisitor,
        SimulatorVisitable, SimulatorWriteState,
    },
    A320,
};
use msfs::{
    legacy::{AircraftVariable, NamedVariable},
    MSFSEvent,
};
use uom::si::{
    electric_current::ampere, electric_potential::volt, f64::*, frequency::hertz, length::foot,
    ratio::percent, thermodynamic_temperature::degree_celsius, velocity::knot,
};

#[msfs::gauge(name=airbus)]
async fn demo(mut gauge: msfs::Gauge) -> Result<(), Box<dyn std::error::Error>> {
    let mut a320 = A320::new();
    let sim_read_writer = SimulatorReadWriter::new()?;

    while let Some(event) = gauge.next_event().await {
        match event {
            MSFSEvent::PreDraw(d) => {
                println!("TICK");

                let state = sim_read_writer.read();
                let mut visitor = SimulatorToModelVisitor::new(&state);
                a320.accept(&mut visitor);

                a320.update(&state.to_context(d.delta_time()));

                let mut visitor = ModelToSimulatorVisitor::new();
                a320.accept(&mut visitor);

                sim_read_writer.write(&visitor.get_state());
            }
            _ => {}
        }
    }

    Ok(())
}

pub struct SimulatorReadWriter {
    ambient_temperature: AircraftVariable,
    apu_egt: NamedVariable,
    apu_egt_caution: NamedVariable,
    apu_egt_warning: NamedVariable,
    apu_flap_open: NamedVariable,
    apu_gen_amperage: NamedVariable,
    apu_gen_frequency: NamedVariable,
    apu_gen_voltage: NamedVariable,
    apu_master_sw: AircraftVariable,
    apu_n: NamedVariable,
    apu_start_contactor_energized: NamedVariable,
    apu_start_sw_available: NamedVariable,
    apu_start_sw_on: NamedVariable,
    indicated_airspeed: AircraftVariable,
    indicated_altitude: AircraftVariable,
}
impl SimulatorReadWriter {
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        Ok(SimulatorReadWriter {
            ambient_temperature: AircraftVariable::from("AMBIENT TEMPERATURE", "celsius", 0)?,
            apu_egt: NamedVariable::from("A32NX_APU_EGT"),
            apu_egt_caution: NamedVariable::from("A32NX_APU_EGT_CAUTION"),
            apu_egt_warning: NamedVariable::from("A32NX_APU_EGT_WARNING"),
            apu_flap_open: NamedVariable::from("APU_FLAP_OPEN"),
            apu_gen_amperage: NamedVariable::from("A32NX_APU_GEN_AMPERAGE"),
            apu_gen_frequency: NamedVariable::from("A32NX_APU_GEN_FREQ"),
            apu_gen_voltage: NamedVariable::from("A32NX_APU_GEN_VOLTAGE"),
            apu_master_sw: AircraftVariable::from("FUELSYSTEM VALVE SWITCH", "Bool", 8)?,
            apu_n: NamedVariable::from("A32NX_APU_N"),
            apu_start_contactor_energized: NamedVariable::from(
                "A32NX_APU_START_CONTACTOR_ENERGIZED",
            ),
            apu_start_sw_available: NamedVariable::from("A32NX_APU_AVAILABLE"),
            apu_start_sw_on: NamedVariable::from("A32NX_APU_START_ACTIVATED"),
            indicated_airspeed: AircraftVariable::from("AIRSPEED INDICATED", "Knots", 0)?,
            indicated_altitude: AircraftVariable::from("INDICATED ALTITUDE", "Feet", 0)?,
        })
    }

    pub fn read(&self) -> SimulatorReadState {
        SimulatorReadState {
            ambient_temperature: ThermodynamicTemperature::new::<degree_celsius>(
                self.ambient_temperature.get(),
            ),
            apu_master_sw_on: to_bool(self.apu_master_sw.get()),
            apu_start_sw_on: to_bool(self.apu_start_sw_on.get_value()),
            apu_bleed_sw_on: true, // TODO
            indicated_airspeed: Velocity::new::<knot>(self.indicated_airspeed.get()),
            indicated_altitude: Length::new::<foot>(self.indicated_altitude.get()),
        }
    }

    pub fn write(&self, state: &SimulatorWriteState) {
        self.apu_egt
            .set_value(state.apu_egt.get::<degree_celsius>());
        self.apu_egt_caution
            .set_value(state.apu_caution_egt.get::<degree_celsius>());
        self.apu_egt_warning
            .set_value(state.apu_warning_egt.get::<degree_celsius>());
        self.apu_flap_open
            .set_value(state.apu_air_intake_flap_opened_for.get::<percent>());
        self.apu_gen_amperage
            .set_value(state.apu_gen_current.get::<ampere>());
        self.apu_gen_frequency
            .set_value(state.apu_gen_frequency.get::<hertz>());
        self.apu_gen_voltage
            .set_value(state.apu_gen_potential.get::<volt>());
        self.apu_n.set_value(state.apu_n.get::<percent>());
        self.apu_start_contactor_energized
            .set_value(from_bool(state.apu_start_contactor_energized));
        self.apu_start_sw_available
            .set_value(from_bool(state.apu_start_sw_available));
        self.apu_start_sw_on
            .set_value(from_bool(state.apu_start_sw_on));
    }
}