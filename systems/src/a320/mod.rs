use self::{fuel::A320Fuel, pneumatic::A320PneumaticOverheadPanel};
use crate::{
    apu::{
        AuxiliaryPowerUnit, AuxiliaryPowerUnitFireOverheadPanel, AuxiliaryPowerUnitOverheadPanel,
    },
    electrical::{ElectricalBusStateFactory, ExternalPowerSource, PowerConsumptionHandler},
    engine::Engine,
    simulator::{
        Aircraft, SimulatorElement, SimulatorElementVisitable, SimulatorElementVisitor,
        UpdateContext,
    },
};
use uom::si::f64::*;

mod electrical;
pub use electrical::*;

mod hydraulic;
pub use hydraulic::*;

mod fuel;

mod pneumatic;

pub struct A320 {
    apu: AuxiliaryPowerUnit,
    apu_fire_overhead: AuxiliaryPowerUnitFireOverheadPanel,
    apu_overhead: AuxiliaryPowerUnitOverheadPanel,
    pneumatic_overhead: A320PneumaticOverheadPanel,
    electrical_overhead: A320ElectricalOverheadPanel,
    fuel: A320Fuel,
    engine_1: Engine,
    engine_2: Engine,
    electrical: A320Electrical,
    ext_pwr: ExternalPowerSource,
    hydraulic: A320Hydraulic,
    hydraulic_overhead: A320HydraulicOverheadPanel,
}
impl A320 {
    pub fn new() -> A320 {
        A320 {
            apu: AuxiliaryPowerUnit::new_aps3200(1),
            apu_fire_overhead: AuxiliaryPowerUnitFireOverheadPanel::new(),
            apu_overhead: AuxiliaryPowerUnitOverheadPanel::new(),
            pneumatic_overhead: A320PneumaticOverheadPanel::new(),
            electrical_overhead: A320ElectricalOverheadPanel::new(),
            fuel: A320Fuel::new(),
            engine_1: Engine::new(1),
            engine_2: Engine::new(2),
            electrical: A320Electrical::new(),
            ext_pwr: ExternalPowerSource::new(),
            hydraulic: A320Hydraulic::new(),
            hydraulic_overhead: A320HydraulicOverheadPanel::new(),
        }
    }
}
impl Default for A320 {
    fn default() -> Self {
        Self::new()
    }
}
impl Aircraft for A320 {
    fn update(&mut self, context: &UpdateContext) {
        self.fuel.update();

        self.apu.update(
            context,
            &self.apu_overhead,
            &self.apu_fire_overhead,
            self.pneumatic_overhead.apu_bleed_is_on(),
            // This will be replaced when integrating the whole electrical system.
            // For now we use the same logic as found in the JavaScript code; ignoring whether or not
            // the engine generators are supplying electricity.
            self.electrical_overhead.apu_generator_is_on()
                && !(self.electrical_overhead.external_power_is_on()
                    && self.electrical_overhead.external_power_is_available()),
            self.fuel.left_inner_tank_has_fuel_remaining(),
        );
        self.apu_overhead.update_after_apu(&self.apu);
        self.pneumatic_overhead.update_after_apu(&self.apu);

        self.electrical.update(
            context,
            &self.engine_1,
            &self.engine_2,
            &self.apu,
            &self.ext_pwr,
            &self.hydraulic,
            &self.electrical_overhead,
        );
        self.electrical_overhead.update_after_elec(&self.electrical);

        self.hydraulic.update(
            context,
            &self.engine_1,
            &self.engine_2,
        );

        let power_supply = self.electrical.create_power_supply();
        let mut power_consumption_handler = PowerConsumptionHandler::new(&power_supply);
        power_consumption_handler.supply_power_to_elements(&mut Box::new(self));

        // Update everything that needs to know if it is powered here.

        power_consumption_handler.determine_power_consumption(&mut Box::new(self));
        power_consumption_handler.write_power_consumption(&mut Box::new(self));
    }
}
impl SimulatorElementVisitable for A320 {
    fn accept(&mut self, visitor: &mut Box<&mut dyn SimulatorElementVisitor>) {
        self.apu.accept(visitor);
        self.apu_fire_overhead.accept(visitor);
        self.apu_overhead.accept(visitor);
        self.electrical_overhead.accept(visitor);
        self.fuel.accept(visitor);
        self.pneumatic_overhead.accept(visitor);
        self.engine_1.accept(visitor);
        self.engine_2.accept(visitor);
        self.electrical.accept(visitor);
        self.ext_pwr.accept(visitor);
        visitor.visit(&mut Box::new(self));
    }
}
impl SimulatorElement for A320 {}
