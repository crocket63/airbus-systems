use crate::{
    overhead::OnOffFaultPushButton,
    pneumatic::BleedAirValveState,
    simulator::{SimulatorElement, SimulatorElementVisitable, SimulatorElementVisitor},
};

pub struct A320PneumaticOverheadPanel {
    apu_bleed: OnOffFaultPushButton,
}
impl A320PneumaticOverheadPanel {
    pub fn new() -> Self {
        A320PneumaticOverheadPanel {
            apu_bleed: OnOffFaultPushButton::new_on("PNEU_APU_BLEED"),
        }
    }

    pub fn update_after_apu<T: BleedAirValveState>(&mut self, apu: &T) {
        self.apu_bleed
            .set_fault(self.apu_bleed.is_on() != apu.bleed_air_valve_is_open());
    }

    pub fn apu_bleed_is_on(&self) -> bool {
        self.apu_bleed.is_on()
    }

    #[cfg(test)]
    pub fn apu_bleed_has_fault(&self) -> bool {
        self.apu_bleed.has_fault()
    }
}
impl SimulatorElementVisitable for A320PneumaticOverheadPanel {
    fn accept(&mut self, visitor: &mut Box<&mut dyn SimulatorElementVisitor>) {
        self.apu_bleed.accept(visitor);

        visitor.visit(&mut Box::new(self));
    }
}
impl SimulatorElement for A320PneumaticOverheadPanel {}

#[cfg(test)]
pub mod tests {
    use super::A320PneumaticOverheadPanel;
    use crate::{
        pneumatic::BleedAirValveState,
        simulator::{SimulatorElement, SimulatorWriter},
    };

    fn overhead() -> A320PneumaticOverheadPanel {
        A320PneumaticOverheadPanel::new()
    }

    struct TestBleedAirValveState {
        open: bool,
    }
    impl TestBleedAirValveState {
        fn new(open: bool) -> Self {
            TestBleedAirValveState { open }
        }
    }
    impl BleedAirValveState for TestBleedAirValveState {
        fn bleed_air_valve_is_open(&self) -> bool {
            self.open
        }
    }

    fn bleed_air_valve_state(open: bool) -> impl BleedAirValveState {
        TestBleedAirValveState::new(open)
    }

    #[test]
    fn when_apu_bleed_on_and_valve_is_closed_fault_is_shown() {
        let mut overhead = overhead();
        overhead.apu_bleed.turn_on();

        overhead.update_after_apu(&bleed_air_valve_state(false));

        assert!(overhead.apu_bleed_has_fault());
    }

    #[test]
    fn when_apu_bleed_on_and_valve_is_open_fault_is_not_shown() {
        let mut overhead = overhead();
        overhead.apu_bleed.turn_on();

        overhead.update_after_apu(&bleed_air_valve_state(true));

        assert!(!overhead.apu_bleed_has_fault());
    }

    #[test]
    fn when_apu_bleed_off_and_valve_is_open_fault_is_shown() {
        let mut overhead = overhead();
        overhead.apu_bleed.turn_off();

        overhead.update_after_apu(&bleed_air_valve_state(true));

        assert!(overhead.apu_bleed_has_fault());
    }

    #[test]
    fn when_apu_bleed_off_and_valve_is_closed_fault_is_not_shown() {
        let mut overhead = overhead();
        overhead.apu_bleed.turn_off();

        overhead.update_after_apu(&bleed_air_valve_state(false));

        assert!(!overhead.apu_bleed_has_fault());
    }
}
