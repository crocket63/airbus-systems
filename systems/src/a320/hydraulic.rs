use std::time::{Duration, Instant};
use uom::si::{
    area::square_meter, f64::*, force::newton, length::foot, length::meter,
    mass_density::kilogram_per_cubic_meter, pressure::atmosphere, pressure::pascal, pressure::psi,
    ratio::percent, thermodynamic_temperature::degree_celsius, time::second, velocity::knot,
    volume::cubic_inch, volume::gallon, volume::liter, volume_rate::cubic_meter_per_second,
    volume_rate::gallon_per_second,
};
use crate::{engine::Engine, hydraulic::{ElectricPump, EngineDrivenPump, HydFluid, HydLoop, LoopColor, PressureSource, Ptu, Pump, RatPump}, overhead::{AutoOffFaultPushButton,OnOffFaultPushButton}, shared::DelayedTrueLogicGate, simulator::UpdateContext};
use crate::simulator::{
    SimulatorElement, SimulatorElementVisitable, SimulatorElementVisitor, SimulatorReader,
};
pub struct A320Hydraulic {
    blue_loop: HydLoop,
    green_loop: HydLoop,
    yellow_loop: HydLoop,
    engine_driven_pump_1: EngineDrivenPump,
    engine_driven_pump_2: EngineDrivenPump,
    blue_electric_pump: ElectricPump,
    yellow_electric_pump: ElectricPump,
    ptu: Ptu,
    total_sim_time_elapsed: Duration,
    lag_time_accumulator: Duration,
    debug_refresh_duration: Duration,
    // Until hydraulic is implemented, we'll fake it with this boolean.
    // blue_pressurised: bool,
}

impl A320Hydraulic {
    const MIN_PRESS_PRESSURISED : f64 = 300.0;
    const HYDRAULIC_SIM_TIME_STEP : u64 = 100; //refresh rate of hydraulic simulation in ms
    const ACTUATORS_SIM_TIME_STEP_MULT : u32 = 2; //refresh rate of actuators as multiplier of hydraulics. 2 means double frequency update

    pub fn new() -> A320Hydraulic {
        A320Hydraulic {

            blue_loop: HydLoop::new(
                LoopColor::Blue,
                false,
                false,
                Volume::new::<gallon>(1.5),
                Volume::new::<gallon>(1.6),
                Volume::new::<gallon>(1.6),
                Volume::new::<gallon>(1.5),
                HydFluid::new(Pressure::new::<pascal>(1450000000.0)),
            ),
            green_loop: HydLoop::new(
                LoopColor::Green,
                true,
                false,
                Volume::new::<gallon>(10.2),
                Volume::new::<gallon>(10.2),
                Volume::new::<gallon>(8.0),
                Volume::new::<gallon>(3.3),
                HydFluid::new(Pressure::new::<pascal>(1450000000.0))
            ),
            yellow_loop: HydLoop::new(
                LoopColor::Blue,
                false,
                true,
                Volume::new::<gallon>(26.00),
                Volume::new::<gallon>(26.41),
                Volume::new::<gallon>(10.0),
                Volume::new::<gallon>(3.83),
                HydFluid::new(Pressure::new::<pascal>(1450000000.0))
            ),
            engine_driven_pump_1: EngineDrivenPump::new(),
            engine_driven_pump_2: EngineDrivenPump::new(),
            blue_electric_pump: ElectricPump::new(),
            yellow_electric_pump: ElectricPump::new(),
            ptu : Ptu::new(),
            total_sim_time_elapsed: Duration::new(0,0),
            lag_time_accumulator: Duration::new(0,0),
            debug_refresh_duration: Duration::new(0,0),
        }
    }

    pub fn is_blue_pressurised(&self) -> bool {
        self.blue_loop.get_pressure().get::<psi>() >= A320Hydraulic::MIN_PRESS_PRESSURISED
    }

    pub fn is_green_pressurised(&self) -> bool {
        self.green_loop.get_pressure().get::<psi>() >= A320Hydraulic::MIN_PRESS_PRESSURISED
    }

    pub fn is_yellow_pressurised(&self) -> bool {
        self.yellow_loop.get_pressure().get::<psi>() >= A320Hydraulic::MIN_PRESS_PRESSURISED
    }

    pub fn update(&mut self, ct: &UpdateContext, engine1 : &Engine, engine2 : &Engine) {

        let min_hyd_loop_timestep = Duration::from_millis(A320Hydraulic::HYDRAULIC_SIM_TIME_STEP); //Hyd Sim rate = 10 Hz


        self.total_sim_time_elapsed += ct.delta;


        //time to catch up in our simulation = new delta + time not updated last iteration
        let time_to_catch=ct.delta + self.lag_time_accumulator;


        //Number of time steps to do according to required time step
        let number_of_steps_f64 = time_to_catch.as_secs_f64()/min_hyd_loop_timestep.as_secs_f64();

        self.debug_refresh_duration+=ct.delta;
        if self.debug_refresh_duration > Duration::from_secs_f64(0.3) {
            println!("---HYDRAULIC UPDATE : t={}", self.total_sim_time_elapsed.as_secs_f64());
            println!("---G: {:.0} B: {:.0} Y: {:.0}", self.green_loop.get_pressure().get::<psi>(),self.blue_loop.get_pressure().get::<psi>(),self.yellow_loop.get_pressure().get::<psi>());
            println!("---EDP1 n2={} EDP2 n2={}", engine1.n2.get::<percent>(), engine2.n2.get::<percent>());
            println!("---EDP1 flowMax={:.1}gpm EDP2 flowMax={:.1}gpm", (self.engine_driven_pump_1.get_delta_vol_max().get::<gallon>() / min_hyd_loop_timestep.as_secs_f64() )* 60.0, (self.engine_driven_pump_2.get_delta_vol_max().get::<gallon>()/min_hyd_loop_timestep.as_secs_f64())*60.0);

            println!("---steps required: {:.2}", number_of_steps_f64);
            self.debug_refresh_duration= Duration::from_secs_f64(0.0);
        }

        if number_of_steps_f64 < 1.0 {
            //Can't do a full time step
            //we can either do an update with smaller step or wait next iteration
            //Other option is to update only actuator position based on known hydraulic
            //state to avoid lag of control surfaces if sim runs really fast
            self.lag_time_accumulator=Duration::from_secs_f64(number_of_steps_f64 * min_hyd_loop_timestep.as_secs_f64()); //Time lag is float part of num of steps * fixed time step to get a result in time
        } else {
            //TRUE UPDATE LOOP HERE
            let num_of_update_loops = number_of_steps_f64.floor() as u32; //Int part is the actual number of loops to do
            //Rest of floating part goes into accumulator
            self.lag_time_accumulator= Duration::from_secs_f64((number_of_steps_f64 - (num_of_update_loops as f64))* min_hyd_loop_timestep.as_secs_f64()); //Keep track of time left after all fixed loop are done


            //UPDATING HYDRAULICS AT FIXED STEP
            for curLoop in  0..num_of_update_loops {
                //UPDATE HYDRAULICS FIXED TIME STEP
                self.ptu.update(&self.green_loop, &self.yellow_loop);
                self.engine_driven_pump_1.update(&min_hyd_loop_timestep,&ct, &self.green_loop, &engine1);
                self.engine_driven_pump_2.update(&min_hyd_loop_timestep,&ct, &self.yellow_loop, &engine2);
                self.yellow_electric_pump.update(&min_hyd_loop_timestep,&ct, &self.yellow_loop);
                self.blue_electric_pump.update(&min_hyd_loop_timestep,&ct, &self.blue_loop);


                self.green_loop.update(&min_hyd_loop_timestep,&ct, Vec::new(), vec![&self.engine_driven_pump_1], Vec::new(), vec![&self.ptu]);
                self.yellow_loop.update(&min_hyd_loop_timestep,&ct, vec![&self.yellow_electric_pump], vec![&self.engine_driven_pump_2], Vec::new(), vec![&self.ptu]);
                self.blue_loop.update(&min_hyd_loop_timestep,&ct, vec![&self.blue_electric_pump], Vec::new(), Vec::new(), Vec::new());
            }

            //UPDATING ACTUATOR PHYSICS AT FIXED STEP / ACTUATORS_SIM_TIME_STEP_MULT
            let num_of_actuators_update_loops = num_of_update_loops * A320Hydraulic::ACTUATORS_SIM_TIME_STEP_MULT;
            for curLoop in  0..num_of_actuators_update_loops {
                //UPDATE ACTUATORS FIXED TIME STEP
            }
        }
    }
}

pub struct A320HydraulicOverheadPanel {
    pub edp1_push_button: AutoOffFaultPushButton,
    pub edp2_push_button: AutoOffFaultPushButton,
    pub blue_epump_push_button: AutoOffFaultPushButton,
    pub ptu_push_button: AutoOffFaultPushButton,
    pub rat_push_button: OnOffFaultPushButton,
    pub yellow_epump_push_button: OnOffFaultPushButton,
}

impl A320HydraulicOverheadPanel {
    pub fn new() -> A320HydraulicOverheadPanel {
        A320HydraulicOverheadPanel {
            edp1_push_button: AutoOffFaultPushButton::new_auto("HYD_EDP1_SW"),
            edp2_push_button: AutoOffFaultPushButton::new_auto("HYD_EDP2_SW"),
            blue_epump_push_button : AutoOffFaultPushButton::new_auto("HYD_BLUE_EPUMP_SW"),
            ptu_push_button : AutoOffFaultPushButton::new_auto("HYD_PTU_SW"),
            rat_push_button : OnOffFaultPushButton::new_off("HYD_RAT_SW"),
            yellow_epump_push_button :OnOffFaultPushButton::new_off("HYD_YELL_EPUMP_SW"),
        }
    }

    pub fn update(&mut self, context: &UpdateContext) {
    }
}

impl SimulatorElementVisitable for A320HydraulicOverheadPanel {
    fn accept(&mut self, visitor: &mut Box<&mut dyn SimulatorElementVisitor>) {
        self.edp1_push_button.accept(visitor);
        self.edp2_push_button.accept(visitor);
        self.blue_epump_push_button.accept(visitor);
        self.ptu_push_button.accept(visitor);
        self.rat_push_button.accept(visitor);
        self.yellow_epump_push_button.accept(visitor);

        visitor.visit(&mut Box::new(self));
    }
}
impl SimulatorElement for A320HydraulicOverheadPanel {}
