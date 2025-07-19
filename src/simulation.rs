use crate::physics::*;

pub const SIMULATION_TIMESTEPS: usize = 2000;
pub const SUBSTEPS: usize = 10;
pub const TEST_PARTICLE_MASS: f32 = 1.0;
pub const TEST_PARTICLE_RADIUS: f32 = 1.0;
pub const COLLISION_THRESHOLD: f32 = 15.0;

#[derive(Clone, Copy, Debug)]
pub enum IntegrationMethod {
    Euler,
    RungeKutta4,
}

pub struct LiveSimulationState {
    pub particle: TestParticle,
    pub stationary_bodies: Vec<StationaryBody>,
    pub trajectory_history: Vec<Vec2>,
    pub current_timestep: usize,
    pub collision_body_index: Option<usize>,
    pub integration_method: IntegrationMethod,
}

impl LiveSimulationState {
    pub fn new(
        start_pos: Vec2, 
        initial_velocity: Vec2,
        stationary_bodies: Vec<StationaryBody>,
        integration_method: IntegrationMethod
    ) -> Self {
        let particle = TestParticle::new(start_pos, initial_velocity, TEST_PARTICLE_MASS, TEST_PARTICLE_RADIUS);
        Self {
            particle,
            stationary_bodies,
            trajectory_history: vec![start_pos],
            current_timestep: 0,
            collision_body_index: None,
            integration_method,
        }
    }
    
    pub fn step(&mut self) {
        if self.current_timestep < SIMULATION_TIMESTEPS && self.collision_body_index.is_none() {
            let dt = 0.016 / SUBSTEPS as f32;
            for _ in 0..SUBSTEPS {
                match self.integration_method {
                    IntegrationMethod::Euler => {
                        update_particle_euler(&mut self.particle, &self.stationary_bodies, dt);
                    }
                    IntegrationMethod::RungeKutta4 => {
                        update_particle_rk4(&mut self.particle, &self.stationary_bodies, dt);
                    }
                }
                
                if let Some(collision_index) = check_collision(&self.particle, &self.stationary_bodies, COLLISION_THRESHOLD) {
                    self.collision_body_index = Some(collision_index);
                    break;
                }
            }
            
            // Record trajectory position every few steps for visualization
            if self.current_timestep % 5 == 0 {
                self.trajectory_history.push(self.particle.pos);
            }
            self.current_timestep += 1;
        }
    }
    
    pub fn is_finished(&self) -> bool {
        self.collision_body_index.is_some() || self.current_timestep >= SIMULATION_TIMESTEPS
    }
}

pub fn run_simulation(
    start_pos: Vec2, 
    initial_velocity: Vec2,
    stationary_bodies: &[StationaryBody], 
    integration_method: IntegrationMethod
) -> Option<usize> {
    let mut particle = TestParticle::new(start_pos, initial_velocity, TEST_PARTICLE_MASS, TEST_PARTICLE_RADIUS);
    let dt = 0.016 / SUBSTEPS as f32;

    for _ in 0..SIMULATION_TIMESTEPS {
        for _ in 0..SUBSTEPS {
            match integration_method {
                IntegrationMethod::Euler => {
                    update_particle_euler(&mut particle, stationary_bodies, dt);
                }
                IntegrationMethod::RungeKutta4 => {
                    update_particle_rk4(&mut particle, stationary_bodies, dt);
                }
            }
            
            if let Some(collision_index) = check_collision(&particle, stationary_bodies, COLLISION_THRESHOLD) {
                return Some(collision_index);
            }
        }
    }
    None // No collision
}

pub fn run_simulation_with_time(
    start_pos: Vec2, 
    initial_velocity: Vec2,
    stationary_bodies: &[StationaryBody], 
    integration_method: IntegrationMethod
) -> Option<(usize, usize)> {
    let mut particle = TestParticle::new(start_pos, initial_velocity, TEST_PARTICLE_MASS, TEST_PARTICLE_RADIUS);
    let dt = 0.016 / SUBSTEPS as f32;

    for timestep in 0..SIMULATION_TIMESTEPS {
        for _ in 0..SUBSTEPS {
            match integration_method {
                IntegrationMethod::Euler => {
                    update_particle_euler(&mut particle, stationary_bodies, dt);
                }
                IntegrationMethod::RungeKutta4 => {
                    update_particle_rk4(&mut particle, stationary_bodies, dt);
                }
            }
            
            if let Some(collision_index) = check_collision(&particle, stationary_bodies, COLLISION_THRESHOLD) {
                return Some((collision_index, timestep));
            }
        }
    }
    None // No collision
}
