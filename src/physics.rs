use std::ops::{Add, Sub, Mul, Div};

pub const G: f32 = 100.0; // Gravitational constant

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub fn new(x: f32, y: f32) -> Self {
        Vec2 { x, y }
    }
    
    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
    
    pub fn normalize(&self) -> Self {
        let len = self.length();
        if len != 0.0 {
            Self { x: self.x / len, y: self.y / len }
        } else {
            Self { x: 0.0, y: 0.0 }
        }
    }
    
    pub fn distance(&self, other: &Self) -> f32 {
        (*self - *other).length()
    }
}

impl Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self { x: self.x + rhs.x, y: self.y + rhs.y }
    }
}

impl Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self { x: self.x - rhs.x, y: self.y - rhs.y }
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        Self { x: self.x * rhs, y: self.y * rhs }
    }
}

impl Div<f32> for Vec2 {
    type Output = Self;
    fn div(self, rhs: f32) -> Self::Output {
        Self { x: self.x / rhs, y: self.y / rhs }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct StationaryBody {
    pub pos: Vec2,
    pub mass: f32,
    pub radius: f32,
    pub color: [u8; 3],
}

impl StationaryBody {
    pub fn new(pos: Vec2, mass: f32, radius: f32, color: [u8; 3]) -> Self {
        Self { pos, mass, radius, color }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TestParticle {
    pub pos: Vec2,
    pub vel: Vec2,
    pub mass: f32,
    pub radius: f32,
}

impl TestParticle {
    pub fn new(pos: Vec2, vel: Vec2, mass: f32, radius: f32) -> Self {
        Self { pos, vel, mass, radius }
    }
}

pub fn calculate_gravitational_force(particle: &TestParticle, stationary_body: &StationaryBody) -> Vec2 {
    let dir = stationary_body.pos - particle.pos;
    let dist = dir.length();
    if dist == 0.0 {
        return Vec2::new(0.0, 0.0);
    }
    let force_mag = G * particle.mass * stationary_body.mass / (dist * dist);
    dir.normalize() * force_mag
}

pub fn calculate_acceleration(particle: &TestParticle, stationary_bodies: &[StationaryBody]) -> Vec2 {
    let mut total_force = Vec2::new(0.0, 0.0);
    for body in stationary_bodies {
        total_force = total_force + calculate_gravitational_force(particle, body);
    }
    total_force / particle.mass
}

// Simple Euler integration
pub fn update_particle_euler(particle: &mut TestParticle, stationary_bodies: &[StationaryBody], dt: f32) {
    let acceleration = calculate_acceleration(particle, stationary_bodies);
    particle.vel = particle.vel + acceleration * dt;
    particle.pos = particle.pos + particle.vel * dt;
}

// Runge-Kutta 4th order integration for better accuracy
pub fn update_particle_rk4(particle: &mut TestParticle, stationary_bodies: &[StationaryBody], dt: f32) {
    let original_particle = *particle;
    
    // k1
    let k1_vel = calculate_acceleration(&original_particle, stationary_bodies) * dt;
    let k1_pos = original_particle.vel * dt;
    
    // k2
    let mut temp_particle = original_particle;
    temp_particle.pos = temp_particle.pos + k1_pos * 0.5;
    temp_particle.vel = temp_particle.vel + k1_vel * 0.5;
    let k2_vel = calculate_acceleration(&temp_particle, stationary_bodies) * dt;
    let k2_pos = temp_particle.vel * dt;
    
    // k3
    temp_particle = original_particle;
    temp_particle.pos = temp_particle.pos + k2_pos * 0.5;
    temp_particle.vel = temp_particle.vel + k2_vel * 0.5;
    let k3_vel = calculate_acceleration(&temp_particle, stationary_bodies) * dt;
    let k3_pos = temp_particle.vel * dt;
    
    // k4
    temp_particle = original_particle;
    temp_particle.pos = temp_particle.pos + k3_pos;
    temp_particle.vel = temp_particle.vel + k3_vel;
    let k4_vel = calculate_acceleration(&temp_particle, stationary_bodies) * dt;
    let k4_pos = temp_particle.vel * dt;
    
    // Final update
    particle.vel = particle.vel + (k1_vel + k2_vel * 2.0 + k3_vel * 2.0 + k4_vel) / 6.0;
    particle.pos = particle.pos + (k1_pos + k2_pos * 2.0 + k3_pos * 2.0 + k4_pos) / 6.0;
}

pub fn check_collision(particle: &TestParticle, stationary_bodies: &[StationaryBody], collision_threshold: f32) -> Option<usize> {
    for (i, body) in stationary_bodies.iter().enumerate() {
        if particle.pos.distance(&body.pos) < collision_threshold {
            return Some(i);
        }
    }
    None
}
