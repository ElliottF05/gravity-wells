use macroquad::prelude::*;
use std::ops::{Add, Sub, Mul, Div};

pub const G: f32 = 1.0;

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
    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y
    }
    pub fn distance(&self, other: &Self) -> f32 {
        (*self - *other).length()
    }
    pub fn lerp(&self, other: &Self, t: f32) -> Self {
        Self {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
        }
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
pub struct GravBody {
    pub pos: Vec2,
    pub vel: Vec2,
    pub mass: f32,
    pub radius: f32,
}

impl GravBody {
    pub fn new(pos: Vec2, vel: Vec2, mass: f32, radius: f32) -> Self {
        Self { pos, vel, mass, radius }
    }

    pub fn grav_force(&self, other: &Self) -> Vec2 {
        let dir = other.pos - self.pos;
        let dist = dir.length();
        if dist == 0.0 {
            return Vec2 { x: 0.0, y: 0.0 };
        }
        let force_mag = G * self.mass * other.mass / (dist * dist);
        dir.normalize() * force_mag
    }

    pub fn draw(&self, center: Vec2, zoom: f32, color: Color) {
        let screen_pos = (self.pos - center) * zoom + Vec2::new(screen_width() / 2.0, screen_height() / 2.0);
        draw_circle(screen_pos.x, screen_pos.y, self.radius * zoom, color);
    }
}

/// Calculates the acceleration for each body based on the current state of all bodies.
fn get_accelerations(bodies: &[GravBody]) -> Vec<Vec2> {
    let mut accels = vec![Vec2::new(0.0, 0.0); bodies.len()];
    for i in 0..bodies.len() {
        let mut total_force = Vec2::new(0.0, 0.0);
        for j in 0..bodies.len() {
            if i != j {
                total_force = total_force + bodies[i].grav_force(&bodies[j]);
            }
        }
        accels[i] = total_force / bodies[i].mass;
    }
    accels
}

/// Updates all bodies in the simulation for a single timestep using a system-aware RK4 integrator.
pub fn update_bodies(bodies: &mut [GravBody], dt: f32) {
    let n = bodies.len();
    let mut k1_vel = vec![Vec2::new(0.0, 0.0); n];
    let mut k1_pos = vec![Vec2::new(0.0, 0.0); n];
    let mut k2_vel = vec![Vec2::new(0.0, 0.0); n];
    let mut k2_pos = vec![Vec2::new(0.0, 0.0); n];
    let mut k3_vel = vec![Vec2::new(0.0, 0.0); n];
    let mut k3_pos = vec![Vec2::new(0.0, 0.0); n];
    let mut k4_vel = vec![Vec2::new(0.0, 0.0); n];
    let mut k4_pos = vec![Vec2::new(0.0, 0.0); n];

    let original_bodies = bodies.to_vec();
    let mut temp_bodies = bodies.to_vec();

    // k1
    let accels = get_accelerations(&original_bodies);
    for i in 0..n {
        k1_vel[i] = accels[i] * dt;
        k1_pos[i] = original_bodies[i].vel * dt;
        temp_bodies[i].pos = original_bodies[i].pos + k1_pos[i] * 0.5;
        temp_bodies[i].vel = original_bodies[i].vel + k1_vel[i] * 0.5;
    }

    // k2
    let accels = get_accelerations(&temp_bodies);
    for i in 0..n {
        k2_vel[i] = accels[i] * dt;
        k2_pos[i] = temp_bodies[i].vel * dt;
        temp_bodies[i].pos = original_bodies[i].pos + k2_pos[i] * 0.5;
        temp_bodies[i].vel = original_bodies[i].vel + k2_vel[i] * 0.5;
    }

    // k3
    let accels = get_accelerations(&temp_bodies);
    for i in 0..n {
        k3_vel[i] = accels[i] * dt;
        k3_pos[i] = temp_bodies[i].vel * dt;
        temp_bodies[i].pos = original_bodies[i].pos + k3_pos[i];
        temp_bodies[i].vel = original_bodies[i].vel + k3_vel[i];
    }

    // k4
    let accels = get_accelerations(&temp_bodies);
    for i in 0..n {
        k4_vel[i] = accels[i] * dt;
        k4_pos[i] = temp_bodies[i].vel * dt;
    }

    // Final update
    for i in 0..n {
        bodies[i].pos = original_bodies[i].pos + (k1_pos[i] + k2_pos[i] * 2.0 + k3_pos[i] * 2.0 + k4_pos[i]) / 6.0;
        bodies[i].vel = original_bodies[i].vel + (k1_vel[i] + k2_vel[i] * 2.0 + k3_vel[i] * 2.0 + k4_vel[i]) / 6.0;
    }
}