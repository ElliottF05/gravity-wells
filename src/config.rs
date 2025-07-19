use crate::physics::{Vec2, StationaryBody};

pub const IMAGE_SIZE: u32 = 600;
pub const DEFAULT_NON_COLLISION_COLOR: [u8; 3] = [20, 20, 20]; // Dark gray

// Configuration for the gravity wells
pub const STATIONARY_BODIES_CONFIG: &[(f32, f32, f32, [u8; 3])] = &[
    // (x, y, mass, color_rgb)
    (150.0, 150.0, 50000.0, [255, 100, 100]), // Red well
    (450.0, 150.0, 30000.0, [100, 255, 100]), // Green well
    (300.0, 400.0, 40000.0, [100, 100, 255]), // Blue well
];

pub fn create_stationary_bodies() -> Vec<StationaryBody> {
    STATIONARY_BODIES_CONFIG
        .iter()
        .map(|(x, y, mass, color)| {
            StationaryBody::new(
                Vec2::new(*x, *y),
                *mass,
                (*mass / 1000.0).sqrt().max(10.0), // Radius based on mass
                *color,
            )
        })
        .collect()
}
