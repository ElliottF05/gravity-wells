use image::{ImageBuffer, Rgb};
use rayon::prelude::*;
use indicatif::ProgressBar;
use std::sync::atomic::{AtomicUsize, Ordering};

use crate::physics::{Vec2, StationaryBody};
use crate::simulation::{run_simulation_with_time, IntegrationMethod};
use crate::config::{IMAGE_SIZE, DEFAULT_NON_COLLISION_COLOR};

pub fn generate_gravity_wells_image(
    stationary_bodies: &[StationaryBody], 
    initial_velocity: Vec2,
    camera_offset: Vec2,
    zoom_factor: f32,
    filename: &str,
    integration_method: IntegrationMethod
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Generating gravity wells image using {:?} integration...", 
             match integration_method {
                 IntegrationMethod::Euler => "Euler",
                 IntegrationMethod::RungeKutta4 => "Runge-Kutta 4",
             });
    
    let num_pixels = (IMAGE_SIZE * IMAGE_SIZE) as usize;
    let mut pixels = vec![Rgb(DEFAULT_NON_COLLISION_COLOR); num_pixels];
    
    let bar = ProgressBar::new(num_pixels as u64);
    let counter = AtomicUsize::new(0);
    
    pixels.par_iter_mut().enumerate().for_each(|(i, pixel)| {
        let px = (i % IMAGE_SIZE as usize) as u32;
        let py = (i / IMAGE_SIZE as usize) as u32;

        // Transform pixel coordinates to world coordinates accounting for camera and zoom
        let world_pos = Vec2::new(
            (px as f32) / zoom_factor - camera_offset.x,
            (py as f32) / zoom_factor - camera_offset.y
        );
        
        if let Some((collision_index, collision_time)) = run_simulation_with_time(world_pos, initial_velocity, stationary_bodies, integration_method) {
            // Color with intensity based on collision time
            let body_color = stationary_bodies[collision_index].color;
            
            // Calculate intensity: 1.0 for immediate collision, fading to 0.0 for max timesteps
            let max_time = crate::simulation::SIMULATION_TIMESTEPS as f32;
            let intensity = (1.0 - (collision_time as f32 / max_time)).max(0.0);
            
            // Apply intensity to the body's color, with minimum intensity to keep it visible
            let min_intensity = 0.15; // Minimum visibility
            let max_intensity = 0.85; // Maximum intensity for pixels (less than full)
            let final_intensity = intensity * (max_intensity - min_intensity) + min_intensity;
            
            let colored_pixel = [
                (body_color[0] as f32 * final_intensity) as u8,
                (body_color[1] as f32 * final_intensity) as u8,
                (body_color[2] as f32 * final_intensity) as u8,
            ];
            *pixel = Rgb(colored_pixel);
        }
        // Otherwise keep default dark color
        
        // Update progress bar occasionally
        let count = counter.fetch_add(1, Ordering::Relaxed);
        if count % 1000 == 0 {
            bar.set_position(count as u64);
        }
    });
    
    bar.finish();

    let mut img = ImageBuffer::new(IMAGE_SIZE, IMAGE_SIZE);
    for (px, py, pixel) in img.enumerate_pixels_mut() {
        *pixel = pixels[(py * IMAGE_SIZE + px) as usize];
    }

    img.save(filename)?;
    println!("Gravity wells image saved to {}", filename);
    
    Ok(())
}
