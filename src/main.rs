mod physics;
mod simulation;
mod config;
mod image_gen;

use macroquad::prelude::*;
use physics::Vec2;
use simulation::{LiveSimulationState, IntegrationMethod};
use config::{IMAGE_SIZE, create_stationary_bodies};
use image_gen::generate_gravity_wells_image;

fn window_conf() -> Conf {
    Conf {
        window_title: "Gravity Wells Visualization".to_owned(),
        window_width: IMAGE_SIZE as i32,
        window_height: IMAGE_SIZE as i32,
        window_resizable: false,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    // Create stationary bodies from configuration
    let stationary_bodies = create_stationary_bodies();
    let mut use_runge_kutta = true; // Default to RK4 for better accuracy
    let mut initial_velocity = Vec2::new(0.0, 0.0); // Starting with zero velocity
    let mut step_size = 2.0; // Configurable step size for velocity and camera movement
    let mut camera_offset = Vec2::new(0.0, 0.0); // Camera position offset
    let mut zoom_factor = 1.0; // Zoom level
    let mut needs_recalculation = true; // Flag to track when we need to regenerate image

    // Helper function to get current image filename
    let get_image_filename = |use_rk4: bool, vel: Vec2, cam: Vec2, zoom: f32| -> String {
        format!("gravity_wells_{}_{:.1}_{:.1}_{:.1}_{:.1}_{:.2}.png", 
                if use_rk4 { "rk4" } else { "euler" },
                vel.x, vel.y, cam.x, cam.y, zoom)
    };

    let mut current_image_filename = get_image_filename(use_runge_kutta, initial_velocity, camera_offset, zoom_factor);
    let mut texture_option: Option<Texture2D> = None;
    let mut live_simulation: Option<LiveSimulationState> = None;
    let mut selected_px: Option<u32> = None;
    let mut selected_py: Option<u32> = None;

    loop {
        clear_background(BLACK);

        // Handle step size changes
        if is_key_pressed(KeyCode::Equal) || is_key_pressed(KeyCode::KpAdd) {
            step_size = (step_size * 1.2_f32).min(50.0); // Increase by 20%, max 50
        }
        if is_key_pressed(KeyCode::Minus) || is_key_pressed(KeyCode::KpSubtract) {
            step_size = (step_size * 0.8_f32).max(0.1); // Decrease by 20%, min 0.1
        }

        // Handle camera movement with WASD
        let mut camera_changed = false;
        if is_key_down(KeyCode::W) {
            camera_offset.y -= step_size;
            camera_changed = true;
        }
        if is_key_down(KeyCode::S) {
            camera_offset.y += step_size;
            camera_changed = true;
        }
        if is_key_down(KeyCode::A) {
            camera_offset.x -= step_size;
            camera_changed = true;
        }
        if is_key_down(KeyCode::D) {
            camera_offset.x += step_size;
            camera_changed = true;
        }

        // Handle zoom with Q and E
        if is_key_down(KeyCode::Q) {
            zoom_factor = (zoom_factor * 0.99).max(0.1); // Zoom out, min 0.1x
            camera_changed = true;
        }
        if is_key_down(KeyCode::E) {
            zoom_factor = (zoom_factor * 1.01).min(10.0); // Zoom in, max 10x
            camera_changed = true;
        }

        // Handle key presses for changing initial velocity
        let mut velocity_changed = false;
        if is_key_pressed(KeyCode::Up) {
            initial_velocity.y -= step_size;
            velocity_changed = true;
        }
        if is_key_pressed(KeyCode::Down) {
            initial_velocity.y += step_size;
            velocity_changed = true;
        }
        if is_key_pressed(KeyCode::Left) {
            initial_velocity.x -= step_size;
            velocity_changed = true;
        }
        if is_key_pressed(KeyCode::Right) {
            initial_velocity.x += step_size;
            velocity_changed = true;
        }

        // Handle key presses for switching integration method
        if is_key_pressed(KeyCode::Space) {
            use_runge_kutta = !use_runge_kutta;
            velocity_changed = true; // This will also trigger recalculation
        }

        // Mark for recalculation if any parameters changed
        if velocity_changed || camera_changed {
            needs_recalculation = true;
            // Clear current simulation when parameters change
            live_simulation = None;
            selected_px = None;
            selected_py = None;
        }

        // Handle Enter key for manual recalculation
        if is_key_pressed(KeyCode::Enter) && needs_recalculation {
            let new_image_filename = get_image_filename(use_runge_kutta, initial_velocity, camera_offset, zoom_factor);
            
            // Only regenerate if this specific configuration doesn't exist
            if !std::path::Path::new(&new_image_filename).exists() {
                let integration_method = if use_runge_kutta {
                    IntegrationMethod::RungeKutta4
                } else {
                    IntegrationMethod::Euler
                };
                
                println!("Generating new image with velocity ({:.1}, {:.1}), camera ({:.1}, {:.1}), zoom {:.2}...", 
                        initial_velocity.x, initial_velocity.y, camera_offset.x, camera_offset.y, zoom_factor);
                if let Err(e) = generate_gravity_wells_image(&stationary_bodies, initial_velocity, camera_offset, zoom_factor, &new_image_filename, integration_method) {
                    eprintln!("Error generating image: {}", e);
                    next_frame().await;
                    continue;
                }
            } else {
                println!("Found existing image: {}", new_image_filename);
            }
            
            // Load the new texture
            current_image_filename = new_image_filename;
            texture_option = Some(load_texture(&current_image_filename).await.unwrap());
            needs_recalculation = false;
        }

        // Load initial texture if none exists
        if texture_option.is_none() && !needs_recalculation {
            if std::path::Path::new(&current_image_filename).exists() {
                texture_option = Some(load_texture(&current_image_filename).await.unwrap());
            } else {
                needs_recalculation = true;
            }
        }

        // Draw gravity wells image if available
        if let Some(texture) = &texture_option {
            draw_texture(texture, 0.0, 0.0, WHITE);

            // Transform stationary bodies for camera and zoom
            let transform_point = |p: Vec2| -> Vec2 {
                Vec2::new(
                    (p.x + camera_offset.x) * zoom_factor,
                    (p.y + camera_offset.y) * zoom_factor
                )
            };

            // Draw stationary bodies on top with camera transformation
            for body in &stationary_bodies {
                let transformed_pos = transform_point(body.pos);
                if transformed_pos.x >= -body.radius && transformed_pos.x < IMAGE_SIZE as f32 + body.radius &&
                   transformed_pos.y >= -body.radius && transformed_pos.y < IMAGE_SIZE as f32 + body.radius {
                    let color = Color::from_rgba(body.color[0], body.color[1], body.color[2], 255);
                    
                    // Draw a dark outline first for better visibility
                    draw_circle(transformed_pos.x, transformed_pos.y, body.radius * zoom_factor + 2.0, BLACK);
                    
                    // Draw the body with full color intensity
                    draw_circle(transformed_pos.x, transformed_pos.y, body.radius * zoom_factor, color);
                }
            }

            // Highlight selected pixel if any
            if let (Some(px), Some(py)) = (selected_px, selected_py) {
                let highlight_size = 4.0;
                let x = px as f32 - highlight_size / 2.0;
                let y = py as f32 - highlight_size / 2.0;
                draw_rectangle_lines(x, y, highlight_size, highlight_size, 2.0, WHITE);
            }

            // Handle mouse clicks (account for camera transformation)
            if is_mouse_button_pressed(MouseButton::Left) {
                let (mx, my) = mouse_position();
                if mx >= 0.0 && mx < IMAGE_SIZE as f32 && my >= 0.0 && my < IMAGE_SIZE as f32 {
                    let px = mx as u32;
                    let py = my as u32;
                    selected_px = Some(px);
                    selected_py = Some(py);

                    // Transform mouse position back to world coordinates
                    let world_pos = Vec2::new(
                        mx / zoom_factor - camera_offset.x,
                        my / zoom_factor - camera_offset.y
                    );
                    
                    let integration_method = if use_runge_kutta {
                        IntegrationMethod::RungeKutta4
                    } else {
                        IntegrationMethod::Euler
                    };
                    
                    live_simulation = Some(LiveSimulationState::new(
                        world_pos,
                        initial_velocity,
                        stationary_bodies.clone(),
                        integration_method,
                    ));
                }
            }

            // Update and draw live simulation
            if let Some(sim) = &mut live_simulation {
                // if !sim.is_finished() {
                //     sim.step();
                // }
                sim.step();

                // Draw trajectory with camera transformation
                for i in 1..sim.trajectory_history.len() {
                    let p1 = transform_point(sim.trajectory_history[i - 1]);
                    let p2 = transform_point(sim.trajectory_history[i]);
                    
                    // Only draw if both points are visible
                    if p1.x >= -50.0 && p1.x < IMAGE_SIZE as f32 + 50.0 &&
                       p1.y >= -50.0 && p1.y < IMAGE_SIZE as f32 + 50.0 &&
                       p2.x >= -50.0 && p2.x < IMAGE_SIZE as f32 + 50.0 &&
                       p2.y >= -50.0 && p2.y < IMAGE_SIZE as f32 + 50.0 {
                        draw_line(p1.x, p1.y, p2.x, p2.y, 2.0, YELLOW);
                    }
                }

                // Draw current particle position with camera transformation
                let particle_pos = transform_point(sim.particle.pos);
                if particle_pos.x >= 0.0 && particle_pos.x < IMAGE_SIZE as f32 &&
                   particle_pos.y >= 0.0 && particle_pos.y < IMAGE_SIZE as f32 {
                    let particle_color = if sim.collision_body_index.is_some() {
                        RED
                    } else {
                        YELLOW
                    };
                    draw_circle(particle_pos.x, particle_pos.y, 3.0, particle_color);
                }
            }
        } else {
            // Show message when no image is loaded
            draw_text("Press ENTER to generate gravity wells image", 10.0, IMAGE_SIZE as f32 / 2.0, 24.0, WHITE);
        }

        // Draw instructions and status
        let integration_name = if use_runge_kutta { "Runge-Kutta 4" } else { "Euler" };
        let mut y_offset = 20.0;
        
        draw_text("Controls:", 10.0, y_offset, 18.0, WHITE);
        y_offset += 20.0;
        
        draw_text("• Arrow keys: Change initial velocity", 10.0, y_offset, 14.0, WHITE);
        y_offset += 16.0;
        draw_text("• WASD: Move camera", 10.0, y_offset, 14.0, WHITE);
        y_offset += 16.0;
        draw_text("• Q/E: Zoom out/in", 10.0, y_offset, 14.0, WHITE);
        y_offset += 16.0;
        draw_text("• +/-: Change step size", 10.0, y_offset, 14.0, WHITE);
        y_offset += 16.0;
        draw_text("• Space: Toggle integration method", 10.0, y_offset, 14.0, WHITE);
        y_offset += 16.0;
        draw_text("• Enter: Recalculate image", 10.0, y_offset, 14.0, WHITE);
        y_offset += 16.0;
        draw_text("• Left click: Start simulation", 10.0, y_offset, 14.0, WHITE);
        y_offset += 20.0;
        
        // Show current settings
        draw_text(&format!("Step Size: {:.1}", step_size), 10.0, y_offset, 16.0, SKYBLUE);
        y_offset += 18.0;
        draw_text(&format!("Initial Velocity: ({:.1}, {:.1})", initial_velocity.x, initial_velocity.y), 10.0, y_offset, 16.0, SKYBLUE);
        y_offset += 18.0;
        draw_text(&format!("Camera: ({:.1}, {:.1})", camera_offset.x, camera_offset.y), 10.0, y_offset, 16.0, SKYBLUE);
        y_offset += 18.0;
        draw_text(&format!("Zoom: {:.2}x", zoom_factor), 10.0, y_offset, 16.0, SKYBLUE);
        y_offset += 18.0;
        draw_text(&format!("Integration: {}", integration_name), 10.0, y_offset, 16.0, SKYBLUE);
        y_offset += 18.0;
        
        if needs_recalculation {
            draw_text("Parameters changed - press ENTER to recalculate", 10.0, y_offset, 16.0, YELLOW);
            y_offset += 18.0;
        }
        
        if let Some(sim) = &live_simulation {
            let status = if let Some(collision_index) = sim.collision_body_index {
                format!("Collided with body {} at timestep {}", collision_index, sim.current_timestep)
            } else if sim.current_timestep >= simulation::SIMULATION_TIMESTEPS {
                "No collision - simulation ended".to_string()
            } else {
                format!("Simulating... timestep {}", sim.current_timestep)
            };
            draw_text(&status, 10.0, y_offset, 16.0, WHITE);
        }

        next_frame().await;
    }
}
