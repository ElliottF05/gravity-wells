use macroquad::prelude::*;
use image::{ImageBuffer, Rgb};
use three_body_problem::{update_bodies, GravBody, Vec2};
use rayon::prelude::*;
use indicatif::{ParallelProgressIterator, ProgressBar};

const IMAGE_SIZE: u32 = 400;
const SIMULATION_TIMESTEPS: usize = 1000;
const SUBSTEPS: usize = 50;
const POSITION_SAMPLE_RATE: usize = 1; // Draw trajectory every N steps
const VY1_MIN: f32 = 0.0;
const VY1_MAX: f32 = 80.0;
const VY2_MIN: f32 = -80.0;
const VY2_MAX: f32 = 0.0;

struct LiveSimulationState {
    bodies: [GravBody; 3],
    trajectory_history: [Vec<Vec2>; 3],
    current_timestep: usize,
}

fn to_macroquad_vec2(v: Vec2) -> macroquad::math::Vec2 {
    macroquad::math::Vec2::new(v.x, v.y)
}

#[macroquad::main("Stability Explorer")]
async fn main() {
    // 1. Generate stability image if it doesn't exist
    if !std::path::Path::new("stability.png").exists() {
        println!("Generating stability image with Rayon...");
        
        let num_pixels = (IMAGE_SIZE * IMAGE_SIZE) as usize;
        let mut pixels = vec![Rgb([0, 0, 0]); num_pixels];
        
        let bar = ProgressBar::new(num_pixels as u64);
        
        pixels.par_iter_mut().progress_with(bar).enumerate().for_each(|(i, pixel)| {
            let px = (i % IMAGE_SIZE as usize) as u32;
            let py = (i / IMAGE_SIZE as usize) as u32;

            let vy1 = VY1_MIN + (VY1_MAX - VY1_MIN) * (px as f32 / (IMAGE_SIZE - 1) as f32);
            let vy2 = VY2_MIN + (VY2_MAX - VY2_MIN) * (py as f32 / (IMAGE_SIZE - 1) as f32);

            let mut bodies = [
                GravBody::new(Vec2::new(50.0, 0.0), Vec2::new(0.0, vy1), 160000.0, 1.0),
                GravBody::new(Vec2::new(-50.0, 0.0), Vec2::new(0.0, vy2), 160000.0, 1.0),
                GravBody::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 160000.0, 1.0),
            ];

            let time_to_collision = run_simulation(&mut bodies);
            let normalized_time = time_to_collision / SIMULATION_TIMESTEPS as f32;
            let color_val = (normalized_time.min(1.0) * 255.0) as u8;
            *pixel = Rgb([color_val, color_val, color_val]);
        });

        let mut img = ImageBuffer::new(IMAGE_SIZE, IMAGE_SIZE);
        for (px, py, pixel) in img.enumerate_pixels_mut() {
            *pixel = pixels[(py * IMAGE_SIZE + px) as usize];
        }

        img.save("stability.png").unwrap();
        println!("Stability image saved to stability.png");
    } else {
        println!("Found existing stability.png, skipping generation.");
    }


    // 2. Run interactive explorer
    let texture = load_texture("stability.png").await.unwrap();
    let mut live_simulation: Option<LiveSimulationState> = None;
    let mut selected_px: Option<u32> = None;
    let mut selected_py: Option<u32> = None;

    loop {
        clear_background(BLACK);

        // Draw stability image
        draw_texture(texture, 0.0, 0.0, WHITE);

        // Highlight selected pixel if any
        if let (Some(px), Some(py)) = (selected_px, selected_py) {
            let highlight_size = 4.0;
            let x = px as f32 - highlight_size / 2.0;
            let y = py as f32 - highlight_size / 2.0;
            draw_rectangle_lines(x, y, highlight_size, highlight_size, 2.0, RED);
        }

        // Draw trajectory window
        draw_rectangle(IMAGE_SIZE as f32, 0.0, IMAGE_SIZE as f32, IMAGE_SIZE as f32, DARKGRAY);

        if is_mouse_button_pressed(MouseButton::Left) {
            let (mx, my) = mouse_position();
            if mx >= 0.0 && mx < IMAGE_SIZE as f32 && my >= 0.0 && my < IMAGE_SIZE as f32 {
                let px = mx as u32;
                let py = my as u32;
                selected_px = Some(px);
                selected_py = Some(py);

                let vy1 = VY1_MIN + (VY1_MAX - VY1_MIN) * (px as f32 / (IMAGE_SIZE - 1) as f32);
                let vy2 = VY2_MIN + (VY2_MAX - VY2_MIN) * (py as f32 / (IMAGE_SIZE - 1) as f32);
                let bodies = [
                    GravBody::new(Vec2::new(50.0, 0.0), Vec2::new(0.0, vy1), 160000.0, 1.0),
                    GravBody::new(Vec2::new(-50.0, 0.0), Vec2::new(0.0, vy2), 160000.0, 1.0),
                    GravBody::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 160000.0, 1.0),
                ];
                live_simulation = Some(LiveSimulationState {
                    trajectory_history: [
                        vec![bodies[0].pos],
                        vec![bodies[1].pos],
                        vec![bodies[2].pos],
                    ],
                    bodies,
                    current_timestep: 0,
                });
            }
        }

        // Update and draw live simulation
        if let Some(sim) = &mut live_simulation {
            if sim.current_timestep < SIMULATION_TIMESTEPS {
                let dt = 0.016 / SUBSTEPS as f32;
                let mut collision = false;
                for _ in 0..SUBSTEPS {
                    update_bodies(&mut sim.bodies, dt);
                    if sim.bodies[0].pos.distance(&sim.bodies[1].pos) < sim.bodies[0].radius + sim.bodies[1].radius ||
                       sim.bodies[0].pos.distance(&sim.bodies[2].pos) < sim.bodies[0].radius + sim.bodies[2].radius ||
                       sim.bodies[1].pos.distance(&sim.bodies[2].pos) < sim.bodies[1].radius + sim.bodies[2].radius {
                        collision = true;
                        break;
                    }
                }

                if collision {
                    sim.current_timestep = SIMULATION_TIMESTEPS; // End simulation
                } else {
                    sim.current_timestep += 1;
                    if sim.current_timestep % POSITION_SAMPLE_RATE == 0 {
                        for i in 0..3 {
                            sim.trajectory_history[i].push(sim.bodies[i].pos);
                        }
                    }
                }
            }

            // --- Camera and Drawing Logic ---
            let screen_center = macroquad::math::Vec2::new(IMAGE_SIZE as f32 + IMAGE_SIZE as f32 / 2.0, IMAGE_SIZE as f32 / 2.0);
            let colors = [YELLOW, BLUE, MAGENTA];

            // Calculate Center of Mass
            let total_mass: f32 = sim.bodies.iter().map(|b| b.mass).sum();
            let center_of_mass = sim.bodies.iter().fold(Vec2::new(0.0, 0.0), |acc, b| acc + b.pos * b.mass) / total_mass;

            // Calculate bounding box of all trajectories
            let mut min_p = sim.bodies[0].pos;
            let mut max_p = sim.bodies[0].pos;
            for traj in sim.trajectory_history.iter() {
                for p in traj.iter() {
                    min_p.x = min_p.x.min(p.x);
                    min_p.y = min_p.y.min(p.y);
                    max_p.x = max_p.x.max(p.x);
                    max_p.y = max_p.y.max(p.y);
                }
            }

            // Calculate zoom to fit bounding box
            let (bbox_w, bbox_h) = (max_p.x - min_p.x, max_p.y - min_p.y);
            let padding = 1.2; // 20% padding
            let zoom_x = if bbox_w > 0.0 { (IMAGE_SIZE as f32 / bbox_w) / padding } else { 1.0 };
            let zoom_y = if bbox_h > 0.0 { (IMAGE_SIZE as f32 / bbox_h) / padding } else { 1.0 };
            let zoom = zoom_x.min(zoom_y);

            // Helper to transform world coords to screen coords
            let transform = |p: Vec2| {
                (to_macroquad_vec2(p) - to_macroquad_vec2(center_of_mass)) * zoom + screen_center
            };

            // Draw traced paths
            for (j, traj) in sim.trajectory_history.iter().enumerate() {
                for i in 1..traj.len() {
                    let p1 = transform(traj[i-1]);
                    let p2 = transform(traj[i]);
                    draw_line(p1.x, p1.y, p2.x, p2.y, 1.0, colors[j]);
                }
            }

            // Draw current body positions
            for (i, body) in sim.bodies.iter().enumerate() {
                let p = transform(body.pos);
                draw_circle(p.x, p.y, body.radius * zoom.min(10.0), colors[i]); // Cap radius zoom
            }
        }

        next_frame().await;
    }
}

fn run_simulation(bodies: &mut [GravBody]) -> f32 {
    let dt = 0.016 / SUBSTEPS as f32;

    for i in 0..SIMULATION_TIMESTEPS {
        for _ in 0..SUBSTEPS {
            update_bodies(bodies, dt);
            // Check for collision inside the substep loop
            if bodies[0].pos.distance(&bodies[1].pos) < bodies[0].radius + bodies[1].radius ||
               bodies[0].pos.distance(&bodies[2].pos) < bodies[0].radius + bodies[2].radius ||
               bodies[1].pos.distance(&bodies[2].pos) < bodies[1].radius + bodies[2].radius {
                return i as f32; // Return time to collision
            }
        }
    }

    SIMULATION_TIMESTEPS as f32 // No collision
}