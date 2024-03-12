extern crate ode_solvers;
use ode_solvers::dopri5::*;

// Define the differential equations
fn equations_of_motion(_t: f64, y: &[f64], dydt: &mut [f64]) {
    // y[0] and y[1] are the coordinates x and y, respectively
    // y[2] and y[3] are the velocities vx and vy, respectively
    dydt[0] = y[2];
    dydt[1] = y[3];
    dydt[2] = 0.0;
    dydt[3] = -9.8; // Acceleration due to gravity
}

fn main() {
    // Initial conditions
    let initial_position: [f64; 2] = [0.0, 0.0];
    let initial_velocity: [f64; 2] = [1.0, 1.0];
    let initial_conditions = initial_position
        .iter()
        .chain(initial_velocity.iter())
        .cloned()
        .collect::<Vec<_>>();

    // Time points for integration
    let t_span = (0.0, 10.0);
    let t_eval = linspace(t_span.0, t_span.1, 100);

    // Integrate the equations of motion
    //let mut integrator = Dopri5::new(equations_of_motion, t_span.0, &initial_conditions, t_span.1);
    //let result = integrator.integrate(&t_eval).unwrap();

    // Extracting the results
    //let x: Vec<f64> = result.iter().step_by(4).cloned().collect();
    //let y: Vec<f64> = result.iter().skip(1).step_by(4).cloned().collect();

    // Plot the trajectory (Note: Plotting in Rust can be more involved and depends on external libraries)
    println!("x: {:?}", initial_position);
    println!("y: {:?}", initial_velocity);
}

// Utility function to create a linspace
fn linspace(start: f64, end: f64, n: usize) -> Vec<f64> {
    (0..n).map(|i| start + i as f64 * (end - start) / (n - 1) as f64).collect()
}