//! This module contains inverse-kinematic functionality for the skelly crate.
//!
//! # Example
//!
//! ```
//! use {skelly::{Skelly, Posture, ik::frik::FrikSolver}, na::{Point3, Vector3, Isometry3}};
//! let mut skelly = Skelly::<f32>::new();
//! let foot = skelly.add_root(Point3::origin());
//! let leg = skelly.attach(Vector3::z().into(), foot);
//! let waist = skelly.attach(Vector3::z().into(), leg);
//! let left_shoulder = skelly.attach(Vector3::z().into(), waist);
//! let left_arm = skelly.attach((-Vector3::x()).into(), left_shoulder);
//! let left_palm = skelly.attach((-Vector3::x()).into(), left_arm);
//! let right_shoulder = skelly.attach(Vector3::z().into(), waist);
//! let right_arm = skelly.attach(Vector3::x().into(), right_shoulder);
//! let right_palm = skelly.attach(Vector3::x().into(), right_arm);
//!
//! // Using the skelly above, do some inverse-kinematics
//! let mut posture = Posture::new(&skelly);
//! let mut solver = FrikSolver::new(0.01);
//!
//! // move left palm to the foot.
//! solver.set_position_goal(left_palm, Point3::origin());
//!
//! // Iteratively solve imposed constraints.
//! for _ in 0..100 {
//!   solver.solve_step(&skelly, &mut posture);
//! }
//!
//! // Write global isometries of every joint in the posture into an array.
//! let mut globals = vec![Isometry3::identity(); skelly.len()];
//! posture.write_globals(&skelly, &Isometry3::identity(), &mut globals);
//!

pub mod fabrik;
pub mod frik;
pub mod rotor;

use {
    crate::skelly::{Posture, Skelly},
    na::Scalar,
};

/// Variants of results for `IkSolver::solve_step` method.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum StepResult {
    /// All constrains and goals are satisfied with error less than configured for solver.
    Solved,

    /// Error in constraints and goals are unsatisfied
    Unsolved,

    /// Returned if solver determined that goals cannot be satisfied given the constraitns.
    Infeasible,
}

/// Trait for ik solvers.
/// Using this common interface user may replace implementation easily.
pub trait IkSolver<T: Scalar> {
    /// Returns new solver with maximum tolerable error.
    fn new(error: T) -> Self;

    /// Performs one step toward solution.
    fn solve_step<D>(&mut self, skelly: &Skelly<T, D>, posture: &mut Posture<T>) -> StepResult;
}
