//! This module contains inverse-kinematic functionality for the skelly crate.

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
