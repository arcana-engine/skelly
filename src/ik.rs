//! This module contains inverse-kinematic functionality for the skelly crate.

// pub mod fabrik;
pub mod rotor;

use {
    crate::skelly::{Posture, Skelly},
    core::ops::Add,
    na::Scalar,
};

/// Variants of results for `IkSolver::solve_step` method.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Status {
    /// All constrains and goals are satisfied with error less than configured for solver.
    Solved,

    /// Error in constraints and goals are unsatisfied
    Unsolved,

    /// Returned if solver determined that goals cannot be satisfied given the constraitns.
    Infeasible,
}

pub struct StepResult<T> {
    error: T,
    status: Status,
}

impl<T> StepResult<T>
where
    T: Add<Output = T>,
{
    pub fn solved(error: T) -> Self {
        StepResult {
            error,
            status: Status::Solved,
        }
    }

    pub fn unsolved(error: T) -> Self {
        StepResult {
            error,
            status: Status::Unsolved,
        }
    }

    pub fn infeasible(error: T) -> Self {
        StepResult {
            error,
            status: Status::Infeasible,
        }
    }

    /// Combine two step results into one
    pub fn reduce(self, rhs: Self) -> Self {
        let status = match (self.status, rhs.status) {
            (Status::Infeasible, _) | (_, Status::Infeasible) => Status::Infeasible,
            (Status::Unsolved, _) | (_, Status::Unsolved) => Status::Unsolved,
            (Status::Solved, Status::Solved) => Status::Solved,
        };

        StepResult {
            error: self.error + rhs.error,
            status,
        }
    }
}

/// Trait for ik solvers.
/// Using this common interface user may replace implementation easily.
pub trait IkSolver<T: Scalar> {
    /// Returns new solver with maximum tolerable error.
    fn new(error: T) -> Self;

    /// Performs one step toward solution.
    fn solve_step<D>(&mut self, skelly: &Skelly<T, D>, posture: &mut Posture<T>) -> StepResult<T>;
}
