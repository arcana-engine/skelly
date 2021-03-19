//!
//! Crate for skeleton animation.
//!
//! Optionally provides inverse-kinematics functionality.
//!
//! # Example
//! ```
//! use {skelly::Skelly, na::{Point3, Vector3, Isometry3}};
//!
//! // build a skelly with leg and two arms.
//! let mut skelly = Skelly::<f32>::new();
//! let foot = skelly.add_root(Point3::origin());
//! let leg = skelly.attach(Vector3::z().into(), foot);
//! let waist = skelly.attach(Vector3::z().into(), leg);
//!
//! let left_shoulder = skelly.attach(Vector3::z().into(), waist);
//! let left_arm = skelly.attach((-Vector3::x()).into(), left_shoulder);
//! let left_palm = skelly.attach((-Vector3::x()).into(), left_arm);
//!
//! let right_shoulder = skelly.attach(Vector3::z().into(), waist);
//! let right_arm = skelly.attach(Vector3::x().into(), right_shoulder);
//! let right_palm = skelly.attach(Vector3::x().into(), right_arm);
//!
//! // Write global isometries of every joint into an array.
//! let mut globals = vec![Isometry3::identity(); skelly.len()];
//! skelly.write_globals(&Isometry3::identity(), &mut globals);
//!
//! ```
//!
//! See `examples/demo.rs` for working example.
//!

#[cfg(feature = "ik")]
pub mod ik;

mod skelly;

pub use self::skelly::*;
