//!
//! skelly crate.
//!

#[cfg(feature = "ik")]
pub mod ik;

mod skelly;

pub use self::skelly::*;
