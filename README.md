# skelly

[![crates](https://img.shields.io/crates/v/skelly.svg?style=for-the-badge&label=skelly)](https://crates.io/crates/skelly)
[![docs](https://img.shields.io/badge/docs.rs-skelly-66c2a5?style=for-the-badge&labelColor=555555&logoColor=white)](https://docs.rs/skelly)
[![actions](https://img.shields.io/github/workflow/status/zakarumych/skelly/badge/master?style=for-the-badge)](https://github.com/zakarumych/skelly/actions?query=workflow%3ARust)
[![MIT/Apache](https://img.shields.io/badge/license-MIT%2FApache-blue.svg?style=for-the-badge)](COPYING)
![loc](https://img.shields.io/tokei/lines/github/zakarumych/skelly?style=for-the-badge)


Crate for skeleton animation.

Optionally provides inverse-kinematics functionality.

## Example
```rust
use {skelly::Skelly, na::{Point3, Vector3, Isometry3}};

// build a skelly with leg and two arms.
let mut skelly = Skelly::<f32>::new();
let foot = skelly.add_root(Point3::origin());
let leg = skelly.attach(Vector3::z().into(), foot);
let waist = skelly.attach(Vector3::z().into(), leg);

let left_shoulder = skelly.attach(Vector3::z().into(), waist);
let left_arm = skelly.attach((-Vector3::x()).into(), left_shoulder);
let left_palm = skelly.attach((-Vector3::x()).into(), left_arm);

let right_shoulder = skelly.attach(Vector3::z().into(), waist);
let right_arm = skelly.attach(Vector3::x().into(), right_shoulder);
let right_palm = skelly.attach(Vector3::x().into(), right_arm);

// Write global isometries of every joint into an array.
let mut globals = vec![Isometry3::identity(); skelly.len()];
skelly.write_globals(&Isometry3::identity(), &mut globals);

```

See `examples/demo.rs` for working example.


## License

Licensed under either of

* Apache License, Version 2.0, ([license/APACHE](license/APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
* MIT license ([license/MIT](license/MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contributions

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

## Donate

[![Become a patron](https://c5.patreon.com/external/logo/become_a_patron_button.png)](https://www.patreon.com/zakarum)
