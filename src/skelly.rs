use na::{Isometry3, Point3, RealField, Scalar, Translation3, UnitQuaternion};

/// One's skeleton.
/// Parameterized with numric value and bone userdata type.
pub struct Skelly<T: Scalar, D = ()> {
    bones: Vec<Bone<T, D>>,
}

struct Bone<T: Scalar, D> {
    isometry: Isometry3<T>,
    parent: Option<usize>,
    userdata: D,
}

impl<T> Skelly<T>
where
    T: Scalar,
{
    /// Add root bone to the skelly.
    /// E.g. one with not parent bone.
    pub fn add_root(&mut self, position: Point3<T>) -> usize
    where
        T: RealField,
    {
        self.add_root_with(position, ())
    }

    /// Attach a bone to an existing bone.
    /// New bone is attached to the parent bone with specified relative position.
    pub fn attach(&mut self, relative_position: Point3<T>, parent: usize) -> usize
    where
        T: RealField,
    {
        self.attach_with(relative_position, parent, ())
    }
}

impl<T, D> Skelly<T, D>
where
    T: Scalar,
{
    /// Returns new empty skelly.
    pub fn new() -> Self {
        Skelly { bones: Vec::new() }
    }

    /// Add root bone to the skelly.
    /// E.g. one with not parent bone.
    pub fn add_root_with(&mut self, position: Point3<T>, userdata: D) -> usize
    where
        T: RealField,
    {
        self.bones.push(Bone {
            isometry: Isometry3 {
                rotation: UnitQuaternion::identity(),
                translation: position.coords.into(),
            },
            parent: None,
            userdata,
        });
        self.bones.len() - 1
    }

    /// Attach a bone to an existing bone.
    /// New bone is attached to the parent bone with specified displacement.
    #[track_caller]
    pub fn attach_with(&mut self, relative_position: Point3<T>, parent: usize, userdata: D) -> usize
    where
        T: RealField,
    {
        assert!(parent < self.bones.len(), "Parent index is ouf of bounds");
        self.bones.push(Bone {
            isometry: Isometry3 {
                rotation: UnitQuaternion::identity(),
                translation: relative_position.coords.into(),
            },
            parent: Some(parent),
            userdata,
        });
        self.bones.len() - 1
    }

    /// Rotate specified bone.
    #[track_caller]
    pub fn rotate(&mut self, bone: usize, rotation: UnitQuaternion<T>)
    where
        T: RealField,
    {
        let bone = &mut self.bones[bone];
        bone.isometry.rotation = bone.isometry.rotation * rotation;
    }

    /// Move specified bone.
    #[track_caller]
    pub fn translate_bone(&mut self, bone: usize, translation: Translation3<T>)
    where
        T: RealField,
    {
        let bone = &mut self.bones[bone];
        bone.isometry.translation = bone.isometry.translation * translation;
    }

    /// Set position for specified bone.
    #[track_caller]
    pub fn set_bone_position(&mut self, bone: usize, position: Point3<T>) {
        self.bones[bone].isometry.translation = position.coords.into();
    }

    /// Returns reference to userdata attached to the bone.
    #[track_caller]
    pub fn get_userdata(&self, bone: usize) -> &D {
        &self.bones[bone].userdata
    }

    /// Returns mutable reference to userdata attached to the bone.
    #[track_caller]
    pub fn get_userdata_mut(&mut self, bone: usize) -> &mut D {
        &mut self.bones[bone].userdata
    }

    #[track_caller]
    pub fn get_parent(&self, bone: usize) -> Option<usize> {
        self.bones[bone].parent
    }

    /// Returns number of bones in the skelly.
    pub fn len(&self) -> usize {
        self.bones.len()
    }

    /// Fill slice of `Mat4` with global isometrys
    /// for each bone of the skelly in specified posture.
    pub fn write_globals(&self, globals: &mut [Isometry3<T>])
    where
        T: RealField,
    {
        self.bones
            .iter()
            .take(globals.len())
            .enumerate()
            .for_each(|(index, bone)| match bone.parent {
                Some(parent) => {
                    debug_assert!(parent < index);
                    globals[index] = globals[parent] * bone.isometry;
                }
                None => {
                    globals[index] = bone.isometry;
                }
            })
    }

    /// Fill slice of `Mat4` with global isometrys
    /// for each bone of the skelly in specified posture.
    pub fn write_globals_for_posture(&self, posture: &Posture<T>, globals: &mut [Isometry3<T>])
    where
        T: RealField,
    {
        self.bones
            .iter()
            .zip(&posture.joints)
            .take(globals.len())
            .enumerate()
            .for_each(|(index, (bone, isometry))| match bone.parent {
                Some(parent) => {
                    debug_assert!(parent < index);
                    globals[index] = globals[parent] * *isometry;
                }
                None => {
                    globals[index] = *isometry;
                }
            })
    }

    /// Makes the skelly to assume specifed posture.
    #[track_caller]
    pub fn assume_posture(&mut self, posture: &Posture<T>)
    where
        T: Copy,
    {
        assert_eq!(self.bones.len(), posture.joints.len());

        self.bones
            .iter_mut()
            .zip(&posture.joints)
            .for_each(|(bone, isometry)| bone.isometry = *isometry);
    }

    /// Make `Posture` instance out of the skelly.
    pub fn make_posture(&self) -> Posture<T>
    where
        T: Copy,
    {
        Posture {
            joints: self.bones.iter().map(|bone| bone.isometry).collect(),
        }
    }

    #[track_caller]
    pub fn make_chain(&self, mut bone: usize, chain: &mut Vec<usize>) {
        while let Some(parent) = self.bones[bone].parent {
            chain.push(parent);
            bone = parent;
        }
    }

    // pub fn iter_chain(&self, mut bone: usize) -> impl Iterator<Item = usize> + '_ {
    //     std::iter::from_fn(move || {
    //         if let Some(parent) = self.bones[bone].parent {
    //             bone = parent;
    //             Some(bone)
    //         } else {
    //             None
    //         }
    //     })
    // }

    #[track_caller]
    pub(crate) fn iter_children(&self, parent: usize) -> impl Iterator<Item = usize> + '_ {
        self.bones
            .iter()
            .enumerate()
            .skip(parent)
            .filter_map(move |(index, bone)| {
                if bone.parent == Some(parent) {
                    Some(index)
                } else {
                    None
                }
            })
    }
}

pub struct Posture<T: Scalar> {
    joints: Vec<Isometry3<T>>,
}

impl<T> Posture<T>
where
    T: Scalar,
{
    pub fn new(len: usize) -> Self
    where
        T: RealField,
    {
        Posture {
            joints: vec![Isometry3::identity(); len],
        }
    }

    pub fn len(&self) -> usize {
        self.joints.len()
    }

    #[track_caller]
    pub fn get_joint(&self, bone: usize) -> &Isometry3<T> {
        &self.joints[bone]
    }

    #[track_caller]
    pub fn get_joint_mut(&mut self, bone: usize) -> &mut Isometry3<T> {
        &mut self.joints[bone]
    }

    #[track_caller]
    pub fn rotate(&mut self, bone: usize, rotation: &UnitQuaternion<T>)
    where
        T: RealField,
    {
        self.joints[bone] *= rotation;
    }

    #[track_caller]
    pub fn translate(&mut self, bone: usize, translation: &Translation3<T>)
    where
        T: RealField,
    {
        self.joints[bone] = translation * self.joints[bone];
    }
}
