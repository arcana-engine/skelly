use na::{Isometry3, Point3, RealField, Scalar, Translation3, UnitQuaternion, Vector3};

/// One's skeleton.
/// Parameterized with numric value and bone userdata type.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde-1", derive(serde::Serialize, serde::Deserialize))]
pub struct Skelly<T: Scalar, D = ()> {
    bones: Vec<Bone<T, D>>,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde-1", derive(serde::Serialize, serde::Deserialize))]
struct Bone<T: Scalar, D> {
    isometry: Isometry3<T>,
    parent: Option<usize>,
    userdata: D,
}

impl<T, D> Default for Skelly<T, D>
where
    T: Scalar,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T, D> Skelly<T, D>
where
    T: Scalar,
{
    /// Returns new empty skelly.
    ///
    /// # Example
    ///
    /// ```
    /// # use skelly::Skelly;
    /// let skelly = Skelly::<f32>::new();
    /// ```
    pub fn new() -> Self {
        Skelly { bones: Vec::new() }
    }

    /// Creates new root bone in the skelly at specified `position`.
    ///
    /// Root bones are ones that have no parent bone.\
    /// Returns id of the added root bone.\
    /// `userdata` will be associated with the bone.
    /// It maybe be accessed with
    /// [`Skelly::get_userdata`] and [`Skelly::get_userdata_mut`]
    /// using returned id.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::Point3};
    /// let mut skelly = Skelly::<f32, &str>::new();
    /// let root = skelly.add_root_with(Point3::origin(), "root-user-data");
    /// ```
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

    /// Attaches new bone to an existing bone with specified id.
    ///
    /// Returns id of the added bone.\
    /// The bone will be placed `relative` to its parent.\
    /// `userdata` will be associated with the bone.
    /// It maybe be accessed with
    /// [`Skelly::get_userdata`] and [`Skelly::get_userdata_mut`]
    /// using returned id.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Vector3}};
    /// let mut skelly = Skelly::<f32, &str>::new();
    /// let root = skelly.add_root_with(Point3::origin(), "root-user-data");
    /// let bone = skelly.attach_with(Vector3::x(), root, "bone-user-data");
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `parent` index is out of bounds.
    #[track_caller]
    pub fn attach_with(&mut self, relative: Vector3<T>, parent: usize, userdata: D) -> usize
    where
        T: RealField,
    {
        assert!(parent < self.bones.len(), "Parent index is ouf of bounds");
        self.bones.push(Bone {
            isometry: Isometry3 {
                rotation: UnitQuaternion::identity(),
                translation: relative.into(),
            },
            parent: Some(parent),
            userdata,
        });

        self.bones.len() - 1
    }

    /// Rotates bone with specified id.
    ///
    /// *Does not* affect relative position to the parent and global position for root bones.
    /// Affects global position of all descendant bones.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Isometry3, UnitQuaternion, Vector3}, core::f32::{consts::PI, EPSILON}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut globals = [Isometry3::identity(); 2];
    /// skelly.write_globals(&Isometry3::identity(), &mut globals);
    /// let bone_global_old = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at (1, 0, 0).
    /// assert!((bone_global_old.translation.vector - Vector3::x()).magnitude() < EPSILON);
    ///
    /// // Rotate root bone. It is still at origin.
    /// // Yet global position of the `bone` attached to `root` has changed accordingly.
    /// skelly.append_rotation(root, UnitQuaternion::from_euler_angles(0.0, 0.0, PI / 2.0));
    ///
    /// skelly.write_globals(&Isometry3::identity(), &mut globals);
    /// let bone_global_new = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space after root rotation at (0, 1, 0).
    /// assert!((bone_global_new.translation.vector - Vector3::y()).magnitude() < EPSILON);
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    #[track_caller]
    pub fn append_rotation(&mut self, bone: usize, rotation: UnitQuaternion<T>)
    where
        T: RealField,
    {
        self.bones[bone].isometry.rotation *= rotation
    }

    /// Rotates bone with specified id.
    ///
    /// Affect relative position to the parent and global position for root bones.
    /// Affects global position of all descendant bones.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Isometry3, UnitQuaternion, Vector3}, core::f32::{consts::PI, EPSILON}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut globals = [Isometry3::identity(); 2];
    /// skelly.write_globals(&Isometry3::identity(), &mut globals);
    /// let bone_global_old = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at (1, 0, 0).
    /// assert!((bone_global_old.translation.vector - Vector3::x()).magnitude() < EPSILON);
    ///
    /// // Rotate the bone. It is still at origin.
    /// // Yet global position of the `bone` attached to `root` has changed accordingly.
    /// skelly.prepend_rotation(bone, UnitQuaternion::from_euler_angles(0.0, 0.0, PI / 2.0));
    ///
    /// skelly.write_globals(&Isometry3::identity(), &mut globals);
    /// let bone_global_new = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space after root rotation at (0, 1, 0).
    /// assert!((bone_global_new.translation.vector - Vector3::y()).magnitude() < EPSILON);
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    #[track_caller]
    pub fn prepend_rotation(&mut self, bone: usize, rotation: UnitQuaternion<T>)
    where
        T: RealField,
    {
        let my_isometry = &mut self.bones[bone].isometry;
        *my_isometry = rotation * &*my_isometry;
    }

    /// Translates bone with specified id.
    ///
    /// Affects relative position to the parent and global position for root bones.
    /// Affects global position of all descendant bones.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Isometry3, UnitQuaternion, Vector3}, core::f32::{consts::PI, EPSILON}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut globals = [Isometry3::identity(); 2];
    /// skelly.write_globals(&Isometry3::identity(), &mut globals);
    /// let bone_global_old = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at (1, 0, 0).
    /// assert!((bone_global_old.translation.vector - Vector3::x()).magnitude() < EPSILON);
    ///
    /// // Translate root bone.
    /// // Global position of the `bone` attached to `root` has changed accordingly.
    /// skelly.append_translation(root, Vector3::z().into());
    ///
    /// skelly.write_globals(&Isometry3::identity(), &mut globals);
    /// let bone_global_new = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space after root translation at (1, 0, 1).
    /// assert!((bone_global_new.translation.vector - (Vector3::x() + Vector3::z())).magnitude() < EPSILON);
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    #[track_caller]
    pub fn append_translation(&mut self, bone: usize, translation: Translation3<T>)
    where
        T: RealField,
    {
        self.bones[bone].isometry.translation *= translation;
    }

    /// Sets relative position for bone with specified id.
    /// Affects global position of all descendant bones.
    ///
    /// This method ignores current relative position of the bone.
    /// To apply translation to current relative poistion see [`Skelly::append_translation`].
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Isometry3, UnitQuaternion, Vector3}, core::f32::{consts::PI, EPSILON}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut globals = [Isometry3::identity(); 2];
    /// skelly.write_globals(&Isometry3::identity(), &mut globals);
    /// let bone_global_old = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at (1, 0, 0).
    /// assert!((bone_global_old.translation.vector - Vector3::x()).magnitude() < EPSILON);
    ///
    /// // Set new relative position for the `bone`.
    /// skelly.set_position(bone, Vector3::z());
    ///
    /// skelly.write_globals(&Isometry3::identity(), &mut globals);
    /// let bone_global_new = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at new position (0, 0, 1).
    /// assert!((bone_global_new.translation.vector - Vector3::z()).magnitude() < EPSILON);
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    #[track_caller]
    pub fn set_position(&mut self, bone: usize, position: Vector3<T>) {
        self.bones[bone].isometry.translation = position.into();
    }

    /// Returns current bone position relative to parent.
    #[track_caller]
    pub fn get_position(&mut self, bone: usize) -> &Vector3<T>
    where
        T: RealField,
    {
        &self.bones[bone].isometry.translation.vector
    }

    /// Sets relative orientation for bone with specified id.
    /// Affects global position of all descendant bones.
    ///
    /// This method ignores current relative position of the bone.
    /// To apply translation to current relative poistion see [`Skelly::append_translation`].
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Isometry3, UnitQuaternion, Vector3}, core::f32::{consts::PI, EPSILON}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut globals = [Isometry3::identity(); 2];
    /// skelly.write_globals(&Isometry3::identity(), &mut globals);
    /// let bone_global_old = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at (1, 0, 0).
    /// assert!((bone_global_old.translation.vector - Vector3::x()).magnitude() < EPSILON);
    ///
    /// // Set new relative orientation for the `bone`.
    /// skelly.set_orientation(root, UnitQuaternion::from_euler_angles(0.0, 0.0, PI / 2.0));
    ///
    /// skelly.write_globals(&Isometry3::identity(), &mut globals);
    /// let bone_global_new = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at new position (0, 0, 1).
    /// assert!((bone_global_new.translation.vector - Vector3::y()).magnitude() < EPSILON);
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    #[track_caller]
    pub fn set_orientation(&mut self, bone: usize, orientation: UnitQuaternion<T>) {
        self.bones[bone].isometry.rotation = orientation;
    }

    /// Returns current bone orientation relative to parent.
    #[track_caller]
    pub fn get_orientation(&mut self, bone: usize) -> &UnitQuaternion<T>
    where
        T: RealField,
    {
        &self.bones[bone].isometry.rotation
    }

    /// Returns current bone isometry relative to parent.
    #[track_caller]
    pub fn get_isometry(&mut self, bone: usize) -> &Isometry3<T>
    where
        T: RealField,
    {
        &self.bones[bone].isometry
    }

    /// Returns reference to userdata associated with the `bone`.
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Vector3}};
    /// let mut skelly = Skelly::<f32, &str>::new();
    /// let root = skelly.add_root_with(Point3::origin(), "root-bone-data");
    /// let bone = skelly.attach_with(Vector3::x(), root, "another-bone-data");
    ///
    /// assert_eq!(*skelly.get_userdata(root), "root-bone-data");
    /// assert_eq!(*skelly.get_userdata(bone), "another-bone-data");
    /// ```
    #[track_caller]
    pub fn get_userdata(&self, bone: usize) -> &D {
        &self.bones[bone].userdata
    }

    /// Returns mutable reference to userdata associated with the `bone`.
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Vector3}};
    /// let mut skelly = Skelly::<f32, Vec<&str>>::new();
    /// let root = skelly.add_root_with(Point3::origin(), vec![]);
    ///
    /// skelly.get_userdata_mut(root).push("another-root-data-entry");
    /// assert_eq!(*skelly.get_userdata(root), ["another-root-data-entry"]);
    /// ```
    #[track_caller]
    pub fn get_userdata_mut(&mut self, bone: usize) -> &mut D {
        &mut self.bones[bone].userdata
    }

    /// Associated new userdata with the `bone`.
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Vector3}};
    /// let mut skelly = Skelly::<f32, &str>::new();
    /// let root = skelly.add_root_with(Point3::origin(), "initial-root-data");
    ///
    /// skelly.set_userdata(root, "new-root-data");
    /// assert_eq!(*skelly.get_userdata(root), "new-root-data");
    /// ```
    #[track_caller]
    pub fn set_userdata(&mut self, bone: usize, userdata: D) {
        self.bones[bone].userdata = userdata
    }

    /// Returns parent of the specified `bone`.
    /// Returns `None` for root bones.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Vector3}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// assert_eq!(skelly.get_parent(bone), Some(root));
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    #[track_caller]
    pub fn get_parent(&self, bone: usize) -> Option<usize> {
        self.bones[bone].parent
    }

    /// Returns number of bones in the skelly.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Vector3}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// assert_eq!(skelly.len(), 2);
    /// ```
    pub fn len(&self) -> usize {
        self.bones.len()
    }

    /// Returns if the skelly has no bones.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Vector3}};
    /// let mut skelly = Skelly::<f32>::new();
    ///
    /// assert!(skelly.is_empty());
    ///
    /// let root = skelly.add_root(Point3::origin());
    ///
    /// assert!(!skelly.is_empty());
    /// ```
    pub fn is_empty(&self) -> bool {
        self.bones.is_empty()
    }

    /// Fills slice of `Isometry3` with global isometries
    /// for each bone of the skelly.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Vector3, Isometry3}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut globals = [Isometry3::identity(); 2];
    /// skelly.write_globals(&Isometry3::identity(), &mut globals);
    /// ```
    pub fn write_globals(&self, skelly_global: &Isometry3<T>, globals: &mut [Isometry3<T>])
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
                    globals[index] = &globals[parent] * &bone.isometry;
                }
                None => {
                    globals[index] = skelly_global * &bone.isometry;
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

    /// Iterates through bone ancestors up until root bone is reached
    /// yielding their ids.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Vector3, Isometry3}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    /// let tip = skelly.attach(Vector3::x(), bone);
    ///
    /// assert_eq!(skelly.iter_chain(tip).collect::<Vec<_>>(), [bone, root]);
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    pub fn iter_chain(&self, mut bone: usize) -> impl Iterator<Item = usize> + '_ {
        std::iter::from_fn(move || {
            if let Some(parent) = self.bones[bone].parent {
                bone = parent;
                Some(bone)
            } else {
                None
            }
        })
    }

    /// Iterates through the bone's direct descendants
    /// yielding their ids.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Vector3, Isometry3}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let left = skelly.attach(Vector3::x(), root);
    /// let right = skelly.attach(Vector3::x(), root);
    ///
    /// assert_eq!(skelly.iter_children(root).collect::<Vec<_>>(), [left, right]);
    /// ```
    ///
    /// This method is not very efficient.
    /// As it effectively scans sub-slice [bone..]
    /// Use with caution for too complex skellies in hot-paths.
    ///
    /// TODO: Consider adding skelly building phase to pack siblings together.
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    #[track_caller]
    pub fn iter_children(&self, parent: usize) -> impl Iterator<Item = usize> + '_ {
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

impl<T> Skelly<T>
where
    T: Scalar,
{
    /// Creates new root bone in the skelly at specified `position`.
    ///
    /// Root bones are ones that have no parent bone.\
    /// Returns id of the added root bone.\
    ///
    /// `skelly.add_root(pos)` is a more pleasant shorthand for `skelly.add_root_with(pos, ())`;
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::Point3};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// ```
    pub fn add_root(&mut self, position: Point3<T>) -> usize
    where
        T: RealField,
    {
        self.add_root_with(position, ())
    }

    /// Attaches new bone to an existing bone with specified id.
    ///
    /// Returns id of the added bone.\
    /// The bone will be placed `relative` to its parent.\
    ///
    /// `skelly.attach(relative, parent)` is a more pleasant shorthand for `skelly.attach_with(relative, parent, ())`;
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::Skelly, na::{Point3, Vector3}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `parent` index is out of bounds.
    #[track_caller]
    pub fn attach(&mut self, relative: Vector3<T>, parent: usize) -> usize
    where
        T: RealField,
    {
        self.attach_with(relative, parent, ())
    }
}

/// Collection of bones transformations
/// that represent a skelly posture.
///
/// It's primary usecase is to be used instead
/// of transformations contained in the `Skelly`.
/// Multiple postures to be processed for the same `Skelly`.
/// Allowing running animations, IK algorithms etc,
/// and then blend them to get final posture.
pub struct Posture<T: Scalar> {
    joints: Vec<Isometry3<T>>,
}

impl<T> Posture<T>
where
    T: Scalar,
{
    /// Returns new `Posture` instance for `skelly`.
    /// Copies current `skelly` transformations.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::{Skelly, Posture}, na::{Point3, Vector3}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut posture = Posture::new(&skelly);
    /// ```
    pub fn new<D>(skelly: &Skelly<T, D>) -> Self
    where
        T: RealField,
    {
        Posture {
            joints: skelly
                .bones
                .iter()
                .map(|bone| bone.isometry.clone())
                .collect(),
        }
    }

    pub fn is_compatible<D>(&self, skelly: &Skelly<T, D>) -> bool {
        self.joints.len() == skelly.bones.len()
    }

    /// Rotates bone with specified id.
    ///
    /// *Does not* affect relative position to the parent and global position for root bones.
    /// Affects global position of all descendant bones.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::{Skelly, Posture}, na::{Point3, Isometry3, UnitQuaternion, Vector3}, core::f32::{consts::PI, EPSILON}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut posture = Posture::new(&skelly);
    ///
    /// let mut globals = [Isometry3::identity(); 2];
    /// posture.write_globals(&skelly, &Isometry3::identity(), &mut globals);
    /// let bone_global_old = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at (1, 0, 0).
    /// assert!((bone_global_old.translation.vector - Vector3::x()).magnitude() < EPSILON);
    ///
    /// // Rotate root bone. It is still at origin.
    /// // Yet global position of the `bone` attached to `root` has changed accordingly.
    /// posture.append_rotation(root, UnitQuaternion::from_euler_angles(0.0, 0.0, PI / 2.0));
    ///
    /// posture.write_globals(&skelly, &Isometry3::identity(), &mut globals);
    /// let bone_global_new = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space after root rotation at (0, 1, 0).
    /// assert!((bone_global_new.translation.vector - Vector3::y()).magnitude() < EPSILON);
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    #[track_caller]
    pub fn append_rotation(&mut self, bone: usize, rotation: UnitQuaternion<T>)
    where
        T: RealField,
    {
        self.joints[bone].rotation *= rotation
    }

    /// Rotates bone with specified id.
    ///
    /// *Does not* affect relative position to the parent and global position for root bones.
    /// Affects global position of all descendant bones.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::{Skelly, Posture}, na::{Point3, Isometry3, UnitQuaternion, Vector3}, core::f32::{consts::PI, EPSILON}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut posture = Posture::new(&skelly);
    ///
    /// let mut globals = [Isometry3::identity(); 2];
    /// posture.write_globals(&skelly, &Isometry3::identity(), &mut globals);
    /// let bone_global_old = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at (1, 0, 0).
    /// assert!((bone_global_old.translation.vector - Vector3::x()).magnitude() < EPSILON);
    ///
    /// // Rotate the bone. It is still at origin.
    /// // Yet global position of the `bone` attached to `root` has changed accordingly.
    /// posture.prepend_rotation(bone, UnitQuaternion::from_euler_angles(0.0, 0.0, PI / 2.0));
    ///
    /// posture.write_globals(&skelly, &Isometry3::identity(), &mut globals);
    /// let bone_global_new = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space after root rotation at (0, 1, 0).
    /// assert!((bone_global_new.translation.vector - Vector3::y()).magnitude() < EPSILON);
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    #[track_caller]
    pub fn prepend_rotation(&mut self, bone: usize, rotation: UnitQuaternion<T>)
    where
        T: RealField,
    {
        let my_isometry = &mut self.joints[bone];
        *my_isometry = rotation * &*my_isometry;
    }

    /// Translates bone with specified id.
    ///
    /// Affects relative position to the parent and global position for root bones.
    /// Affects global position of all descendant bones.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::{Skelly, Posture}, na::{Point3, Isometry3, UnitQuaternion, Vector3}, core::f32::{consts::PI, EPSILON}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut posture = Posture::new(&skelly);
    ///
    /// let mut globals = [Isometry3::identity(); 2];
    /// posture.write_globals(&skelly, &Isometry3::identity(), &mut globals);
    /// let bone_global_old = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at (1, 0, 0).
    /// assert!((bone_global_old.translation.vector - Vector3::x()).magnitude() < EPSILON);
    ///
    /// // Translate root bone.
    /// // Global position of the `bone` attached to `root` has changed accordingly.
    /// posture.append_translation(root, Vector3::z().into());
    ///
    /// posture.write_globals(&skelly, &Isometry3::identity(), &mut globals);
    /// let bone_global_new = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space after root translation at (1, 0, 1).
    /// assert!((bone_global_new.translation.vector - (Vector3::x() + Vector3::z())).magnitude() < EPSILON);
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    #[track_caller]
    pub fn append_translation(&mut self, bone: usize, translation: Translation3<T>)
    where
        T: RealField,
    {
        self.joints[bone].translation *= translation;
    }

    /// Sets relative position for bone with specified id.
    /// Affects global position of all descendant bones.
    ///
    /// This method ignores current relative position of the bone.
    /// To apply translation to current relative poistion see [`Skelly::append_translation`].
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::{Skelly, Posture}, na::{Point3, Isometry3, UnitQuaternion, Vector3}, core::f32::{consts::PI, EPSILON}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut posture = Posture::new(&skelly);
    ///
    /// let mut globals = [Isometry3::identity(); 2];
    /// posture.write_globals(&skelly, &Isometry3::identity(), &mut globals);
    /// let bone_global_old = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at (1, 0, 0).
    /// assert!((bone_global_old.translation.vector - Vector3::x()).magnitude() < EPSILON);
    ///
    /// // Set new relative position for the `bone`.
    /// posture.set_position(bone, Vector3::z());
    ///
    /// posture.write_globals(&skelly, &Isometry3::identity(), &mut globals);
    /// let bone_global_new = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at new position (0, 0, 1).
    /// assert!((bone_global_new.translation.vector - Vector3::z()).magnitude() < EPSILON);
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    #[track_caller]
    pub fn set_position(&mut self, bone: usize, position: Vector3<T>) {
        self.joints[bone].translation = position.into();
    }

    /// Returns current bone position relative to parent.
    #[track_caller]
    pub fn get_position(&mut self, bone: usize) -> &Vector3<T>
    where
        T: RealField,
    {
        &self.joints[bone].translation.vector
    }

    /// Sets relative orientation for bone with specified id.
    /// Affects global position of all descendant bones.
    ///
    /// This method ignores current relative position of the bone.
    /// To apply translation to current relative poistion see [`Skelly::append_translation`].
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::{Skelly, Posture}, na::{Point3, Isometry3, UnitQuaternion, Vector3}, core::f32::{consts::PI, EPSILON}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut posture = Posture::new(&skelly);
    ///
    /// let mut globals = [Isometry3::identity(); 2];
    /// posture.write_globals(&skelly, &Isometry3::identity(), &mut globals);
    /// let bone_global_old = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at (1, 0, 0).
    /// assert!((bone_global_old.translation.vector - Vector3::x()).magnitude() < EPSILON);
    ///
    /// // Set new relative orientation for the `bone`.
    /// posture.set_orientation(root, UnitQuaternion::from_euler_angles(0.0, 0.0, PI / 2.0));
    ///
    /// posture.write_globals(&skelly, &Isometry3::identity(), &mut globals);
    /// let bone_global_new = globals[bone];
    ///
    /// // Ensure that bone is placed correctly in global space at new position (0, 0, 1).
    /// assert!((bone_global_new.translation.vector - Vector3::y()).magnitude() < EPSILON);
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if `bone` index is out of bounds.
    #[track_caller]
    pub fn set_orientation(&mut self, bone: usize, orientation: UnitQuaternion<T>) {
        self.joints[bone].rotation = orientation;
    }

    /// Returns current bone orientation relative to parent.
    #[track_caller]
    pub fn get_orientation(&mut self, bone: usize) -> &UnitQuaternion<T>
    where
        T: RealField,
    {
        &self.joints[bone].rotation
    }

    /// Returns current bone isometry relative to parent.
    #[track_caller]
    pub fn get_isometry(&mut self, bone: usize) -> &Isometry3<T>
    where
        T: RealField,
    {
        &self.joints[bone]
    }

    /// Fills slice of `Isometry3` with global isometries
    /// for each bone of the `skelly` in this posture.
    ///
    /// # Example
    ///
    /// ```
    /// # use {skelly::{Skelly, Posture}, na::{Point3, Vector3, Isometry3}};
    /// let mut skelly = Skelly::<f32>::new();
    /// let root = skelly.add_root(Point3::origin());
    /// let bone = skelly.attach(Vector3::x(), root);
    ///
    /// let mut posture = Posture::new(&skelly);
    ///
    /// // Animate the skelly by modifying posture iteratively.
    ///
    /// let mut globals = [Isometry3::identity(); 2];
    /// posture.write_globals(&skelly, &Isometry3::identity(), &mut globals);
    /// ```
    ///
    /// # Panics
    ///
    /// Panics if this posture is not compatible with the `skelly`.\
    /// To check for compatibility use [`Posture::is_compatible`].\
    /// One may use [`Posture`] with [`Skelly`] used to create that [`Posture`]
    /// (see [`Posture::new`]) as it is guaranteed to be compatible
    /// until new bone is added.
    pub fn write_globals<D>(
        &self,
        skelly: &Skelly<T, D>,
        skelly_global: &Isometry3<T>,
        globals: &mut [Isometry3<T>],
    ) where
        T: RealField,
    {
        assert_eq!(
            self.joints.len(),
            skelly.len(),
            "Posture is not compatible with the skelly"
        );

        self.joints
            .iter()
            .zip(&skelly.bones)
            .take(globals.len())
            .enumerate()
            .for_each(|(index, (isometry, bone))| match bone.parent {
                Some(parent) => {
                    debug_assert!(parent < index);
                    globals[index] = &globals[parent] * isometry;
                }
                None => {
                    globals[index] = skelly_global * isometry;
                }
            })
    }
}
