//! This module contains inverse-kinematic functionality for the skelly crate.

use {
    super::{IkSolver, StepResult},
    crate::skelly::{Posture, Skelly},
    na::{Isometry3, Point3, RealField, Scalar, UnitQuaternion},
};

#[derive(Clone, Copy)]
struct IkGoal<T: Scalar> {
    bone: usize,
    position: Option<Point3<T>>,
    orientation: Option<UnitQuaternion<T>>,
}
pub struct FrikSolver<T: Scalar> {
    epsilon: T,
    min_len: usize,
    goals: Vec<IkGoal<T>>,

    // temp vectors. saved to keep allocation.
    forward_queue: Vec<QueueItem<T>>,
    globals: Vec<Isometry3<T>>,
}

impl<T> Clone for FrikSolver<T>
where
    T: Scalar,
{
    fn clone(&self) -> Self {
        FrikSolver {
            epsilon: self.epsilon.clone(),
            min_len: self.min_len,
            goals: self.goals.clone(),
            forward_queue: Vec::new(),
            globals: Vec::new(),
        }
    }

    fn clone_from(&mut self, source: &Self) {
        self.epsilon = source.epsilon.clone();
        self.min_len = source.min_len;
        self.goals = source.goals.clone();
    }
}

impl<T> IkSolver<T> for FrikSolver<T>
where
    T: RealField + Copy,
{
    fn new(error: T) -> Self {
        Self::new(error)
    }

    fn solve_step<D>(&mut self, skelly: &Skelly<T, D>, posture: &mut Posture<T>) -> StepResult {
        self.solve_step(skelly, posture)
    }
}

impl<T> FrikSolver<T>
where
    T: Scalar,
{
    pub fn new(epsilon: T) -> Self {
        FrikSolver {
            goals: Vec::new(),
            min_len: 0,
            forward_queue: Vec::new(),
            globals: Vec::new(),
            epsilon,
        }
    }

    pub fn set_position_goal(&mut self, bone: usize, position: Point3<T>)
    where
        T: Copy,
    {
        match self.goals.iter_mut().find(|goal| goal.bone == bone) {
            Some(goal) => {
                if goal.bone == bone {
                    goal.position = Some(position);
                }
            }
            None => {
                self.min_len = self.min_len.min(bone + 1);
                self.goals.push(IkGoal {
                    bone,
                    position: Some(position),
                    orientation: None,
                })
            }
        }
    }

    pub fn set_orientation_goal(&mut self, bone: usize, orientation: UnitQuaternion<T>)
    where
        T: Copy,
    {
        match self.goals.iter_mut().find(|goal| goal.bone == bone) {
            Some(goal) => {
                if goal.bone == bone {
                    goal.orientation = Some(orientation);
                }
            }
            None => {
                self.min_len = self.min_len.min(bone + 1);
                self.goals.push(IkGoal {
                    bone,
                    position: None,
                    orientation: Some(orientation),
                })
            }
        }
    }

    pub fn solve_step<D>(&mut self, skelly: &Skelly<T, D>, posture: &mut Posture<T>) -> StepResult
    where
        T: RealField + Copy,
    {
        assert!(posture.is_compatible(skelly));
        assert!(self.min_len <= skelly.len());

        self.globals.resize_with(skelly.len(), Isometry3::identity);
        posture.write_globals(skelly, &Isometry3::identity(), &mut self.globals);

        self.forward_queue.clear();

        let mut total_error = T::zero();

        // enque effectors
        for goal in &self.goals {
            if let Some(position) = goal.position {
                let effector = Point3::from(self.globals[goal.bone].translation.vector);

                let error = position.coords.metric_distance(&effector.coords);
                total_error += error;

                if let Some(parent) = skelly.get_parent(goal.bone) {
                    enque(&mut self.forward_queue, parent, effector, position);
                }
            }
        }

        if total_error < self.epsilon {
            return StepResult::Solved;
        }

        // Traverse from effectors to roots.
        while let Some((bone, effector, target)) = deque(&mut self.forward_queue) {
            let global = &self.globals[bone];
            let inverse = global.inverse();

            let old_effector_local = inverse * effector;
            let target_local = inverse * target;

            let required_rotation =
                UnitQuaternion::rotation_between(&old_effector_local.coords, &target_local.coords)
                    .unwrap_or_else(UnitQuaternion::identity);

            posture.append_rotation(bone, required_rotation);

            let required_rotation_child = required_rotation.inverse();
            for child in skelly.iter_children(bone) {
                let new_orientation = required_rotation_child * posture.get_orientation(child);
                posture.set_orientation(child, new_orientation);
            }

            let new_effector_local = required_rotation * old_effector_local;
            let new_target_local = target_local - new_effector_local;

            if let Some(parent) = skelly.get_parent(bone) {
                enque(
                    &mut self.forward_queue,
                    parent,
                    Point3::from(global.translation.vector),
                    global * Point3::from(new_target_local),
                );
            }
        }

        StepResult::Unsolved
    }
}

struct QueueItem<T: Scalar> {
    bone: usize,
    effector: Point3<T>,
    target: Point3<T>,
}

fn enque<T>(queue: &mut Vec<QueueItem<T>>, bone: usize, effector: Point3<T>, target: Point3<T>)
where
    T: Scalar,
{
    let index = queue
        .binary_search_by(|item| item.bone.cmp(&bone))
        .unwrap_or_else(|x| x);

    queue.insert(
        index,
        QueueItem {
            bone,
            effector,
            target,
        },
    );
}

fn deque<T>(queue: &mut Vec<QueueItem<T>>) -> Option<(usize, Point3<T>, Point3<T>)>
where
    T: RealField + Copy,
{
    let first = queue.pop()?;
    let mut count = T::one();

    let mut effector_sum = first.effector.coords;
    let mut target_sum = first.target.coords;
    while let Some(item) = queue.pop() {
        if item.bone != first.bone {
            queue.push(item);
            break;
        }

        count += T::one();
        effector_sum += item.effector.coords;
        target_sum += item.target.coords;
    }

    let effector = Point3::from(effector_sum / count);
    let target = Point3::from(target_sum / count);

    Some((first.bone, effector, target))
}
