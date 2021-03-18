//! This module contains inverse-kinematic functionality for the skelly crate.

use {
    super::{IkSolver, Status, StepResult},
    crate::skelly::{Posture, Skelly},
    na::{Isometry3, Point3, RealField, Scalar, UnitQuaternion},
};

#[derive(Clone, Copy)]
struct IkGoal<T: Scalar> {
    bone: usize,
    position: Option<Point3<T>>,
    orientation: Option<UnitQuaternion<T>>,
}

pub struct RotorSolver<T: Scalar> {
    epsilon: T,
    min_len: usize,
    goals: Vec<IkGoal<T>>,

    // temp vectors. saved to keep allocation.
    queue: Vec<QueueItem<T>>,
    globals: Vec<Isometry3<T>>,
}

impl<T> Clone for RotorSolver<T>
where
    T: Scalar,
{
    fn clone(&self) -> Self {
        RotorSolver {
            epsilon: self.epsilon.clone(),
            min_len: self.min_len,
            goals: self.goals.clone(),
            queue: Vec::new(),
            globals: Vec::new(),
        }
    }

    fn clone_from(&mut self, source: &Self) {
        self.epsilon = source.epsilon.clone();
        self.min_len = source.min_len;
        self.goals = source.goals.clone();
    }
}

impl<T> IkSolver<T> for RotorSolver<T>
where
    T: RealField,
{
    fn new(error: T) -> Self {
        Self::new(error)
    }

    fn solve_step<D>(&mut self, skelly: &Skelly<T, D>, posture: &mut Posture<T>) -> StepResult<T> {
        self.solve_step(skelly, posture)
    }
}

impl<T> RotorSolver<T>
where
    T: Scalar,
{
    pub fn new(epsilon: T) -> Self {
        RotorSolver {
            goals: Vec::new(),
            min_len: 0,
            queue: Vec::new(),
            globals: Vec::new(),
            epsilon,
        }
    }

    pub fn set_position_goal(&mut self, bone: usize, position: Point3<T>)
    where
        T: Copy,
    {
        match self
            .goals
            .iter_mut()
            .skip_while(|goal| goal.bone != bone)
            .next()
        {
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
        match self
            .goals
            .iter_mut()
            .skip_while(|goal| goal.bone != bone)
            .next()
        {
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

    pub fn solve_step<D>(
        &mut self,
        skelly: &Skelly<T, D>,
        posture: &mut Posture<T>,
    ) -> StepResult<T>
    where
        T: RealField,
    {
        assert_eq!(skelly.len(), posture.len());
        assert!(self.min_len <= skelly.len());

        self.globals.resize_with(skelly.len(), Isometry3::identity);
        skelly.write_globals_for_posture(posture, &mut self.globals);

        let mut total = StepResult {
            error: T::zero(),
            status: Status::Solved,
        };

        for goal in &self.goals {
            if let Some(position) = goal.position {
                let tip = Point3::from(self.globals[goal.bone].translation.vector);

                // let error = tip.coords.metric_distance(&position.coords);
                // if error < self.epsilon {
                //     continue;
                // }
                if let Some(parent) = skelly.get_parent(goal.bone) {
                    enque(&mut self.queue, parent, tip, position);
                }
            }
        }

        while let Some((bone, tip, goal)) = deque(&mut self.queue) {
            let inv = self.globals[bone].translation.inverse();

            let mut tip_local = inv * tip;
            let goal_local = inv * goal;

            if tip_local.coords.magnitude_squared() < self.epsilon {
                continue;
            }

            if goal_local.coords.magnitude_squared() < self.epsilon {
                continue;
            }

            let rot = UnitQuaternion::rotation_between(&tip_local.coords, &goal_local.coords)
                // .unwrap_or_else(|| UnitQuaternion::from_euler_angles(T::one(), T::one(), T::one()));
                .unwrap_or_else(UnitQuaternion::identity);

            // rotate the joint
            posture.rotate(bone, &rot);
            tip_local = rot * tip_local;

            let error = tip_local.coords.metric_distance(&goal_local.coords);
            if error < self.epsilon {
                total = total.reduce(StepResult::solved(error));
                continue;
            }

            if let Some(parent) = skelly.get_parent(bone) {
                let tip = self.globals[bone].translation * tip_local;
                enque(&mut self.queue, parent, tip, goal);
            }
        }

        total
    }

    // fn solve_one_pos<D>(
    //     chain: &mut Vec<usize>,
    //     globals: &mut Vec<Isometry3<T>>,
    //     skelly: &Skelly<T, D>,
    //     posture: &mut Posture<T>,
    //     epsilon: T,
    //     bone: usize,
    //     goal: Point3<T>,
    // ) -> StepResult<T>
    // where
    //     T: RealField,
    // {
    //     skelly.make_chain(bone, chain);

    //     if chain.is_empty() {
    //         // For the root case, just move it to the target position.
    //         posture.get_joint_mut(bone).translation = goal.coords.into();
    //         return StepResult {
    //             error: T::zero(),
    //             status: Status::Solved,
    //         };
    //     }

    //     for &bone in chain.iter().rev() {
    //         let identity = Isometry3::identity();
    //         let last = globals.last().unwrap_or(&identity);
    //         let global = last * posture.get_joint(bone);
    //         globals.push(global);
    //     }

    //     let mut tip =
    //         globals.last().unwrap() * posture.get_joint(bone).translation * Point3::origin();

    //     let error = tip.coords.metric_distance(&goal.coords);
    //     if error < epsilon {
    //         return StepResult {
    //             error,
    //             status: Status::Solved,
    //         };
    //     }

    //     globals.reverse();

    //     for (chain_index, &bone) in chain.iter().enumerate() {
    //         let inv = globals[chain_index].inverse();

    //         let mut tip_local = inv * tip;
    //         if tip_local.coords.magnitude_squared() < epsilon {
    //             continue;
    //         }

    //         let goal_local = inv * goal;
    //         if goal_local.coords.magnitude_squared() < epsilon {
    //             continue;
    //         }

    //         let rot = UnitQuaternion::rotation_between(&tip_local.coords, &goal_local.coords)
    //             // .unwrap_or_else(|| UnitQuaternion::from_euler_angles(T::one(), T::one(), T::one()));
    //             .unwrap_or_else(UnitQuaternion::identity);

    //         // rotate the joint
    //         let local = posture.get_joint_mut(bone);
    //         *local *= rot;
    //         tip_local = rot * tip_local;

    //         let error = tip_local.coords.metric_distance(&goal_local.coords);
    //         if error < epsilon {
    //             return StepResult {
    //                 error,
    //                 status: Status::Solved,
    //             };
    //         }

    //         tip = globals[chain_index] * tip_local;
    //         continue;
    //     }

    //     let error = tip.coords.metric_distance(&goal.coords);
    //     StepResult {
    //         error,
    //         status: if error < epsilon {
    //             Status::Solved
    //         } else {
    //             Status::Unsolved
    //         },
    //     }
    // }
}

#[derive(Debug)]
struct QueueItem<T: Scalar> {
    bone: usize,
    tip: Point3<T>,
    goal: Point3<T>,
}

fn enque<T>(queue: &mut Vec<QueueItem<T>>, bone: usize, tip: Point3<T>, goal: Point3<T>)
where
    T: Scalar,
{
    let index = queue
        .binary_search_by(|item| item.bone.cmp(&bone))
        .unwrap_or_else(|x| x);

    queue.insert(index, QueueItem { bone, tip, goal });
}

fn deque<T>(queue: &mut Vec<QueueItem<T>>) -> Option<(usize, Point3<T>, Point3<T>)>
where
    T: RealField,
{
    let first = queue.pop()?;

    let mut count = T::one();
    let mut tip_sum = first.tip.coords;
    let mut goal_sum = first.goal.coords;
    while let Some(item) = queue.pop() {
        if item.bone != first.bone {
            queue.push(item);
            break;
        }

        count += T::one();
        tip_sum += item.tip.coords;
        goal_sum += item.goal.coords;
    }

    Some((
        first.bone,
        Point3::from(tip_sum / count),
        Point3::from(goal_sum / count),
    ))
}
