use {
    macroquad::{
        camera::{set_camera, Camera as _, Camera3D},
        color::{
            Color, BLUE, DARKGRAY, GOLD, GREEN, LIME, MAGENTA, MAROON, ORANGE, PINK, RED, WHITE,
            YELLOW,
        },
        input::{
            is_key_pressed, is_mouse_button_down, is_mouse_button_pressed, mouse_position, KeyCode,
            MouseButton,
        },
        models::{draw_line_3d, draw_sphere},
        time::get_frame_time,
        window::{clear_background, next_frame, screen_height, screen_width},
    },
    na::{Isometry3, Point, Point3, Vector, Vector3},
    skelly::{
        ik::{fabrik::FabrikSolver, frik::FrikSolver, rotor::RotorSolver, StepResult},
        Posture, Skelly,
    },
    std::collections::VecDeque,
};

struct SlidingWindowCounter {
    window: usize,
    counters: VecDeque<usize>,
    total: usize,
}

impl SlidingWindowCounter {
    fn new(window: usize) -> Self {
        SlidingWindowCounter {
            window,
            counters: VecDeque::new(),
            total: 0,
        }
    }

    fn add(&mut self, count: usize) {
        if self.counters.is_empty() {
            self.next();
        }

        *self.counters.back_mut().unwrap() += count;
        self.total += count;
    }

    fn next(&mut self) {
        self.counters.push_back(0);

        while self.counters.len() > self.window {
            self.total -= self.counters.pop_front().unwrap();
        }
    }

    fn mean(&self) -> f32 {
        self.total as f32 / self.counters.len() as f32
    }
}

#[macroquad::main("ik-test")]
async fn main() {
    let mut skelly = Skelly::<f32, Color>::new();
    let mut index = skelly.add_root_with(Point::origin(), GOLD);
    index = skelly.attach_with(Vector3::z().into(), index, MAROON);
    index = skelly.attach_with(Vector3::z().into(), index, PINK);

    let mut fst = skelly.attach_with(Vector3::z().into(), index, ORANGE);
    fst = skelly.attach_with((-Vector3::x()).into(), fst, MAGENTA);
    fst = skelly.attach_with((-Vector3::x()).into(), fst, BLUE);

    let mut snd = skelly.attach_with(Vector3::z().into(), index, LIME);
    snd = skelly.attach_with(Vector3::x().into(), snd, YELLOW);
    snd = skelly.attach_with(Vector3::x().into(), snd, WHITE);

    let mut trd = skelly.attach_with(Vector3::z().into(), index, LIME);
    trd = skelly.attach_with(Vector3::z().into(), trd, YELLOW);
    trd = skelly.attach_with(Vector3::z().into(), trd, WHITE);

    let mut globals = vec![Isometry3::identity(); skelly.len()];

    let mut frik_solver = FrikSolver::<f32>::new(0.01);
    let mut fabrik_solver = FabrikSolver::<f32>::new(0.01);
    let mut rotor_solver = RotorSolver::<f32>::new(0.01);

    let mut camera = Camera3D::default();
    let mut fst_target = None;
    let mut snd_target = None;
    let mut trd_target = None;
    let mut frik_posture = Posture::new(&skelly);
    let mut fabrik_posture = Posture::new(&skelly);
    let mut rotor_posture = Posture::new(&skelly);

    camera.position.y += 5.0;

    let mut frik_steps = SlidingWindowCounter::new(500);
    let mut fabrik_steps = SlidingWindowCounter::new(500);
    let mut rotor_steps = SlidingWindowCounter::new(500);

    // let mut solver_wait_for = 1.0;
    let mut steps_report_wait_for = 5.0;

    loop {
        next_frame().await;

        let frame_time = get_frame_time();

        if is_key_pressed(KeyCode::Escape) {
            break;
        }

        let camera_matrix = camera.matrix();

        if is_mouse_button_pressed(MouseButton::Left) {
            let (x, y) = mouse_position();
            let (x, y) = (
                x * 2.0 / screen_width() - 1.0,
                1.0 - y * 2.0 / screen_height(),
            );

            let camera_matrix_inv = camera_matrix.inverse();
            let o = camera_matrix_inv.transform_point3(macroquad::math::Vec3::zero());
            let t = camera_matrix_inv.transform_point3(macroquad::math::Vec3::new(x, y, 0.999));
            let d = t - o;
            // let f = 3.0 / d.y;
            let f = -o.y / d.y;
            let x = d * f + o;

            let target = Point3::from(Vector::from([x.x, x.y, x.z]));
            fst_target = Some(target);
            frik_solver.set_position_goal(fst, target);
            fabrik_solver.set_position_goal(fst, target);
            rotor_solver.set_position_goal(fst, target);
            frik_steps.next();
            fabrik_steps.next();
            rotor_steps.next();
        }

        if is_mouse_button_pressed(MouseButton::Right) {
            let (x, y) = mouse_position();
            let (x, y) = (
                x * 2.0 / screen_width() - 1.0,
                1.0 - y * 2.0 / screen_height(),
            );

            let camera_matrix_inv = camera_matrix.inverse();
            let o = camera_matrix_inv.transform_point3(macroquad::math::Vec3::new(0.0, 0.0, 0.0));
            let t = camera_matrix_inv.transform_point3(macroquad::math::Vec3::new(x, y, 0.999));
            let d = t - o;
            let f = -o.y / d.y;
            let x = d * f + o;

            let target = Point::from(Vector::from([x.x, x.y, x.z]));
            snd_target = Some(target);
            frik_solver.set_position_goal(snd, target);
            fabrik_solver.set_position_goal(snd, target);
            rotor_solver.set_position_goal(snd, target);
            frik_steps.next();
            fabrik_steps.next();
            rotor_steps.next();
        }

        if is_mouse_button_pressed(MouseButton::Middle) {
            let (x, y) = mouse_position();
            let (x, y) = (
                x * 2.0 / screen_width() - 1.0,
                1.0 - y * 2.0 / screen_height(),
            );

            let camera_matrix_inv = camera_matrix.inverse();
            let o = camera_matrix_inv.transform_point3(macroquad::math::Vec3::new(0.0, 0.0, 0.0));
            let t = camera_matrix_inv.transform_point3(macroquad::math::Vec3::new(x, y, 0.999));
            let d = t - o;
            let f = -o.y / d.y;
            let x = d * f + o;

            let target = Point::from(Vector::from([x.x, x.y, x.z]));
            trd_target = Some(target);
            frik_solver.set_position_goal(trd, target);
            fabrik_solver.set_position_goal(trd, target);
            rotor_solver.set_position_goal(trd, target);
            frik_steps.next();
            fabrik_steps.next();
            rotor_steps.next();
        }

        // solver_wait_for -= frame_time;
        // while solver_wait_for < 0.0 {
        //     solver_wait_for += 0.5;

        if fst_target.is_some() {
            for _ in 0..1 {
                if let StepResult::Unsolved = frik_solver.solve_step(&skelly, &mut frik_posture) {
                    frik_steps.add(1);
                } else {
                    // println!("FRIK SOLVED");
                    break;
                }
            }
            for _ in 0..1 {
                if let StepResult::Unsolved = fabrik_solver.solve_step(&skelly, &mut fabrik_posture)
                {
                    fabrik_steps.add(1);
                } else {
                    // println!("FABRIK SOLVED");
                    break;
                }
            }
            for _ in 0..1 {
                if let StepResult::Unsolved = rotor_solver.solve_step(&skelly, &mut rotor_posture) {
                    rotor_steps.add(1);
                } else {
                    // println!("ROTOR SOLVED");
                    break;
                }
            }
        }
        // }

        steps_report_wait_for -= frame_time;
        while steps_report_wait_for < 0.0 {
            steps_report_wait_for += 5.0;

            println!("Mean steps count");
            println!("FRIK: {}", frik_steps.mean());
            println!("FABRIK: {}", fabrik_steps.mean());
            println!("ROTOR: {}", rotor_steps.mean());
        }

        set_camera(camera);

        clear_background(DARKGRAY);

        if let Some(fst_target) = fst_target {
            draw_sphere(
                macroquad::math::Vec3::new(fst_target.x, fst_target.y, fst_target.z),
                0.1,
                None,
                RED,
            );
        }

        if let Some(snd_target) = snd_target {
            draw_sphere(
                macroquad::math::Vec3::new(snd_target.x, snd_target.y, snd_target.z),
                0.1,
                None,
                GREEN,
            );
        }

        if let Some(trd_target) = trd_target {
            draw_sphere(
                macroquad::math::Vec3::new(trd_target.x, trd_target.y, trd_target.z),
                0.1,
                None,
                BLUE,
            );
        }
        draw_skelly(&skelly, &frik_posture, &mut globals, BLUE);
        draw_skelly(&skelly, &fabrik_posture, &mut globals, RED);
        draw_skelly(&skelly, &rotor_posture, &mut globals, GREEN);
    }
}

fn draw_skelly(
    skelly: &Skelly<f32, Color>,
    posture: &Posture<f32>,
    globals: &mut Vec<Isometry3<f32>>,
    color: Color,
) {
    posture.write_globals(skelly, &Isometry3::identity(), globals);

    for index in 0..skelly.len() {
        if let Some(parent) = skelly.get_parent(index) {
            let start = &globals[parent].translation.vector;
            let end = &globals[index].translation.vector;
            // let color = *skelly.get_userdata(index);
            draw_line_3d(
                macroquad::math::Vec3::new(start.x, start.y, start.z),
                macroquad::math::Vec3::new(end.x, end.y, end.z),
                color,
            );
        }
    }
}
