use {
    macroquad::{
        camera::{set_camera, Camera as _, Camera3D},
        color::{
            Color, BLUE, DARKGRAY, GOLD, GREEN, LIME, MAGENTA, MAROON, ORANGE, PINK, RED, WHITE,
            YELLOW,
        },
        input::{is_key_pressed, is_mouse_button_down, mouse_position, KeyCode, MouseButton},
        models::{draw_line_3d, draw_sphere},
        time::get_frame_time,
        window::{clear_background, next_frame, screen_height, screen_width},
    },
    na::{Isometry3, Point, Point3, Vector, Vector3},
    skelly::{
        ik::{fabrik::FabrikSolver, rotor::RotorSolver},
        Posture, Skelly,
    },
};

#[macroquad::main("ik-test")]
async fn main() {
    let mut skelly = Skelly::<f32, Color>::new();
    let mut index = skelly.add_root_with(Point::origin(), GOLD);
    index = skelly.attach_with(Vector3::z().into(), index, MAROON);
    index = skelly.attach_with(Vector3::z().into(), index, PINK);

    let mut left = skelly.attach_with(Vector3::z().into(), index, ORANGE);
    left = skelly.attach_with((-Vector3::x()).into(), left, MAGENTA);
    left = skelly.attach_with((-Vector3::x()).into(), left, BLUE);

    let mut right = skelly.attach_with(Vector3::z().into(), index, LIME);
    right = skelly.attach_with(Vector3::x().into(), right, YELLOW);
    right = skelly.attach_with(Vector3::x().into(), right, WHITE);

    let mut globals = vec![Isometry3::identity(); skelly.len()];

    let mut fabrik_solver = FabrikSolver::<f32>::new(0.0001);
    let mut rotor_solver = RotorSolver::<f32>::new(0.0001);
    // solver.set_position_goal(index, Point::origin());
    // solver.set_position_goal(left, Point::origin());
    // solver.set_position_goal(right, Point::origin());

    let mut camera = Camera3D::default();
    let mut left_target = Point::origin();
    let mut right_target = Point::origin();
    let mut fabrik_posture = skelly.make_posture();
    let mut rotor_posture = skelly.make_posture();

    camera.position.y += 5.0;

    let mut solver_wait_for = 1.0;

    loop {
        if is_key_pressed(KeyCode::Escape) {
            break;
        }

        let camera_matrix = camera.matrix();

        if is_mouse_button_down(MouseButton::Left) {
            let (x, y) = mouse_position();
            let (x, y) = (
                x * 2.0 / screen_width() - 1.0,
                1.0 - y * 2.0 / screen_height(),
            );

            let camera_matrix_inv = camera_matrix.inverse();
            let o = camera_matrix_inv.transform_point3(macroquad::math::Vec3::zero());
            let t = camera_matrix_inv.transform_point3(macroquad::math::Vec3::new(x, y, 0.999));
            let d = t - o;
            let f = -o.y / d.y;
            let x = d * f + o;

            left_target = Point3::from(Vector::from([x.x, x.y, x.z]));
            fabrik_solver.set_position_goal(left, left_target);
            rotor_solver.set_position_goal(left, left_target);
        }

        if is_mouse_button_down(MouseButton::Right) {
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

            right_target = Point::from(Vector::from([x.x, x.y, x.z]));
            fabrik_solver.set_position_goal(right, right_target);
            rotor_solver.set_position_goal(right, right_target);
        }

        solver_wait_for -= get_frame_time();
        while solver_wait_for < 0.0 {
            let _solved = fabrik_solver.solve_step(&skelly, &mut fabrik_posture);
            let _solved = rotor_solver.solve_step(&skelly, &mut rotor_posture);
            let _solved = rotor_solver.solve_step(&skelly, &mut rotor_posture);
            solver_wait_for += 0.05;
        }

        // *posture.get_joint_mut(0) *= UnitQuaternion::from_euler_angles(0.1, 0.1, 0.1);

        set_camera(camera);

        next_frame().await;
        clear_background(DARKGRAY);

        draw_sphere(
            macroquad::math::Vec3::new(left_target.x, left_target.y, left_target.z),
            0.1,
            None,
            RED,
        );

        draw_sphere(
            macroquad::math::Vec3::new(right_target.x, right_target.y, right_target.z),
            0.1,
            None,
            GREEN,
        );
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
    skelly.write_globals_for_posture(posture, globals);

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
