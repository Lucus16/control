use kiss3d::window::Window;

use nphysics3d::{joint, object, world};

mod control;

type F = f32;

type Color = nalgebra::Point3<F>;
type Point = nalgebra::Point3<F>;
type Vector = nalgebra::Vector3<F>;
type Isometry = nalgebra::Isometry3<F>;

type BodyHandle = object::DefaultBodyHandle;
type MechanicalWorld = world::DefaultMechanicalWorld<F>;
type BodySet = object::DefaultBodySet<F>;

type Line = (Point, Point, Color);

const TAU: F = 6.283185307179586;

fn green() -> Color {
    Color::new(0.0, 1.0, 0.0)
}

fn p(x: F, y: F, z: F) -> Point {
    Point::new(x, y, z)
}

fn v(x: F, y: F, z: F) -> Vector {
    Vector::new(x, y, z)
}

fn gravity() -> Vector {
    Vector::y() * -9.81
}

trait Graphics {
    fn position(&self, body_set: &BodySet) -> Isometry;
    fn lines(&self) -> Vec<Line>;
    fn render(&self, window: &mut Window, body_set: &BodySet) {
        let pos = self.position(body_set);
        for (p1, p2, color) in self.lines() {
            window.draw_line(&(pos * p1), &(pos * p2), &color)
        }
    }
}

struct DroneSensors {
    x: F,      // m
    y: F,      // m
    z: F,      // m
    vx: F,     // m/s
    vy: F,     // m/s
    vz: F,     // m/s
    pitch: F,  // rad, 0 is level
    roll: F,   // rad, 0 is level
    yaw: F,    // rad, 0 is north
    vpitch: F, // rad/s
    vroll: F,  // rad/s
    vyaw: F,   // rad/s
}

struct DroneDrivers {
    fl: F, // pwm duty cycle
    fr: F, // pwm duty cycle
    bl: F, // pwm duty cycle
    br: F, // pwm duty cycle
}

struct Drone {
    body_handle: BodyHandle,
}

impl Drone {
    fn step(&mut self) {
        // - compute sensors
        // - get motors from control
        // - limit motors
        // - compute motor forces
        // - compute drag forces
        // - apply forces and torques
    }
}

impl Graphics for Drone {
    fn position(&self, body_set: &BodySet) -> Isometry {
        *body_set.rigid_body(self.body_handle).unwrap().position()
    }

    fn lines(&self) -> Vec<Line> {
        vec![
            (p(0.3, 0.0, 0.3), p(-0.3, 0.0, -0.3), green()),
            (p(0.3, 0.0, -0.3), p(-0.3, 0.0, 0.3), green()),
            // (p(0.3, 0.0, 0.3), p(0.3, 0.1, 0.3), green()),
            // (p(0.3, 0.0, -0.3), p(0.3, 0.1, -0.3), green()),
            // (p(-0.3, 0.0, 0.3), p(-0.3, 0.1, 0.3), green()),
            // (p(-0.3, 0.0, -0.3), p(-0.3, 0.1, -0.3), green()),
        ]
    }
}

fn main() {
    let mut window = Window::new("ctrl");
    let mut ground = window.add_cube(1000.0, 0.0, 1000.0);
    ground.set_color(0.6, 0.3, 0.0);

    let mut mechanical_world = MechanicalWorld::new(gravity());
    let mut geometrical_world = world::DefaultGeometricalWorld::new();
    let mut body_set = BodySet::new();
    let mut colliders = object::DefaultColliderSet::new();
    let mut joint_constraints = joint::DefaultJointConstraintSet::new();
    let mut force_generators = nphysics3d::force_generator::DefaultForceGeneratorSet::new();

    let drone_body = object::RigidBodyDesc::new()
        .translation(v(0.0, 1.0, 0.0))
        .mass(1.0)
        .build();

    let mut drone = Drone {
        body_handle: body_set.insert(drone_body),
    };

    let mut camera = kiss3d::camera::ArcBall::new(p(4.0, 2.0, 0.0), p(0.0, 1.0, 0.0));
    camera.set_max_pitch(TAU * 0.25);

    while window.render_with_camera(&mut camera) {
        mechanical_world.step(
            &mut geometrical_world,
            &mut body_set,
            &mut colliders,
            &mut joint_constraints,
            &mut force_generators,
        );
        drone.step();
        drone.render(&mut window, &body_set);
    }
}
