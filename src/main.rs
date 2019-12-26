use kiss3d::window::Window;

use ncollide3d::shape;
use nphysics3d::{joint, object, world};
use object::{Body, BodyPart};

mod control;

use control::Controller;

type F = f32;

type Color = nalgebra::Point3<F>;
type Point = nalgebra::Point3<F>;
type Vector = nalgebra::Vector3<F>;
type Isometry = nalgebra::Isometry3<F>;
type Force = nphysics3d::algebra::Force3<F>;

type BodyHandle = object::DefaultBodyHandle;
type MechanicalWorld = world::DefaultMechanicalWorld<F>;
type GeometricalWorld = world::DefaultGeometricalWorld<F>;
type ColliderSet = object::DefaultColliderSet<F>;
type JointConstraintSet = joint::DefaultJointConstraintSet<F>;
type ForceGeneratorSet = nphysics3d::force_generator::DefaultForceGeneratorSet<F>;
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

trait Graphics {
    fn lines(&self, body_set: &BodySet) -> Vec<Line>;
    fn render(&self, window: &mut Window, body_set: &BodySet) {
        for (p1, p2, color) in self.lines(body_set) {
            window.draw_line(&p1, &p2, &color)
        }
    }
}

pub struct Sensors {
    lin: F,   // m
    angle: F, // radius, 0 is up
}

pub struct Drivers {
    lin: F, // newton
}

struct Drone {
    body_handle: BodyHandle,
    controller: Controller,
}

impl Drone {
    fn step(&mut self, body_set: &mut BodySet) {
        // - compute sensors
        let lin = 0.0;
        let angle = 0.0;
        let sensors = Sensors { lin, angle };
        // - get drivers from control
        let mut drivers = Drivers { lin: 0.0 };
        self.controller.control(&sensors, &mut drivers);
        // - limit drivers
        drivers.lin = num::clamp(drivers.lin, -1.0, 1.0);
        // - compute driver forces
        let body = body_set.multibody_mut(self.body_handle).unwrap();
        body.apply_force(
            0,
            &Force::linear(v(0.0, 0.0, drivers.lin)),
            nphysics3d::algebra::ForceType::Force,
            true,
        );
        // - compute random wind forces
        // - compute drag forces
        // - apply forces and torques
    }
}

impl Graphics for Drone {
    fn lines(&self, body_set: &BodySet) -> Vec<Line> {
        let pos0 = body_set
            .multibody(self.body_handle)
            .unwrap()
            .link(0)
            .unwrap()
            .position();
        let pos1 = body_set
            .multibody(self.body_handle)
            .unwrap()
            .link(1)
            .unwrap()
            .position();

        vec![(pos0 * p(0.0, 0.0, 0.0), pos1 * p(0.0, 0.0, 0.0), green())]
    }
}

fn main() {
    let mut mechanical_world = MechanicalWorld::new(v(0.0, -9.81 / 5.0, 0.0));
    let mut geometrical_world = GeometricalWorld::new();
    let mut body_set = BodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joint_constraints = JointConstraintSet::new();
    let mut force_generators = ForceGeneratorSet::new();

    let radius = 0.1;
    let cuboid = shape::ShapeHandle::new(shape::Cuboid::new(v(radius, radius, radius)));
    let collider_desc = object::ColliderDesc::new(cuboid.clone()).density(1.0);

    let mut prism_joint = joint::PrismaticJoint::new(Vector::z_axis(), 0.0);
    prism_joint.enable_min_offset(-2.0);
    prism_joint.enable_max_offset(2.0);
    let mut prism_desc = object::MultibodyDesc::new(prism_joint)
        .mass(0.1)
        .body_shift(v(0.0, -0.1, 0.0));

    let revo = joint::RevoluteJoint::new(Vector::x_axis(), 0.0);
    prism_desc
        .add_child(revo)
        .set_mass(1.0)
        .set_body_shift(v(0.0, -1.0, 0.0));

    let prism = prism_desc.build();
    let prism_handle = body_set.insert(prism);

    colliders.insert(collider_desc.build(object::BodyPartHandle(prism_handle, 0)));
    colliders.insert(collider_desc.build(object::BodyPartHandle(prism_handle, 1)));

    let mut window = Window::new("ctrl");
    let mut ground = window.add_cube(1000.0, 0.0, 1000.0);
    ground.set_color(0.6, 0.3, 0.0);

    let mut drone = Drone {
        body_handle: prism_handle,
        controller: Controller::new(),
    };

    let mut camera = kiss3d::camera::ArcBall::new(p(10.0, 2.0, 0.0), p(0.0, 1.0, 0.0));
    camera.set_max_pitch(TAU * 0.25);

    while window.render_with_camera(&mut camera) {
        mechanical_world.step(
            &mut geometrical_world,
            &mut body_set,
            &mut colliders,
            &mut joint_constraints,
            &mut force_generators,
        );
        drone.step(&mut body_set);
        drone.render(&mut window, &body_set);
    }
}
