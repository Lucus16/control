#![allow(unused)]
use kiss3d::window::Window;
use nphysics3d::algebra::ForceType;
use rand::prelude::*;

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
const WIND_DELTA: F = 0.01;
const WIND_FACTOR: F = 0.999;
const BASE_DRAG: F = 0.05;
const POLE_DRAG: F = 1.0;

fn blue() -> Color {
    Color::new(0.5, 0.5, 1.0)
}

fn green() -> Color {
    Color::new(0.0, 1.0, 0.0)
}

fn yellow() -> Color {
    Color::new(1.0, 1.0, 0.0)
}

fn p(x: F, y: F, z: F) -> Point {
    Point::new(x, y, z)
}

fn v(x: F, y: F, z: F) -> Vector {
    Vector::new(x, y, z)
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
    fn step(&mut self, window: &mut Window, body_set: &mut BodySet, wind: Vector) -> bool {
        let mbody = body_set.multibody(self.body_handle).unwrap();
        let base = mbody.link(0).unwrap();
        let pole = mbody.link(1).unwrap();
        let base_pos = base.position() * p(0.0, 0.0, 0.0);
        let base_com = base.center_of_mass();
        let pole_pos = pole.position() * p(0.0, 0.0, 0.0);
        let pole_com = pole.center_of_mass();

        // - compute sensors TODO
        let lin = base_pos.z;
        let angle = pole_pos.z - base_pos.z;
        let reset = pole_pos.y < base_pos.y;
        let sensors = Sensors { lin, angle };
        // - get drivers from control
        let mut drivers = Drivers { lin: 0.0 };
        self.controller.control(&sensors, &mut drivers);
        // - limit drivers
        drivers.lin = num::clamp(drivers.lin, -1.0, 1.0);
        let lin_driver = v(0.0, 0.0, drivers.lin);
        let body = body_set.multibody_mut(self.body_handle).unwrap();
        // - compute driver forces
        body.apply_force(0, &Force::linear(lin_driver), ForceType::Force, true);
        // - compute drag forces relative to global wind TODO
        body.apply_force(0, &Force::linear(wind * BASE_DRAG), ForceType::Force, true);
        body.apply_force(1, &Force::linear(wind * POLE_DRAG), ForceType::Force, true);

        window.draw_line(&p(0.0, 0.01, -2.0), &p(0.0, 0.01, 2.0), &green());
        window.draw_line(&base_pos, &pole_pos, &green());
        window.draw_point(&base_com, &yellow());
        window.draw_point(&pole_com, &yellow());
        window.draw_line(&base_com, &(base_com + 10.0 * wind * BASE_DRAG), &blue());
        window.draw_line(&pole_com, &(pole_com + 10.0 * wind * POLE_DRAG), &blue());

        reset
    }
}

fn main() {
    let mut window = Window::new("ctrl");
    window.set_point_size(5.0);
    while sim(&mut window) {}
}

fn sim(window: &mut Window) -> bool {
    let mut rng = rand::thread_rng();

    let mut mechanical_world = MechanicalWorld::new(v(0.0, -9.81, 0.0));
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

    let mut ground = window.add_cube(1000.0, 0.0, 1000.0);
    ground.set_color(0.6, 0.3, 0.0);

    let mut drone = Drone {
        body_handle: prism_handle,
        controller: Controller::new(),
    };

    let mut camera = kiss3d::camera::ArcBall::new(p(4.0, 1.0, 0.0), p(0.0, 1.0, 0.0));
    camera.set_max_pitch(TAU * 0.25);

    let mut wind = v(0.0, 0.0, 0.0);

    while window.render_with_camera(&mut camera) {
        mechanical_world.step(
            &mut geometrical_world,
            &mut body_set,
            &mut colliders,
            &mut joint_constraints,
            &mut force_generators,
        );

        let wind_x_delta: F = rng.gen_range(-1.0, 1.0) * WIND_DELTA;
        let wind_z_delta: F = rng.gen_range(-1.0, 1.0) * WIND_DELTA;
        wind.x = (wind.x + wind_x_delta) * WIND_FACTOR;
        wind.x = 0.0;
        wind.z = (wind.z + wind_z_delta) * WIND_FACTOR;

        let reset = drone.step(window, &mut body_set, wind);
        if reset {
            return true;
        }
    }

    return false;
}
