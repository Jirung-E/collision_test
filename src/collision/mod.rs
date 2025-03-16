mod convex_hull;
pub mod dynamic_collision;
pub mod static_collision;

pub use convex_hull::*;


pub struct CollisionDetails {
    pub normal: glam::Vec3A,
    pub penetration: f32,
    // pub contact_point: Vec<glam::Vec3A>,
}
