use crate::{BoundingBox, Sphere};
use super::{CollisionDetails, ConvexHull};


/// 움직이는 물체(self)와 움직이지 않는 물체(other)의 충돌 검사
pub trait DynamicCollision<T: ConvexHull> {
    fn check_dynamic_collision(&self, velocity: &glam::Vec3A, other: &T) -> bool;
    fn check_dynamic_collision_details(&self, velocity: &glam::Vec3A, other: &T) -> Option<CollisionDetails>;
}


impl DynamicCollision<BoundingBox> for BoundingBox {
    fn check_dynamic_collision(&self, velocity: &glam::Vec3A, other: &BoundingBox) -> bool {
        todo!()
    }

    fn check_dynamic_collision_details(&self, velocity: &glam::Vec3A, other: &BoundingBox) -> Option<CollisionDetails> {
        todo!()
    }
}

impl DynamicCollision<Sphere> for BoundingBox {
    fn check_dynamic_collision(&self, velocity: &glam::Vec3A, other: &Sphere) -> bool {
        todo!()
    }

    fn check_dynamic_collision_details(&self, velocity: &glam::Vec3A, other: &Sphere) -> Option<CollisionDetails> {
        todo!()
    }
}


impl DynamicCollision<BoundingBox> for Sphere {
    fn check_dynamic_collision(&self, velocity: &glam::Vec3A, other: &BoundingBox) -> bool {
        todo!()
    }

    fn check_dynamic_collision_details(&self, velocity: &glam::Vec3A, other: &BoundingBox) -> Option<CollisionDetails> {
        todo!()
    }
}

impl DynamicCollision<Sphere> for Sphere {
    fn check_dynamic_collision(&self, velocity: &glam::Vec3A, other: &Sphere) -> bool {
        todo!()
    }

    fn check_dynamic_collision_details(&self, velocity: &glam::Vec3A, other: &Sphere) -> Option<CollisionDetails> {
        todo!()
    }
}