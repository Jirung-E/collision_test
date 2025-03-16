use crate::{BoundingBox, Sphere};
use super::{CollisionDetails, ConvexHull};


/// 움직이지 않는 물체끼리의 충돌 검사
pub trait StaticCollision<T: ConvexHull> {
    fn check_static_collision(&self, other: &T) -> bool;
    fn check_static_collision_details(&self, other: &T) -> Option<CollisionDetails>;
}


impl StaticCollision<BoundingBox> for BoundingBox {
    fn check_static_collision(&self, other: &BoundingBox) -> bool {
        if self.rotation().is_some() || other.rotation().is_some() {
            self.obb_collision(other)
        } else {
            self.aabb_collision(other)
        }
    }

    fn check_static_collision_details(&self, other: &BoundingBox) -> Option<CollisionDetails> {
        if self.rotation().is_some() || other.rotation().is_some() {
            self.obb_collision_details(other)
        } else {
            self.aabb_collision_details(other)
        }
    }
}

impl StaticCollision<Sphere> for BoundingBox {
    fn check_static_collision(&self, sphere: &Sphere) -> bool {
        // Sphere를 BoundingBox의 로컬 공간으로 변환
        let local_sphere_center = match self.rotation() {
            Some(rotation) => {
                let inv_rotation = rotation.transpose();    // 회전행렬의 전치행렬은 역행렬과 같다.
                let local_origin = inv_rotation * (sphere.center - self.center);
                local_origin
            }
            None => sphere.center - self.center,
        };

        let aabb_extents = self.extents();
        let mut distance_sq = 0.0;

        for i in 0..3 {
            // 각 축에 대해 Box에서 벗어난 거리 측정
            let dist = local_sphere_center[i].abs() - aabb_extents[i];
            if dist > 0.0 {
                // dist <= 0.0인 경우는 거리 0으로 처리
                distance_sq += dist.powi(2);
            }
        }

        distance_sq <= sphere.radius.powi(2)
    }

    fn check_static_collision_details(&self, sphere: &Sphere) -> Option<CollisionDetails> {
        // Sphere를 BoundingBox의 로컬 공간으로 변환
        let local_sphere_center = match self.rotation() {
            Some(rotation) => {
                let inv_rotation = rotation.transpose();    // 회전행렬의 전치행렬은 역행렬과 같다.
                let local_origin = inv_rotation * (sphere.center - self.center);
                local_origin
            }
            None => sphere.center - self.center,
        };

        let aabb_extents = self.extents();
        let mut to_center = glam::Vec3::ZERO;

        for i in 0..3 {
            // 각 축에 대해 Box에서 벗어난 거리 측정
            let dist = local_sphere_center[i].abs() - aabb_extents[i];
            if dist >= 0.0 {
                to_center[i] = local_sphere_center[i].signum() * dist;
            }
        }

        let to_center = glam::Vec3A::from(to_center);
        let penetration = sphere.radius - to_center.length();

        if penetration < 0.0 {
            return None;
        }
        
        let to_center = match self.rotation() {
            Some(rotation) => {
                rotation * to_center
            }
            None => to_center
        };
        let normal = -to_center.normalize_or_zero();
        
        Some(CollisionDetails {
            normal,
            penetration,
        })
    }
}


impl StaticCollision<BoundingBox> for Sphere {
    fn check_static_collision(&self, other: &BoundingBox) -> bool {
        other.check_static_collision(self)
    }

    fn check_static_collision_details(&self, other: &BoundingBox) -> Option<CollisionDetails> {
        let mut details = other.check_static_collision_details(self)?;
        details.penetration = -details.penetration;
        Some(details)
    }
}

impl StaticCollision<Sphere> for Sphere {
    fn check_static_collision(&self, other: &Sphere) -> bool {
        let center1 = glam::Vec3A::from(self.center);
        let center2 = glam::Vec3A::from(other.center);
        (center1 - center2).length_squared() <= (self.radius + other.radius).powi(2)
    }

    fn check_static_collision_details(&self, other: &Sphere) -> Option<CollisionDetails> {
        let center1 = glam::Vec3A::from(self.center);
        let center2 = glam::Vec3A::from(other.center);
        let normal = center1 - center2;
        let distance = normal.length();
        let penetration = self.radius + other.radius - distance;
        if penetration < 0.0 {
            return None;
        }

        Some(CollisionDetails {
            normal: normal.normalize_or_zero(),
            penetration,
        })
    }
}
