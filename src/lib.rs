mod convex_hull;
mod bounds;
mod sphere;

pub use convex_hull::ConvexHull;
pub use bounds::BoundingBox;
pub use sphere::Sphere;


pub struct CollisionDetails {
    pub normal: glam::Vec3A,
    pub penetration: f32,
    // pub contact_point: Vec<glam::Vec3A>,
}


impl BoundingBox {
    pub fn check_boundingbox_collision(&self, other: &BoundingBox) -> bool {
        if self.rotation().is_some() || other.rotation().is_some() {
            self.obb_collision(other)
        } else {
            self.aabb_collision(other)
        }
    }
        
    pub fn check_sphere_collision(&self, sphere: &Sphere) -> bool {
        // Sphere를 BoundingBox의 로컬 공간으로 변환
        let local_sphere_center = match self.rotation() {
            Some(rotation) => {
                let inv_rotation = rotation.transpose();    // 회전행렬의 전치행렬은 역행렬과 같다.
                let local_origin = inv_rotation * (sphere.center - self.center);
                local_origin
            }
            None => sphere.center - self.center,
        };

        Self::check_aabb_sphere_collision(
            &self.extents(), 
            &local_sphere_center, 
            sphere.radius
        )
    }

    /// 원점에 위치하는 회전이 없는 BoundingBox와 Sphere의 충돌 체크
    fn check_aabb_sphere_collision(aabb_extents: &glam::Vec3, center: &glam::Vec3, radius: f32) -> bool {
        let mut distance_sq = 0.0;

        for i in 0..3 {
            // 각 축에 대해 Box에서 벗어난 거리 측정
            let dist = center[i].abs() - aabb_extents[i];
            if dist > 0.0 {
                // dist <= 0.0인 경우는 거리 0으로 처리
                distance_sq += dist.powi(2);
            }
        }

        distance_sq <= radius.powi(2)
    }
}

impl Sphere {
    pub fn check_sphere_collision(&self, other: &Sphere) -> bool {
        let center1 = glam::Vec3A::from(self.center);
        let center2 = glam::Vec3A::from(other.center);
        (center1 - center2).length_squared() <= (self.radius + other.radius).powi(2)
    }

    pub fn check_sphere_collision_details(&self, other: &Sphere) -> Option<CollisionDetails> {
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

    pub fn check_boundingbox_collision(&self, other: &BoundingBox) -> bool {
        other.check_sphere_collision(self)
    }
}
