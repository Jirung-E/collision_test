#[derive(Debug, Clone)]
pub struct Sphere {
    pub center: glam::Vec3,
    pub radius: f32,
}

impl Sphere {
    pub fn check_point_collision(&self, point: &glam::Vec3A) -> bool {
        let center = glam::Vec3A::from(self.center);
        (point - center).length_squared() <= self.radius.powi(2)
    }

    pub fn inflated(&self, amound: f32) -> Sphere {
        Sphere {
            center: self.center,
            radius: self.radius + amound,
        }
    }
}
