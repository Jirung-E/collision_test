#[derive(Debug, Clone, Copy)]
pub struct BoundingBox {
    pub center: glam::Vec3,
    /// center로부터 x, y, z 방향으로 확장되는 길이  
    /// extents: (0.5, 0.5, 0.5) 인 경우 박스의 크기는 (1, 1, 1)  
    /// 음수는 허용하지 않음  
    extents: glam::Vec3,
    rotation: Option<glam::Mat3>, // OBB를 위함
}

impl Default for BoundingBox {
    fn default() -> Self {
        Self {
            center: glam::Vec3::ZERO,
            extents: glam::Vec3::ZERO,
            rotation: None, // default None B/C AABB가 default 인 상태
        }
    }
}

impl BoundingBox {
    /// Axis-Aligned Bounding Box 생성
    pub fn new(center: glam::Vec3, extents: glam::Vec3) -> Self {
        Self {
            center,
            extents: extents.abs(), // extents는 음수가 될 수 없음
            rotation: None,
        }
    }

    /// Oriented Bounding Box 생성
    pub fn new_rotated(center: glam::Vec3, extents: glam::Vec3, rotation: glam::Mat3) -> Self {
        Self {
            center,
            extents: extents.abs(), // extents는 음수가 될 수 없음
            rotation: Some(rotation),
        }
    }

    pub fn set_rotation(&mut self, rotation: glam::Mat3) {
        self.rotation = Some(rotation);
    }

    pub fn rotation(&self) -> Option<glam::Mat3> {
        self.rotation
    }

    pub fn extents(&self) -> glam::Vec3 {
        self.extents
    }

    pub fn center(&self) -> glam::Vec3 {
        self.center
    }

    // AABB collision detection
    pub fn aabb_collision(&self, other: &BoundingBox) -> bool {
        let x_overlap = (self.center.x - other.center.x).abs() <= (self.extents.x + other.extents.x);
        let y_overlap = (self.center.y - other.center.y).abs() <= (self.extents.y + other.extents.y);
        let z_overlap = (self.center.z - other.center.z).abs() <= (self.extents.z + other.extents.z);

        x_overlap && y_overlap && z_overlap
    }

    // SAT 를 이용한 OBB collision detection
    pub fn obb_collision(&self, other: &BoundingBox) -> bool {
        let self_axes = self.get_axes();
        let other_axes = other.get_axes();

        // cross products > vector
        let cross_products: [glam::Vec3A; 9] = [
            self_axes[0].cross(other_axes[0]),
            self_axes[0].cross(other_axes[1]),
            self_axes[0].cross(other_axes[2]),
            self_axes[1].cross(other_axes[0]),
            self_axes[1].cross(other_axes[1]),
            self_axes[1].cross(other_axes[2]),
            self_axes[2].cross(other_axes[0]),
            self_axes[2].cross(other_axes[1]),
            self_axes[2].cross(other_axes[2]),
        ];

        // 모든 축 확인
        let axes_to_test = self_axes.iter()
            .chain(other_axes.iter())       // 양 OBB의 지역 축
            .chain(cross_products.iter());  // Cross product 축

        let vbox1 = VertexBox::from(self);
        let vbox2 = VertexBox::from(other);

        for axis in axes_to_test {
            if !vbox1.overlaps_on_axis(&vbox2, axis) {
                return false; // if 분리된 축이 존재 = 충돌 없음
            }
        }

        true // 분리된 축 없음 = 충돌
    }

    // OBB의 지역 축 가져오기 (회전 행렬의 열)
    fn get_axes(&self) -> [glam::Vec3A; 3] {
        let rotation = self.rotation.unwrap_or(glam::Mat3::IDENTITY);
        [
            glam::Vec3A::from(rotation.x_axis),
            glam::Vec3A::from(rotation.y_axis),
            glam::Vec3A::from(rotation.z_axis),
        ]
    }

    // 월드 공간에서 OBB의 정점 가져오기
    pub fn get_vertices(&self) -> [glam::Vec3A; 8] {
        let center = glam::Vec3A::from(self.center);
        let extents = glam::Vec3A::from(self.extents);
        let vertices = [
            glam::Vec3A::new(1.0, 1.0, 1.0) * extents,
            glam::Vec3A::new(-1.0, 1.0, 1.0) * extents,
            glam::Vec3A::new(1.0, -1.0, 1.0) * extents,
            glam::Vec3A::new(-1.0, -1.0, 1.0) * extents,
            glam::Vec3A::new(1.0, 1.0, -1.0) * extents,
            glam::Vec3A::new(-1.0, 1.0, -1.0) * extents,
            glam::Vec3A::new(1.0, -1.0, -1.0) * extents,
            glam::Vec3A::new(-1.0, -1.0, -1.0) * extents,
        ];

        if let Some(rotation) = self.rotation {
            vertices.map(|v| center + rotation * v)
        } else {
            vertices.map(|v| center + v)
        }
    }
}


pub struct VertexBox {
    vertices: [glam::Vec3A; 8],
}

impl From<&BoundingBox> for VertexBox {
    fn from(boundingbox: &BoundingBox) -> Self {
        Self {
            vertices: boundingbox.get_vertices(),
        }
    }
}

impl VertexBox {
    // OBB를 축에 투영하고 투영 간격(최소, 최대)을 반환하는 메서드
    fn project_onto_axis(&self, axis: &glam::Vec3A) -> (f32, f32) {
        //각 정점을 축에 투영하는 dot product 계산
        let projections: [f32; 8] = [
            axis.dot(self.vertices[0]),
            axis.dot(self.vertices[1]),
            axis.dot(self.vertices[2]),
            axis.dot(self.vertices[3]),
            axis.dot(self.vertices[4]),
            axis.dot(self.vertices[5]),
            axis.dot(self.vertices[6]),
            axis.dot(self.vertices[7]),
        ];

        let min_proj = *projections.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
        let max_proj = *projections.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();

        (min_proj, max_proj)
    }

    // 두 OBB가 주어진 축에서 겹치는지 확인
    fn overlaps_on_axis(&self, other: &VertexBox, axis: &glam::Vec3A) -> bool {
        let (min_a, max_a) = self.project_onto_axis(axis);
        let (min_b, max_b) = other.project_onto_axis(axis);

        // 축에서 투영이 겹치는지 확인
        max_a >= min_b && max_b >= min_a
    }

    pub fn get_vertices(&self) -> &[glam::Vec3A; 8] {
        &self.vertices
    }
}