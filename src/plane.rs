use glam::{Vec3, Vec4, Vec4Swizzles};

use crate::{
    brush::{Face, FaceIntersect},
    util::TOLERANCE,
};

#[derive(Debug, Copy, Clone)]
pub struct Plane {
    pub normal: Vec3,
    pub distance: f32,
}

impl Plane {
    pub fn new(normal: Vec3, distance: f32) -> Self {
        Self { normal, distance }
    }

    pub fn from_face(face: Face) -> Self {
        let normal = face.normal();
        assert!(normal.is_finite());
        let distance = face.p1.dot(normal);

        Self { normal, distance }
    }

    pub fn distance_to_point(&self, point: Vec3) -> f32 {
        point.dot(self.normal) - self.distance
    }

    pub fn intersect_ray(&self, ray_origin: Vec3, ray_direction: Vec3) -> Option<f32> {
        let denom = self.normal.dot(ray_direction);
        if denom.abs() > f32::EPSILON {
            let t = (self.normal * self.distance - ray_origin).dot(self.normal) / denom;
            if t >= 0.0 {
                return Some(t);
            }
        }

        None
    }

    pub fn classify_face(&self, face: Face) -> FaceIntersect {
        let d1 = self.distance_to_point(face.p1);
        let d2 = self.distance_to_point(face.p2);
        let d3 = self.distance_to_point(face.p3);

        if d1.abs() <= TOLERANCE && d2.abs() <= TOLERANCE && d3.abs() <= TOLERANCE {
            if face.normal().dot(self.normal) > 0.0 {
                return FaceIntersect::CoplanarFront;
            } else {
                return FaceIntersect::CoplanarBack;
            }
        }

        if d1 >= -TOLERANCE && d2 >= -TOLERANCE && d3 >= -TOLERANCE {
            FaceIntersect::Front
        } else if d1 <= TOLERANCE && d2 <= TOLERANCE && d3 <= TOLERANCE {
            FaceIntersect::Back
        } else {
            FaceIntersect::Intersect
        }
    }

    pub fn split_face(
        &self,
        face: Face,
        front_result: &mut Vec<Face>,
        back_result: &mut Vec<Face>,
    ) {
        let mut front_count = 0;
        let mut back_count = 0;
        let mut coplanar_count = 0;

        let mut front = [Vec4::NAN; 3];
        let mut back = [Vec4::NAN; 3];
        let mut coplanar = [Vec3::NAN; 3];

        for p in face.points() {
            let distance = self.distance_to_point(p);
            if distance >= TOLERANCE {
                front[front_count] = p.extend(distance);
                front_count += 1;
            } else if distance <= -TOLERANCE {
                back[back_count] = p.extend(distance);
                back_count += 1;
            } else {
                coplanar[coplanar_count] = p;
                coplanar_count += 1;
            }
        }

        let normal = face.normal();
        let orient = |face: Face| {
            if face.normal().dot(normal) < 0.0 {
                Face::new(face.p3, face.p2, face.p1)
            } else {
                face
            }
        };

        if coplanar_count == 1 {
            assert_eq!(back_count, 1);
            assert_eq!(front_count, 1);
            let back = back[0];
            let front = front[0];
            let coplanar = coplanar[0];

            let i1 = back.xyz().lerp(front.xyz(), back.w / (back.w - front.w));

            front_result.push(orient(Face::new(coplanar, front.xyz(), i1)));
            back_result.push(orient(Face::new(coplanar, i1, back.xyz())));
        } else if front_count == 1 && back_count == 2 {
            // One point in front, two in back
            let f = front[0].xyz();

            let back1 = back[0].xyz();
            let back2 = back[1].xyz();

            let i1 = f.lerp(back1, front[0].w / (front[0].w - back[0].w));
            let i2 = f.lerp(back2, front[0].w / (front[0].w - back[1].w));

            front_result.push(orient(Face::new(f, i1, i2)));
            back_result.push(orient(Face::new(back1, back2, i1)));
            back_result.push(orient(Face::new(i1, back2, i2)));
        } else if front_count == 2 && back_count == 1 {
            let b = back[0].xyz();
            let front1 = front[0].xyz();
            let front2 = front[1].xyz();

            // Two points in front, one in back
            let t1 = back[0].w / (back[0].w - front[0].w);
            let t2 = back[0].w / (back[0].w - front[1].w);

            let i1 = b.lerp(front1, t1);
            let i2 = b.lerp(front2, t2);

            back_result.push(orient(Face::new(b, i1, i2)));
            front_result.push(orient(Face::new(front1, front2, i1)));
            front_result.push(orient(Face::new(i1, front2, i2)));
        }
    }

    pub(crate) fn invert(&self) -> Self {
        Self {
            normal: -self.normal,
            distance: -self.distance,
        }
    }
}
