use std::{collections::BTreeMap, f32::consts::TAU};

use glam::{vec2, Mat4, Vec3};
use itertools::Itertools;
use slab::Slab;

use crate::{
    brush::{Brush, Face},
    edge::Edge3D,
    edgelist::{PolygonEdge, VerticalPlane},
    link::{LinkKind, NavmeshLink},
    span::Span,
    tree::BspTree,
    util::TOLERANCE,
};

#[derive(Debug, Clone)]
pub struct NavmeshSettings {
    pub max_step_height: f32,
    pub max_slope_cosine: f32,
    pub agent_radius: f32,
}

impl NavmeshSettings {
    pub fn new() -> Self {
        Self {
            max_step_height: 0.7,
            max_slope_cosine: 0.707,
            agent_radius: 0.2,
        }
    }
}

impl Default for NavmeshSettings {
    fn default() -> Self {
        Self::new()
    }
}

pub struct Navmesh {
    polygons: Slab<Face>,
    polygon_links: BTreeMap<usize, Vec<usize>>,
    links: Slab<NavmeshLink>,
    settings: NavmeshSettings,
}

impl Navmesh {
    pub fn new<'a>(
        settings: NavmeshSettings,
        brushes: impl IntoIterator<Item = (Mat4, &'a Brush)>,
    ) -> Self {
        let agent_radius = settings.agent_radius;

        let brushes = brushes
            .into_iter()
            .filter_map(|(transform, brush)| {
                // inflate and transform each brush
                let faces = brush
                    .faces()
                    .iter()
                    .map(|face| {
                        face.map(|p| transform.transform_point3(p + p.signum() * agent_radius))
                    })
                    .collect_vec();

                BspTree::build(&faces)
            })
            .collect_vec();

        let tree = brushes.into_iter().reduce(|mut brush, other| {
            brush.union(other);
            brush
        });

        let mut output_faces = Slab::new();
        for face in tree
            .map(|v| v.polygons())
            .into_iter()
            .flatten()
            .filter(|v| v.normal().dot(Vec3::Y) > settings.max_slope_cosine)
        {
            output_faces.insert(face);
        }

        let mut this = Self {
            settings,
            polygons: output_faces,
            links: Slab::new(),
            polygon_links: Default::default(),
        };

        this.generate_links();
        this
    }

    pub fn walkable_polygons(&self) -> impl Iterator<Item = (usize, &Face)> {
        self.polygons
            .iter()
            .filter(|(_, v)| v.normal().dot(Vec3::Y) > self.settings.max_slope_cosine)
    }

    pub fn closest_polygon(&self, point: Vec3) -> Option<(usize, Face)> {
        self.polygons
            .iter()
            .filter(|v| v.1.contains_point(point))
            .map(|v| (v.0, v.1, v.1.distance_to_plane(point)))
            // .filter(|v| v.2 >= -TOLERANCE)
            .min_by_key(|v| ordered_float::OrderedFloat(v.2))
            .map(|(index, &face, _)| (index, face))
    }

    pub fn generate_links(&mut self) {
        let mut edgeplanes: BTreeMap<_, EdgeLinkPlane> = BTreeMap::new();

        self.polygon_links.clear();
        self.links.clear();

        let mut create_link = |link: NavmeshLink| {
            let index = self.links.insert(link);
            self.polygon_links
                .entry(link.from())
                .or_default()
                .push(index);

            let index = self.links.insert(link.reverse());

            self.polygon_links.entry(link.to()).or_default().push(index);
        };

        // Assign edge to vertplanes
        for (id, face) in &self.polygons {
            for (p1, p2) in face.edges() {
                let edge = PolygonEdge::new(id, p1, p2);

                let plane = edge.as_vertical_plane();

                let canonical_plane = plane.canonicalize();

                let disc_angle = (((canonical_plane.angle + TAU) % TAU) * 1024.0).round() as u32;
                let distance = (canonical_plane.distance * 256.0).round() as i32;

                tracing::info!(disc_angle, distance);

                if plane.normal.dot(canonical_plane.normal) > 0.0 {
                    edgeplanes
                        .entry((disc_angle, distance))
                        .or_insert_with(|| EdgeLinkPlane::new(canonical_plane))
                        .front
                        .push(edge);
                } else {
                    edgeplanes
                        .entry((disc_angle, distance))
                        .or_insert_with(|| EdgeLinkPlane::new(canonical_plane))
                        .back
                        .push(edge);
                }
            }
        }

        for plane in edgeplanes.values() {
            for back_edge in &plane.back {
                let back_interval = plane.plane.coplanar_interval(back_edge);

                for front_edge in &plane.front {
                    let front_interval = plane.plane.coplanar_interval(front_edge);
                    let overlap = back_interval.intersect(front_interval);

                    if overlap.is_empty() {
                        continue;
                    }

                    let s = plane.plane.coplanar_edge(back_edge);
                    let d = plane.plane.coplanar_edge(front_edge);

                    // let e1_len = e1.1 - e1.0;
                    // let e2_len = e2.1 - e2.0;

                    // let m_s = e1_len.y / e1_len.x;
                    // let m_d = e2_len.y / e2_len.x;

                    let m_s = (s.1.y - s.0.y) / (s.1.x - s.0.x);
                    let m_d = (d.1.y - d.0.y) / (d.1.x - d.0.x);

                    let c_s = s.0.y - m_s * s.0.x;
                    let c_d = d.0.y - m_d * d.0.x;

                    let delta_m = m_d - m_s;
                    let delta_c = c_d - c_s;

                    if delta_m.abs() > TOLERANCE {
                        let walk_x = -delta_c / delta_m;

                        let step_up_x = (self.settings.max_step_height - delta_c) / delta_m;

                        let step_down_x = (-self.settings.max_step_height - delta_c) / delta_m;

                        let step_down = Span::new(walk_x.min(step_down_x), walk_x.max(step_down_x));
                        let step_up = Span::new(walk_x.min(step_up_x), walk_x.max(step_up_x));

                        let step_down = step_down.intersect(overlap);
                        let step_up = step_up.intersect(overlap);

                        if !step_down.is_empty() {
                            let s_low = vec2(step_down.min, m_s * step_down.min + c_s);
                            let s_high = vec2(step_down.max, m_s * step_down.max + c_s);

                            let d_low = vec2(step_down.min, m_d * step_down.min + c_d);
                            let d_high = vec2(step_down.max, m_d * step_down.max + c_d);

                            create_link(NavmeshLink::new(
                                front_edge.polygon(),
                                back_edge.polygon(),
                                LinkKind::StepUp(
                                    Edge3D::new(
                                        plane.coplanar_to_world(s_low),
                                        plane.coplanar_to_world(s_high),
                                    ),
                                    Edge3D::new(
                                        plane.coplanar_to_world(d_low),
                                        plane.coplanar_to_world(d_high),
                                    ),
                                ),
                            ));
                        }

                        if !step_up.is_empty() {
                            let s_low = vec2(step_up.min, m_s * step_up.min + c_s);
                            let s_high = vec2(step_up.max, m_s * step_up.max + c_s);

                            let d_low = vec2(step_up.min, m_d * step_up.min + c_d);
                            let d_high = vec2(step_up.max, m_d * step_up.max + c_d);

                            create_link(NavmeshLink::new(
                                front_edge.polygon(),
                                back_edge.polygon(),
                                LinkKind::StepUp(
                                    Edge3D::new(
                                        plane.coplanar_to_world(d_low),
                                        plane.coplanar_to_world(d_high),
                                    ),
                                    Edge3D::new(
                                        plane.coplanar_to_world(s_low),
                                        plane.coplanar_to_world(s_high),
                                    ),
                                ),
                            ));
                        }
                    } else if delta_c.abs() < self.settings.max_step_height {
                        let s1 = vec2(overlap.min, m_s * overlap.min + c_s);
                        let s2 = vec2(overlap.max, m_s * overlap.max + c_s);

                        let d1 = vec2(overlap.min, m_d * overlap.min + c_d);
                        let d2 = vec2(overlap.max, m_d * overlap.max + c_d);

                        if delta_c > TOLERANCE {
                            create_link(NavmeshLink::new(
                                back_edge.polygon(),
                                front_edge.polygon(),
                                LinkKind::StepUp(
                                    Edge3D::new(
                                        plane.coplanar_to_world(s1),
                                        plane.coplanar_to_world(s2),
                                    ),
                                    Edge3D::new(
                                        plane.coplanar_to_world(d1),
                                        plane.coplanar_to_world(d2),
                                    ),
                                ),
                            ));
                        } else if delta_c < -TOLERANCE {
                            create_link(NavmeshLink::new(
                                front_edge.polygon(),
                                back_edge.polygon(),
                                LinkKind::StepUp(
                                    Edge3D::new(
                                        plane.coplanar_to_world(d1),
                                        plane.coplanar_to_world(d2),
                                    ),
                                    Edge3D::new(
                                        plane.coplanar_to_world(s1),
                                        plane.coplanar_to_world(s2),
                                    ),
                                ),
                            ));
                        } else {
                            create_link(NavmeshLink::new(
                                front_edge.polygon(),
                                back_edge.polygon(),
                                LinkKind::Walk(Edge3D::new(
                                    plane.coplanar_to_world(s1),
                                    plane.coplanar_to_world(s2),
                                )),
                            ));
                        }
                    }
                }
            }
        }
    }

    pub fn polygons(&self) -> &Slab<Face> {
        &self.polygons
    }

    pub fn links(&self) -> &Slab<NavmeshLink> {
        &self.links
    }

    pub fn polygon_links(&self) -> &BTreeMap<usize, Vec<usize>> {
        &self.polygon_links
    }
}

#[derive(Debug)]
pub(crate) struct EdgeLinkPlane {
    plane: VerticalPlane,
    front: Vec<PolygonEdge>,
    back: Vec<PolygonEdge>,
}

impl std::ops::Deref for EdgeLinkPlane {
    type Target = VerticalPlane;

    fn deref(&self) -> &Self::Target {
        &self.plane
    }
}

impl EdgeLinkPlane {
    pub fn new(plane: VerticalPlane) -> Self {
        Self {
            plane,
            front: Vec::new(),
            back: Vec::new(),
        }
    }
}
