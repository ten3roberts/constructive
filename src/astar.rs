use std::collections::{btree_map::Entry, BTreeMap, BTreeSet, BinaryHeap};

use glam::Vec3;
use itertools::Itertools;

use crate::{link::NavmeshLink, navmesh::Navmesh, util::TOLERANCE};

#[derive(Debug, Clone, Copy)]
pub struct Waypoint {
    target_polygon: usize,
    edge: Option<usize>,
    point: Vec3,
}

impl Waypoint {
    pub fn new(target_polygon: usize, edge: Option<usize>, point: Vec3) -> Self {
        Self {
            target_polygon,
            edge,
            point,
        }
    }

    pub fn edge(&self) -> Option<usize> {
        self.edge
    }

    pub fn target_polygon(&self) -> usize {
        self.target_polygon
    }

    pub fn point(&self) -> Vec3 {
        self.point
    }
}

pub fn astar<F>(navmesh: &Navmesh, start: Vec3, end: Vec3, heuristic: F) -> Option<Vec<Waypoint>>
where
    F: Fn(Vec3, Vec3) -> f32,
{
    let mut open = BinaryHeap::new();
    let (start_node, _) = navmesh.closest_polygon(start)?;
    let (end_node, _) = navmesh.closest_polygon(end)?;

    // Information of how a node was reached
    let mut backtraces: BTreeMap<_, Backtrace> = BTreeMap::new();
    let start = Backtrace::start(start_node, start, (heuristic)(start, end));

    // Push the fist node
    open.push(start);
    backtraces.insert(start_node, start);

    let mut closed = BTreeSet::new();
    let mut path = Vec::new();

    // Expand the node with the lowest total cost
    while let Some(current) = open.pop() {
        if closed.contains(&current.node) {
            continue;
        }

        // End found
        // Generate backtrace and terminate
        if current.node == end_node {
            contruct_backtrace(end, current.node, backtraces, &mut path);
            shorten(navmesh, &mut path);
            // resolve_clip(portals, path, info.agent_radius);

            return Some(path);
        }

        let end_rel = end - current.point;

        // Add all edges to the open list and update backtraces
        let portals = navmesh
            .polygon_links()
            .get(&current.node)
            .into_iter()
            .flatten()
            .filter_map(|&portal| {
                let link = &navmesh.links()[portal];
                if link.to() == current.node || closed.contains(&link.to()) {
                    return None;
                }

                assert_eq!(link.from(), current.node);

                // Distance to each of the nodes
                let (p1, p2) = (link.destination_edge().p1, link.destination_edge().p2);
                let midpoint = (p1 + p2) / 2.0;
                let p1_dist = (heuristic)(p1, end);
                let p2_dist = (heuristic)(p2, end);
                let midpoint_dist = (heuristic)(midpoint, end);

                // let p = if portal.normal().dot(end_rel) > 0.0 {
                //     portal.clip(current.point, end, info.agent_radius)
                let p = if let Some(p) = link
                    .destination_edge()
                    .intersect_ray_clipped(current.point, end - current.point)
                {
                    p
                } else {
                    midpoint
                };

                // let p = midpoint;
                let backtrace = Backtrace::new(portal, link, p, &current, (heuristic)(p, end));

                // Update backtrace
                // If the cost to this node is lower than previosuly found,
                // overwrite with the new backtrace.
                match backtraces.entry(backtrace.node) {
                    Entry::Occupied(mut val) => {
                        if val.get().total_cost > backtrace.total_cost {
                            val.insert(backtrace);
                        } else {
                            return None;
                        }
                    }
                    Entry::Vacant(entry) => {
                        entry.insert(backtrace);
                    }
                }

                Some(backtrace)
            });

        // Add the edges
        open.extend(portals);

        // The current node is now done and won't be revisited
        assert!(closed.insert(current.node))
    }

    None
}

fn contruct_backtrace(
    end: Vec3,
    mut current: usize,
    backtraces: BTreeMap<usize, Backtrace>,
    path: &mut Vec<Waypoint>,
) {
    path.clear();
    path.push(Waypoint::new(current, None, end));
    let mut prev = end;
    loop {
        // Backtrace backwards
        let node = backtraces[&current];

        if path.len() < 2 || prev.distance_squared(node.point) > TOLERANCE {
            path.push(Waypoint::new(node.node, node.portal, node.point));
        }

        prev = node.point;

        // Continue up the backtrace
        if let Some(prev) = node.prev {
            current = prev;
        } else {
            break;
        }
    }

    path.reverse();
}

#[derive(Debug, Copy, Clone, PartialEq)]
struct Backtrace {
    // Index to the portal
    node: usize,
    // The first side of the portal
    point: Vec3,
    portal: Option<usize>,
    prev: Option<usize>,
    start_cost: f32,
    total_cost: f32,
}

impl Backtrace {
    fn start(node: usize, point: Vec3, heuristic: f32) -> Self {
        Self {
            node,
            point,
            portal: None,
            prev: None,
            start_cost: 0.0,
            total_cost: heuristic,
        }
    }

    fn new(
        edge_index: usize,
        edge: &NavmeshLink,
        point: Vec3,
        prev: &Backtrace,
        heuristic: f32,
    ) -> Self {
        let start_cost = prev.start_cost + point.distance(prev.point);
        Self {
            node: edge.to(),
            portal: Some(edge_index),
            point,
            prev: Some(prev.node),
            start_cost,
            total_cost: start_cost + heuristic,
        }
    }
}

// Order by lowest total_cost
impl<'a> PartialOrd for Backtrace {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        other.total_cost.partial_cmp(&self.total_cost)
    }
}

impl<'a> Eq for Backtrace {}

impl<'a> Ord for Backtrace {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .total_cost
            .partial_cmp(&self.total_cost)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

fn resolve_clip(navmesh: &Navmesh, path: &mut [Waypoint], margin: f32) {
    if path.len() < 3 {
        return;
    }

    let a = &path[0];
    let c = &path[2];
    let b = &mut path[1];

    if let Some(portal) = b.edge {
        let link = &navmesh.links()[portal];
        let edge = link.destination_edge();
        let p = edge.p1;
        let q = edge.p2;

        if (b.point.distance(p) < margin + TOLERANCE) || (b.point.distance(q) < margin + TOLERANCE)
        {
            // let normal = portal.normal();
            // let a_inc = (a.point - b.point)
            //     .normalize_or_zero()
            //     .perp_dot(normal)
            //     .abs();

            // let c_inc = (c.point - b.point)
            //     .normalize_or_zero()
            //     .perp_dot(normal)
            //     .abs();

            // b.point += normal * margin * (c_inc - a_inc)
        }
    }

    // resolve_clip(portals, &mut path[1..], margin)
}

fn shorten(navmesh: &Navmesh, path: &mut [Waypoint]) {
    for _ in 0..100 {
        let mut shortened = 0;
        for i in 0..path.len() {
            let [a, b, c, ..] = &mut path[i..] else {
                break;
            };

            // let a = &path[0];
            // let b = &path[1];
            // let c = &path[2];

            if let Some(edge) = b.edge {
                let portal = navmesh.links()[edge];
                // c was directly visible from a
                let edge = portal.destination_edge();
                if let Some(p) = edge.intersect_ray_clipped(a.point, c.point - a.point) {
                    let prev = b.point;
                    if (prev.distance_squared(p)) > TOLERANCE {
                        path[i + 1].point = p;
                        shortened += 1;
                    }

                    // // // Try to shorten the next strip.
                    // // // If successful, retry shortening for this strip
                    // // if shorten(navmesh, &mut path[1..]) && prev.distance_squared(p) > TOLERANCE {
                    // //     shorten(navmesh, path);
                    // // }

                    // return shorten(navmesh, &mut path[1..]);
                }

                // return shorten(navmesh, &mut path[1..]);
            }
        }

        if shortened == 0 {
            break;
        }
    }

    // shorten(navmesh, &mut path[1..]);
    // return false;
}
