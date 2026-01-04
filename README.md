# Constructive

[![Crates](https://img.shields.io/crates/v/constructive?style=for-the-badge)](https://crates.io/crates/constructive)
[![Docs](https://img.shields.io/docsrs/constructive?style=for-the-badge)](https://docs.rs/constructive)

Constructive is a navigation mesh library for Rust that uses Constructive Solid Geometry to generate walkable surfaces from scene geometry. Instead of voxelization, it operates directly on BSP trees to produce clean polygon boundaries.

## Features

- Navmesh generation from arbitrary scene geometry
- Engine-agnostic core
- Overlapping navmesh support
- Any-angle pathfinding with path smoothing
- Automatic step links and level stitching
- [Ivy](https://github.com/ten3roberts/ivy) engine integration

## Mesh Generation

Boolean operations on BSP trees carve navigable regions from geometry. Surfaces are filtered by slope angle, and agent radius inflation ensures clearance around obstacles.

## Pathfinding

A* search finds routes across the polygon graph. Path smoothing eliminates unnecessary waypoints by computing edge intersections, producing the shortest valid route for natural-looking movement.

## Links

The library analyzes height differences between adjacent surfaces and creates step-up/step-down connections automatically. Stairs, ramps, and ledges become traversable without manual annotation.

## Integration

An optional Ivy plugin provides automatic navmesh regeneration when scene geometry changes, with debug visualization for the mesh, links, and paths.

## Examples

![image](https://github.com/user-attachments/assets/3e7ec903-ebc4-4422-b9b3-98fc76a59a17)

Rendered using [Ivy](https://github.com/ten3roberts/ivy)
