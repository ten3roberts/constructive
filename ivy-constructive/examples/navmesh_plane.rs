use constructive::{astar::astar, brush::Brush, navmesh::NavmeshSettings};
use glam::{vec3, Mat4, Quat, Vec2, Vec3};
use itertools::Itertools;
use ivy_constructive::{
    components::{self, navmesh},
    plugin::{NavmeshDebugPlugin, NavmeshPlugin},
};
use ivy_engine::{
    engine,
    flax::{Entity, System, World},
    gizmos,
    input::layer::InputLayer,
    ivy_assets::AssetCache,
    ivy_core::{
        gizmos::{Line, Sphere},
        palette::{Srgb, Srgba},
        update_layer::{FixedTimeStep, Plugin, ScheduledLayer},
        Color, ColorExt, EngineLayer, EntityBuilderExt, DEG_45,
    },
    ivy_game::{
        orbit_camera::OrbitCameraPlugin,
        ray_picker::RayPickingPlugin,
        viewport_camera::{CameraSettings, ViewportCameraLayer},
    },
    ivy_graphics::{
        mesh::{MeshData, NORMAL_ATTRIBUTE, POSITION_ATTRIBUTE, TEX_COORD_ATTRIBUTE},
        texture::TextureData,
    },
    ivy_wgpu::{
        components::forward_pass,
        driver::WinitDriver,
        layer::GraphicsLayer,
        light::{LightBundle, LightKind, LightParams},
        material_desc::{MaterialData, PbrMaterialData},
        mesh_desc::MeshDesc,
        primitives::UvSpherePrimitive,
        renderer::{EnvironmentData, RenderObjectBundle},
    },
    physics::{
        rapier3d::prelude::{RigidBodyType, SharedShape},
        ColliderBundle, PhysicsPlugin,
    },
    postprocessing::preconfigured::{SurfacePbrPipelineDesc, SurfacePbrRenderer},
    world_transform, App, RigidBodyBundle, TransformBundle,
};
use winit::{dpi::PhysicalSize, window::WindowAttributes};

pub fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();

    App::builder()
        .with_driver(WinitDriver::new(
            WindowAttributes::default().with_inner_size(PhysicalSize::new(1920, 1080)),
        ))
        .with_layer(EngineLayer::new())
        .with_layer(GraphicsLayer::new(|world, assets, store, gpu, surface| {
            Ok(SurfacePbrRenderer::new(
                world,
                assets,
                store,
                gpu,
                surface,
                SurfacePbrPipelineDesc::default(),
            ))
        }))
        .with_layer(InputLayer::new())
        .with_layer(
            ScheduledLayer::new(FixedTimeStep::new(0.02))
                .with_plugin(OrbitCameraPlugin)
                .with_plugin(RayPickingPlugin)
                .with_plugin(NavmeshPlugin::new(NavmeshSettings::default()))
                .with_plugin(NavmeshDebugPlugin)
                .with_plugin(ExamplePlugin)
                .with_plugin(PhysicsPlugin::new().with_gravity(Vec3::ZERO)),
        )
        .with_layer(ViewportCameraLayer::new(CameraSettings {
            environment_data: EnvironmentData {
                fog_color: Srgb::new(0.0, 0.1, 0.05),
                fog_density: 0.001,
                fog_blend: 1.0,
            },
            fov: 1.0,
        }))
        .run()
}

struct ExamplePlugin;

impl Plugin for ExamplePlugin {
    fn install(
        &self,
        world: &mut World,
        assets: &AssetCache,
        events: &mut ivy_engine::ivy_core::update_layer::ScheduleSetBuilder,
    ) -> anyhow::Result<()> {
        let brushes = [
            (
                TransformBundle::default(),
                Brush::cube().with_transform(Mat4::from_scale(vec3(10.0, 0.4, 10.0))),
            ),
            (
                TransformBundle::default().with_position(vec3(-10.0, 0.5, -10.0)),
                Brush::cube().with_transform(Mat4::from_scale(vec3(5.0, 0.4, 5.0))),
            ),
            (
                TransformBundle::default().with_position(vec3(-10.0, 0.0, -10.0)),
                Brush::cube().with_transform(Mat4::from_scale(vec3(1.0, 5.0, 1.0))),
            ),
            (
                TransformBundle::default().with_position(vec3(0.5, -0.2, -0.5)),
                Brush::cube(),
            ),
            (
                TransformBundle::default()
                    .with_position(vec3(3.0, 0.3, 4.0))
                    .with_rotation(Quat::from_axis_angle(Vec3::Y, 2.0)),
                Brush::cube().with_transform(Mat4::from_scale(vec3(4.0, 1.0, 2.0))),
            ),
            (
                TransformBundle::default()
                    .with_position(vec3(-3.0, 0.4, 4.0))
                    .with_rotation(Quat::from_axis_angle(Vec3::X, DEG_45)),
                Brush::cube(),
            ),
        ];

        Entity::builder()
            .mount(TransformBundle::default().with_rotation(Quat::from_axis_angle(Vec3::X, -1.0)))
            .mount(LightBundle {
                params: LightParams::new(Srgb::new(1.0, 1.0, 1.0), 1.0),
                kind: LightKind::Directional,
                cast_shadow: true,
            })
            .spawn(world);

        let start_point = Entity::builder()
            .mount(
                TransformBundle::default()
                    .with_position(vec3(8.0, 1.0, 8.0))
                    .with_scale(Vec3::ONE * 0.2),
            )
            .mount(RenderObjectBundle::new(
                MeshDesc::Content(assets.load(&UvSpherePrimitive::default())),
                &[(
                    forward_pass(),
                    MaterialData::PbrMaterial(
                        PbrMaterialData::new()
                            .with_albedo(TextureData::srgba(Srgba::new(0.0, 0.7, 0.2, 1.0))),
                    ),
                )],
            ))
            .mount(RigidBodyBundle::new(RigidBodyType::Dynamic).with_mass(1.0))
            .mount(ColliderBundle::new(SharedShape::ball(0.2)))
            .spawn(world);

        let end_point = Entity::builder()
            .mount(
                TransformBundle::default()
                    .with_position(vec3(5.0, 1.0, 0.0))
                    .with_scale(Vec3::ONE * 0.2),
            )
            .mount(RenderObjectBundle::new(
                MeshDesc::Content(assets.load(&UvSpherePrimitive::default())),
                &[(
                    forward_pass(),
                    MaterialData::PbrMaterial(
                        PbrMaterialData::new()
                            .with_albedo(TextureData::srgba(Srgba::new(0.7, 0.0, 0.2, 1.0))),
                    ),
                )],
            ))
            .mount(RigidBodyBundle::new(RigidBodyType::Dynamic).with_mass(1.0))
            .mount(ColliderBundle::new(SharedShape::ball(0.2)))
            .spawn(world);

        events
            .fixed_mut()
            .with_system(System::builder().with_world().build(move |world: &World| {
                let gizmos = world.get(engine(), gizmos())?;
                let Ok(navmesh) = world.get(engine(), navmesh()) else {
                    tracing::warn!("no navmesh");
                    return Ok(());
                };

                let mut gizmos = gizmos.begin_section("navmesh_example");
                let start_pos = world
                    .get(start_point, world_transform())?
                    .transform_point3(Vec3::ZERO);

                let end_pos = world
                    .get(end_point, world_transform())?
                    .transform_point3(Vec3::ZERO);

                if let Some((_, face)) = navmesh.closest_polygon(start_pos) {
                    for edge in face.edges() {
                        gizmos.draw(Line::from_points(
                            edge.0,
                            edge.1,
                            0.02,
                            Color::new(0.0, 0.5, 0.0, 1.0),
                        ));
                    }
                }

                if let Some((_, face)) = navmesh.closest_polygon(end_pos) {
                    for edge in face.edges() {
                        gizmos.draw(Line::from_points(edge.0, edge.1, 0.02, Color::red()));
                    }
                }

                let path = astar(&navmesh, start_pos, end_pos, |a, b| a.distance(b));

                let color = Color::new(0.5, 0.0, 0.0, 1.0);
                for (from, to) in path.iter().flatten().tuple_windows() {
                    gizmos.draw(Line::from_points(from.point(), to.point(), 0.04, color));

                    gizmos.draw(Sphere::new(to.point(), 0.08, color));
                }

                anyhow::Ok(())
            }));

        Ok(())
    }
}

fn brush_to_mesh(brush: &Brush) -> MeshData {
    let vertex_count = brush.faces().len() * 3;
    let mut mesh = MeshData::new()
        .with_attribute(
            POSITION_ATTRIBUTE,
            brush.faces().iter().flat_map(|v| v.points()),
        )
        .with_attribute(
            TEX_COORD_ATTRIBUTE,
            (0..vertex_count as u32).map(|_| Vec2::ZERO),
        )
        .with_attribute(
            NORMAL_ATTRIBUTE,
            brush.faces().iter().flat_map(|v| {
                let n = v.normal();
                [n, n, n]
            }),
        )
        .with_indices(0..vertex_count as u32);

    mesh.generate_tangents().unwrap();
    mesh
}
