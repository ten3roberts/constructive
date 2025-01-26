use csg_nav::brush::Brush;
use glam::{vec3, Mat4, Quat, Vec2, Vec3};
use ivy_engine::{
    flax::{Entity, World},
    input::layer::InputLayer,
    ivy_assets::AssetCache,
    ivy_core::{
        palette::{Srgb, Srgba},
        update_layer::{FixedTimeStep, Plugin, ScheduledLayer},
        EngineLayer, EntityBuilderExt,
    },
    ivy_game::{
        orbit_camera::OrbitCameraPlugin,
        viewport_camera::{CameraSettings, ViewportCameraLayer},
    },
    ivy_graphics::{
        mesh::{MeshData, NORMAL_ATTRIBUTE, POSITION_ATTRIBUTE, TEX_COORD_ATTRIBUTE},
        texture::{TextureData, TextureDesc},
    },
    ivy_ui::image,
    ivy_wgpu::{
        components::forward_pass,
        driver::WinitDriver,
        layer::GraphicsLayer,
        light::{LightBundle, LightKind, LightParams},
        material_desc::{MaterialData, PbrMaterialData},
        mesh_desc::MeshDesc,
        renderer::{EnvironmentData, RenderObjectBundle},
    },
    postprocessing::preconfigured::{SurfacePbrPipelineDesc, SurfacePbrRenderer},
    App, TransformBundle,
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
                .with_plugin(ExamplePlugin),
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
        _: &mut ivy_engine::ivy_core::update_layer::ScheduleSetBuilder,
    ) -> anyhow::Result<()> {
        let mut brush = Brush::plane();
        brush.transform(Mat4::from_scale(Vec3::splat(10.0)));

        let clip_brushes = [
            // Brush::cube().with_transform(Mat4::from_rotation_translation(
            //     Quat::from_axis_angle(Vec3::Z, 1.0),
            //     vec3(1.5, 0.0, 0.5),
            // )),
            // Brush::cube().with_transform(Mat4::from_rotation_translation(
            //     Quat::IDENTITY,
            //     vec3(0.5, 0.9, -0.5),
            // )),
            // Brush::cube().with_transform(Mat4::from_rotation_translation(
            //     Quat::from_axis_angle(Vec3::Y, 2.0),
            //     vec3(7.0, 0.5, 0.0),
            // )),
            Brush::cube().with_transform(Mat4::from_rotation_translation(
                Quat::IDENTITY,
                vec3(1.5, 0.0, 0.5),
            )),
            Brush::cube().with_transform(Mat4::from_rotation_translation(
                Quat::IDENTITY,
                vec3(0.5, 0.9, -2.5),
            )),
            Brush::cube().with_transform(Mat4::from_rotation_translation(
                Quat::IDENTITY,
                vec3(7.0, 0.5, 0.0),
            )),
        ];

        for clip in &clip_brushes {
            brush = clip.create_tree().clip_brush(brush);
        }

        let flat_material = PbrMaterialData::new();

        Entity::builder()
            .mount(TransformBundle::default())
            .mount(RenderObjectBundle::new(
                MeshDesc::Content(assets.insert(brush_to_mesh(&brush))),
                &[(
                    forward_pass(),
                    MaterialData::UnlitMaterial(flat_material.clone()),
                )],
            ))
            .spawn(world);

        for clip in clip_brushes {
            let flat_material = PbrMaterialData::new()
                .with_albedo(TextureData::srgba(Srgba::new(1.0, 0.0, 0.0, 1.0)));

            Entity::builder()
                .mount(TransformBundle::default())
                .mount(RenderObjectBundle::new(
                    MeshDesc::Content(assets.insert(brush_to_mesh(&clip))),
                    &[(
                        forward_pass(),
                        MaterialData::WireframeMaterial(flat_material),
                    )],
                ))
                .spawn(world);
        }

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
