use constructive::{
    brush::PositionedBrush,
    link::LinkKind,
    navmesh::{Navmesh, NavmeshSettings},
};
use flax::{components::child_of, entity::EntityKind};
use glam::{Mat4, Vec2};
use itertools::Itertools;
use ivy_engine::{
    engine,
    flax::{
        filter::ChangeFilter, system, BoxedSystem, CommandBuffer, Component, Entity, FetchExt,
        Query, QueryBorrow, System, World,
    },
    gizmos,
    ivy_assets::AssetCache,
    ivy_core::{
        gizmos::{Gizmos, Line},
        palette::Srgba,
        update_layer::Plugin,
        Color, ColorExt, EntityBuilderExt,
    },
    ivy_graphics::{
        mesh::{MeshData, NORMAL_ATTRIBUTE, POSITION_ATTRIBUTE, TEX_COORD_ATTRIBUTE},
        texture::TextureData,
    },
    ivy_wgpu::{
        components::forward_pass,
        material_desc::{MaterialData, PbrMaterialData},
        mesh_desc::MeshDesc,
        renderer::RenderObjectBundle,
    },
    world_transform, TransformBundle, TransformQuery,
};

use crate::components::{self, brushes, navmesh, navmesh_settings};

pub struct NavmeshPlugin {
    settings: NavmeshSettings,
}

impl NavmeshPlugin {
    pub fn new(settings: NavmeshSettings) -> Self {
        Self { settings }
    }
}

impl Plugin for NavmeshPlugin {
    fn install(
        &self,
        world: &mut World,
        _: &AssetCache,
        schedules: &mut ivy_engine::ivy_core::update_layer::ScheduleSetBuilder,
    ) -> anyhow::Result<()> {
        world.set(engine(), navmesh_settings(), self.settings)?;

        schedules
            .fixed_mut()
            .with_system(generate_navmesh_system())
            .flush();

        Ok(())
    }
}

// Visualizes the navmesh
pub struct NavmeshDebugPlugin;

impl Plugin for NavmeshDebugPlugin {
    fn install(
        &self,
        _: &mut World,
        assets: &AssetCache,
        schedules: &mut ivy_engine::ivy_core::update_layer::ScheduleSetBuilder,
    ) -> anyhow::Result<()> {
        let mut debug_entity = None;
        let assets = assets.clone();

        let generate_navmesh = System::builder()
            .with_cmd_mut()
            .with_world()
            .with_query(Query::new(navmesh().modified()))
            .build(
                move |cmd: &mut CommandBuffer,
                      world: &World,
                      mut query: QueryBorrow<ChangeFilter<Navmesh>>| {
                    for navmesh in &mut query {
                        if let Some(id) = debug_entity.take() {
                            cmd.despawn(id);
                        }

                        let id = world.reserve_one(EntityKind::empty());
                        debug_entity = Some(id);

                        cmd.append_to(id, Entity::builder().mount(TransformBundle::default()));

                        let flat_material = PbrMaterialData::new()
                            .with_albedo(TextureData::srgba(Srgba::new(0.5, 0.0, 0.5, 0.1)));

                        cmd.spawn(
                            Entity::builder()
                                .mount(TransformBundle::default())
                                .mount(RenderObjectBundle::new(
                                    MeshDesc::Content(assets.insert(navmesh_to_mesh(navmesh))),
                                    &[(
                                        forward_pass(),
                                        MaterialData::UnlitMaterial(flat_material.clone()),
                                    )],
                                ))
                                .set_default(child_of(id)),
                        );

                        let flat_material = PbrMaterialData::new()
                            .with_albedo(TextureData::srgba(Srgba::new(0.0, 0.5, 1.0, 0.5)));

                        cmd.spawn(
                            Entity::builder()
                                .mount(TransformBundle::default())
                                .mount(RenderObjectBundle::new(
                                    MeshDesc::Content(
                                        assets.insert(walkable_navmesh_to_mesh(navmesh)),
                                    ),
                                    &[(
                                        forward_pass(),
                                        MaterialData::PbrMaterial(flat_material.clone()),
                                    )],
                                ))
                                .set_default(child_of(id)),
                        );

                        let flat_material = PbrMaterialData::new()
                            .with_albedo(TextureData::srgba(Srgba::new(0.0, 0.5, 1.0, 1.0)));

                        let id = world.reserve_one(EntityKind::empty());
                        cmd.append_to(
                            id,
                            Entity::builder().mount(TransformBundle::default()).mount(
                                RenderObjectBundle::new(
                                    MeshDesc::Content(
                                        assets.insert(walkable_navmesh_to_mesh(navmesh)),
                                    ),
                                    &[(
                                        forward_pass(),
                                        MaterialData::WireframeMaterial(flat_material.clone()),
                                    )],
                                ),
                            ),
                        );
                    }
                },
            );

        schedules
            .per_tick_mut()
            .with_system(generate_navmesh)
            .with_system(navmesh_gizmos_system());

        Ok(())
    }
}

#[system(with_query(Query::new(navmesh())))]
fn navmesh_gizmos_system(gizmos: &Gizmos, query: &mut QueryBorrow<Component<Navmesh>>) {
    let mut gizmos = gizmos.begin_section("navmesh_gizmos");
    for navmesh in query {
        const LINE_THICKNESS: f32 = 0.005;
        for (_, link) in navmesh.links() {
            match link.kind() {
                LinkKind::Walk(_) => {}
                LinkKind::StepUp(bot, top) => {
                    let bot_line =
                        Line::from_points(bot.p1, bot.p2, LINE_THICKNESS, Color::orange());

                    let top_line =
                        Line::from_points(top.p1, top.p2, LINE_THICKNESS, Color::orange());

                    let support_1 =
                        Line::from_points(bot.p1, top.p1, LINE_THICKNESS, Color::orange());
                    let support_2 =
                        Line::from_points(bot.p2, top.p2, LINE_THICKNESS, Color::orange());

                    gizmos.draw(top_line);
                    gizmos.draw(bot_line);
                    gizmos.draw(support_1);
                    gizmos.draw(support_2);
                }
            }
        }
    }
}

fn walkable_navmesh_to_mesh(navmesh: &Navmesh) -> MeshData {
    let polygons = navmesh.walkable_polygons().collect_vec();
    let vertex_count = polygons.len() * 3;

    let mut mesh = MeshData::new()
        .with_attribute(
            POSITION_ATTRIBUTE,
            polygons.iter().flat_map(|(_, v)| v.points()),
        )
        .with_attribute(
            TEX_COORD_ATTRIBUTE,
            (0..vertex_count as u32).map(|_| Vec2::ZERO),
        )
        .with_attribute(
            NORMAL_ATTRIBUTE,
            polygons.iter().flat_map(|(_, v)| {
                let n = v.normal();
                [n, n, n]
            }),
        )
        .with_indices(0..vertex_count as u32);

    mesh.generate_tangents().unwrap();
    mesh
}

fn navmesh_to_mesh(navmesh: &Navmesh) -> MeshData {
    let polygons = navmesh.brush_polygons();
    let vertex_count = polygons.len() * 3;

    let mut mesh = MeshData::new()
        .with_attribute(POSITION_ATTRIBUTE, polygons.iter().flat_map(|v| v.points()))
        .with_attribute(
            TEX_COORD_ATTRIBUTE,
            (0..vertex_count as u32).map(|_| Vec2::ZERO),
        )
        .with_attribute(
            NORMAL_ATTRIBUTE,
            polygons.iter().flat_map(|v| {
                let n = v.normal();
                [n, n, n]
            }),
        )
        .with_indices(0..vertex_count as u32);

    mesh.generate_tangents().unwrap();
    mesh
}

fn generate_navmesh_system() -> BoxedSystem {
    System::builder()
        .with_query(Query::new(TransformQuery::new().modified()).with(brushes()))
        .with_query(Query::new((brushes(), world_transform())))
        .with_query(Query::new(navmesh_settings()))
        .with_cmd_mut()
        .build(
            |mut changed: QueryBorrow<_, _>,
             mut query: QueryBorrow<(Component<Vec<PositionedBrush>>, Component<Mat4>)>,
             mut settings: QueryBorrow<Component<NavmeshSettings>>,
             cmd: &mut CommandBuffer| {
                if changed.iter().next().is_none() {
                    return;
                }

                let brushes = query.iter().flat_map(|(brushes, &transform)| {
                    brushes.iter().map(move |v| {
                        PositionedBrush::new(transform * v.transform(), v.brush().clone())
                    })
                });

                let navmesh = Navmesh::new(
                    settings.get(engine()).ok().copied().unwrap_or_default(),
                    brushes,
                );

                cmd.set(engine(), components::navmesh(), navmesh);
            },
        )
        .boxed()
}
