use bevy::prelude::*;
//  NOTE: This type alias is for clarity
use bevy_rapier2d::{geometry::Group as CollisionGroup, prelude::*};

#[allow(dead_code)]
#[derive(Debug, Default, Eq, PartialEq)]
enum LongitudinalDirection {
    Forwards,
    Backwards,
    #[default]
    Still,
}

#[derive(Debug, Default, Eq, PartialEq)]
enum HorizontalDirection {
    Left,
    Right,
    #[default]
    Still,
}

#[derive(Debug, Default, Eq, PartialEq)]
enum VerticalDirection {
    Up,
    Down,
    #[default]
    Still,
}

const PLAYER_COLLISION_GROUP: CollisionGroup = CollisionGroup::GROUP_1;
const PLATFORM_COLLISION_GROUP: CollisionGroup = CollisionGroup::GROUP_1;

const PLAYER_MOVEMENT_SPEED: f32 = 200.0;
const PLAYER_SPRINT_SPEED: f32 = 500.0;
const PLAYER_GRAVITY: f32 = 100.0;
const PLAYER_JUMP_VELOCITY: f32 = 100.0;
const PLAYER_INITIAL_POSITION: Vec2 = Vec2::ZERO;
const PLAYER_HALF_SIZE: Vec2 = Vec2::new(20.0, 40.0);
const PLAYER_SKIN_WIDTH: f32 = 0.015;
const PLAYER_PHYSICS_HALF_SIZE: Vec2 = Vec2::new(
    PLAYER_HALF_SIZE.x * (1.0 - PLAYER_SKIN_WIDTH),
    PLAYER_HALF_SIZE.y * (1.0 - PLAYER_SKIN_WIDTH),
);

const HORIZONTAL_RAYCAST_COUNT: u8 = 10;
const VERTICAL_RAYCAST_COUNT: u8 = 10;

const HORIZONTAL_RAYCAST_SPACING: f32 =
    2.0 * PLAYER_PHYSICS_HALF_SIZE.y / (HORIZONTAL_RAYCAST_COUNT - 1) as f32;
const VERTICAL_RAYCAST_SPACING: f32 =
    2.0 * PLAYER_PHYSICS_HALF_SIZE.x / (VERTICAL_RAYCAST_COUNT - 1) as f32;

struct Ray {
    origin: Vec2,
    direction: Vec2,
    magnitude: f32,
}

impl Ray {
    fn new(origin: Vec2, direction: Vec2, magnitude: f32) -> Self {
        Self {
            origin,
            direction,
            magnitude,
        }
    }
}

#[derive(Bundle)]
struct CuboidBundle {
    collider: Collider,
    collision_groups: CollisionGroups,
    rigidbody: RigidBody,
    transform: TransformBundle,
}

impl CuboidBundle {
    fn new(center: Vec2, half_size: Vec2, groups: Group, ignore_groups: Group) -> Self {
        CuboidBundle {
            collider: Collider::cuboid(half_size.x, half_size.y),
            collision_groups: CollisionGroups::new(groups, ignore_groups),
            rigidbody: RigidBody::KinematicPositionBased,
            transform: TransformBundle::from(Transform::from_xyz(center.x, center.y, 0.0)),
        }
    }
}

#[derive(Component)]
struct Platform;

#[derive(Bundle)]
struct PlatformBundle {
    platform: Platform,
    collider: CuboidBundle,
}

impl PlatformBundle {
    fn new(center: Vec2, half_size: Vec2) -> Self {
        PlatformBundle {
            platform: Platform,
            collider: CuboidBundle::new(
                center,
                half_size,
                PLATFORM_COLLISION_GROUP,
                CollisionGroup::NONE,
            ),
        }
    }
}

#[derive(Component, Default)]
struct Player {
    horizontal_direction: HorizontalDirection,
    vertical_direction: VerticalDirection,
    grounded: bool,
}

#[derive(Component, Deref, DerefMut)]
struct Velocity(Vec2);

#[allow(dead_code)]
#[derive(Component, Default)]
struct PlanarDirection {
    horizontal_direction: HorizontalDirection,
    vertical_direction: VerticalDirection,
}

#[derive(Bundle)]
struct PlayerBundle {
    player: Player,
    collider: CuboidBundle,
    velocity: Velocity,
    direction: PlanarDirection,
}

impl PlayerBundle {
    fn new(initial_position: Vec2, half_size: Vec2) -> Self {
        PlayerBundle {
            player: Player::default(),
            collider: CuboidBundle::new(
                initial_position,
                half_size,
                PLAYER_COLLISION_GROUP,
                PLAYER_COLLISION_GROUP,
            ),
            velocity: Velocity(Vec2::ZERO),
            direction: PlanarDirection::default(),
        }
    }
}

fn move_player(
    mut player: Query<(&mut Transform, &mut Velocity, &mut Player, Entity)>,
    time: Res<Time<Fixed>>,
    keys: Res<ButtonInput<KeyCode>>,
    rapier: Res<RapierContext>,
    mut gizmos: Gizmos,
) {
    use HorizontalDirection as HDir;
    use VerticalDirection as VDir;

    let (mut transform, mut velocity, mut player, player_handle) = match player.get_single_mut() {
        Ok(player) => player,
        Err(_) => panic!("ERROR: Multiple player instances"),
    };

    let current_position = transform.translation.xy();

    velocity.x = 0.0;

    if keys.pressed(KeyCode::KeyA) {
        velocity.x -= PLAYER_MOVEMENT_SPEED;
    }

    if keys.pressed(KeyCode::KeyD) {
        velocity.x += PLAYER_MOVEMENT_SPEED;
    }

    if keys.pressed(KeyCode::ShiftLeft) {
        velocity.x = PLAYER_SPRINT_SPEED * velocity.x.signum();
    }

    if velocity.x < 0.0 {
        player.horizontal_direction = HDir::Left;
    } else if velocity.x > 0.0 {
        player.horizontal_direction = HDir::Right;
    }

    let mut delta_velocity_x = velocity.x * time.delta_seconds();

    for ray in 0..HORIZONTAL_RAYCAST_COUNT {
        let mut origin_color = Color::BLUE;
        let mut raycast_color = Color::GREEN;

        let raycast_length = if delta_velocity_x.abs() > PLAYER_SKIN_WIDTH {
            delta_velocity_x.abs()
        } else {
            PLAYER_SKIN_WIDTH
        };

        let raycast = match player.horizontal_direction {
            HDir::Left => Ray::new(
                current_position - PLAYER_PHYSICS_HALF_SIZE
                    + Vec2::Y * ray as f32 * HORIZONTAL_RAYCAST_SPACING,
                Vec2::NEG_X,
                raycast_length,
            ),
            HDir::Right => Ray::new(
                current_position
                    + PLAYER_PHYSICS_HALF_SIZE
                    + Vec2::NEG_Y * ray as f32 * HORIZONTAL_RAYCAST_SPACING,
                Vec2::X,
                raycast_length,
            ),
            HDir::Still => continue,
        };

        if let Some((_, hit)) = rapier.cast_ray_and_get_normal(
            raycast.origin,
            raycast.direction,
            raycast.magnitude,
            true,
            QueryFilter::default().exclude_collider(player_handle),
        ) {
            delta_velocity_x =
                (hit.time_of_impact - PLAYER_SKIN_WIDTH) * raycast.direction.x.signum();

            origin_color = Color::ORANGE_RED;
            raycast_color = Color::RED;
        }

        gizmos.ray_2d(
            raycast.origin,
            raycast.direction * raycast.magnitude * 5.0,
            raycast_color,
        );

        gizmos.circle_2d(raycast.origin, 2.0, origin_color);
    }

    player.grounded = false;

    if keys.just_pressed(KeyCode::Space) {
        velocity.y = PLAYER_JUMP_VELOCITY;
    }

    velocity.y -= PLAYER_GRAVITY * time.delta_seconds();

    if velocity.y < 0.0 {
        player.vertical_direction = VerticalDirection::Down;
    } else if velocity.y > 0.0 {
        player.vertical_direction = VerticalDirection::Up;
    }

    let mut delta_velocity_y = velocity.y * time.delta_seconds();

    for ray in 0..VERTICAL_RAYCAST_COUNT {
        let mut origin_color = Color::BLUE;
        let mut raycast_color = Color::GREEN;

        let raycast_length = if delta_velocity_y.abs() > PLAYER_SKIN_WIDTH {
            delta_velocity_y.abs()
        } else {
            PLAYER_SKIN_WIDTH
        };

        let raycast = match player.vertical_direction {
            VDir::Down => Ray::new(
                current_position - PLAYER_PHYSICS_HALF_SIZE
                    + Vec2::X * ray as f32 * VERTICAL_RAYCAST_SPACING,
                Vec2::NEG_Y,
                raycast_length,
            ),
            VDir::Up => Ray::new(
                current_position
                    + PLAYER_PHYSICS_HALF_SIZE
                    + Vec2::NEG_X * ray as f32 * VERTICAL_RAYCAST_SPACING,
                Vec2::Y,
                raycast_length,
            ),
            VDir::Still => continue,
        };

        if let Some((_, hit)) = rapier.cast_ray_and_get_normal(
            raycast.origin,
            raycast.direction,
            raycast.magnitude,
            true,
            QueryFilter::default().exclude_collider(player_handle),
        ) {
            delta_velocity_y =
                (hit.time_of_impact - PLAYER_SKIN_WIDTH) * raycast.direction.y.signum();

            origin_color = Color::ORANGE_RED;
            raycast_color = Color::RED;
            player.grounded = true;
        }

        gizmos.ray_2d(
            raycast.origin,
            raycast.direction * raycast.magnitude * 5.0,
            raycast_color,
        );

        gizmos.circle_2d(raycast.origin, 2.0, origin_color);
    }

    velocity.x = delta_velocity_x / time.delta_seconds();
    velocity.y = delta_velocity_y / time.delta_seconds();

    transform.translation += velocity.extend(0.0) * time.delta_seconds();
}

fn setup_world(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
    commands.spawn(PlatformBundle::new(
        Vec2::new(0.0, -100.0),
        Vec2::new(200.0, 30.0),
    ));
    commands.spawn(PlatformBundle::new(
        Vec2::new(-150.0, 100.0),
        Vec2::new(10.0, 90.0),
    ));
    commands.spawn(PlatformBundle::new(
        Vec2::new(150.0, 100.0),
        Vec2::new(10.0, 90.0),
    ));
    commands.spawn(PlayerBundle::new(PLAYER_INITIAL_POSITION, PLAYER_HALF_SIZE));
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(50.0))
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_world)
        .add_systems(Update, bevy::window::close_on_esc)
        .add_systems(FixedUpdate, move_player)
        .run();
}
