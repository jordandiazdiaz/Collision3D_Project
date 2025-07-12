#import "Basic";
#load "broadphase/collision_3d_broadphase.jai";
#load "core/collision_3d_detection.jai";

CollisionWorld :: struct {
    colliders: [..]Collider;
    static_colliders: [..]int;
    dynamic_colliders: [..]int;
    
    broadphase_bvh: *BVH;
    broadphase_octree: *Octree;
    broadphase_grid: *SpatialHashGrid;
    
    use_bvh: bool = true;
    use_continuous_detection: bool = false;
    
    collision_pairs: [..]BroadphasePair;
    contact_manifolds: [..]ContactManifold;
}

ContactManifold :: struct {
    collider_a: int;
    collider_b: int;
    contact_count: int;
    contacts: [4]ContactPoint;
    normal: Vector3;
}

ContactPoint :: struct {
    position: Vector3;
    penetration: float;
    impulse: float;
}

RigidBody :: struct {
    mass: float;
    inverse_mass: float;
    inertia: Matrix3;
    inverse_inertia: Matrix3;
    
    position: Vector3;
    orientation: Quaternion;
    linear_velocity: Vector3;
    angular_velocity: Vector3;
    
    force: Vector3;
    torque: Vector3;
    
    restitution: float = 0.2;
    friction: float = 0.5;
}

Matrix3 :: struct {
    m: [3][3] float;
}

create_collision_world :: () -> *CollisionWorld {
    world := New(CollisionWorld);
    world.broadphase_bvh = create_bvh();
    return world;
}

add_collider :: (world: *CollisionWorld, collider: Collider, is_static: bool = false) -> int {
    id := world.colliders.count;
    array_add(*world.colliders, collider);
    
    if is_static {
        array_add(*world.static_colliders, id);
    } else {
        array_add(*world.dynamic_colliders, id);
    }
    
    aabb := get_collider_aabb(*collider);
    if world.use_bvh {
        bvh_insert(world.broadphase_bvh, aabb, id);
    }
    
    return id;
}

get_collider_aabb :: (collider: *Collider) -> AABB {
    transform_matrix := matrix4_from_transform(collider.transform);
    
    if collider.type == {
        case .SPHERE;
            world_center := transform_point(transform_matrix, collider.sphere.center);
            world_radius := collider.sphere.radius * max(collider.transform.scale.x, 
                                                         max(collider.transform.scale.y, 
                                                             collider.transform.scale.z));
            return aabb_from_sphere(.{world_center, world_radius});
            
        case .AABB;
            corners: [8]Vector3;
            corners[0] = .{collider.aabb.min.x, collider.aabb.min.y, collider.aabb.min.z};
            corners[1] = .{collider.aabb.max.x, collider.aabb.min.y, collider.aabb.min.z};
            corners[2] = .{collider.aabb.min.x, collider.aabb.max.y, collider.aabb.min.z};
            corners[3] = .{collider.aabb.max.x, collider.aabb.max.y, collider.aabb.min.z};
            corners[4] = .{collider.aabb.min.x, collider.aabb.min.y, collider.aabb.max.z};
            corners[5] = .{collider.aabb.max.x, collider.aabb.min.y, collider.aabb.max.z};
            corners[6] = .{collider.aabb.min.x, collider.aabb.max.y, collider.aabb.max.z};
            corners[7] = .{collider.aabb.max.x, collider.aabb.max.y, collider.aabb.max.z};
            
            for i: 0..7 {
                corners[i] = transform_point(transform_matrix, corners[i]);
            }
            return aabb_from_points(corners);
            
        case .OBB;
            world_obb: OBB;
            world_obb.center = transform_point(transform_matrix, collider.obb.center);
            world_obb.half_extents = collider.obb.half_extents;
            world_obb.half_extents.x *= collider.transform.scale.x;
            world_obb.half_extents.y *= collider.transform.scale.y;
            world_obb.half_extents.z *= collider.transform.scale.z;
            world_obb.orientation = quaternion_multiply(collider.transform.rotation, collider.obb.orientation);
            return aabb_from_obb(world_obb);
            
        case .CAPSULE;
            world_capsule: Capsule;
            world_capsule.point_a = transform_point(transform_matrix, collider.capsule.point_a);
            world_capsule.point_b = transform_point(transform_matrix, collider.capsule.point_b);
            world_capsule.radius = collider.capsule.radius * max(collider.transform.scale.x,
                                                                max(collider.transform.scale.y,
                                                                    collider.transform.scale.z));
            
            min_point := Vector3.{
                min(world_capsule.point_a.x, world_capsule.point_b.x) - world_capsule.radius,
                min(world_capsule.point_a.y, world_capsule.point_b.y) - world_capsule.radius,
                min(world_capsule.point_a.z, world_capsule.point_b.z) - world_capsule.radius
            };
            max_point := Vector3.{
                max(world_capsule.point_a.x, world_capsule.point_b.x) + world_capsule.radius,
                max(world_capsule.point_a.y, world_capsule.point_b.y) + world_capsule.radius,
                max(world_capsule.point_a.z, world_capsule.point_b.z) + world_capsule.radius
            };
            return .{min_point, max_point};
            
        case .CONVEX_MESH;
            transformed_vertices: [..]Vector3;
            array_reserve(*transformed_vertices, collider.convex_mesh.vertices.count);
            for vertex: collider.convex_mesh.vertices {
                array_add(*transformed_vertices, transform_point(transform_matrix, vertex));
            }
            result := aabb_from_points(transformed_vertices);
            array_free(transformed_vertices);
            return result;
    }
    
    return .{};
}

update_broadphase :: (world: *CollisionWorld) {
    array_clear(*world.collision_pairs);
    
    if world.use_bvh {
        bvh_query_pairs(world.broadphase_bvh, *world.collision_pairs);
    }
}

test_collider_pair :: (a: *Collider, b: *Collider) -> CollisionInfo {
    if a.type == .SPHERE && b.type == .SPHERE {
        return test_sphere_sphere(
            transform_sphere(a.sphere, a.transform),
            transform_sphere(b.sphere, b.transform)
        );
    } else if a.type == .AABB && b.type == .AABB {
        return test_aabb_aabb(
            transform_aabb(a.aabb, a.transform),
            transform_aabb(b.aabb, b.transform)
        );
    } else if a.type == .SPHERE && b.type == .AABB {
        return test_sphere_aabb(
            transform_sphere(a.sphere, a.transform),
            transform_aabb(b.aabb, b.transform)
        );
    } else if a.type == .AABB && b.type == .SPHERE {
        info := test_sphere_aabb(
            transform_sphere(b.sphere, b.transform),
            transform_aabb(a.aabb, a.transform)
        );
        info.normal = info.normal * -1;
        return info;
    } else if a.type == .OBB && b.type == .OBB {
        return test_obb_obb(
            transform_obb(a.obb, a.transform),
            transform_obb(b.obb, b.transform)
        );
    } else if a.type == .CAPSULE && b.type == .CAPSULE {
        return test_capsule_capsule(
            transform_capsule(a.capsule, a.transform),
            transform_capsule(b.capsule, b.transform)
        );
    } else if a.type == .CONVEX_MESH && b.type == .CONVEX_MESH {
        info: CollisionInfo;
        info.has_collision = test_convex_convex_gjk(a.convex_mesh, b.convex_mesh);
        return info;
    }
    
    return .{};
}

transform_sphere :: (sphere: Sphere, transform: Transform) -> Sphere {
    matrix := matrix4_from_transform(transform);
    return .{
        center = transform_point(matrix, sphere.center),
        radius = sphere.radius * max(transform.scale.x, max(transform.scale.y, transform.scale.z))
    };
}

transform_aabb :: (aabb: AABB, transform: Transform) -> AABB {
    matrix := matrix4_from_transform(transform);
    corners: [8]Vector3;
    corners[0] = .{aabb.min.x, aabb.min.y, aabb.min.z};
    corners[1] = .{aabb.max.x, aabb.min.y, aabb.min.z};
    corners[2] = .{aabb.min.x, aabb.max.y, aabb.min.z};
    corners[3] = .{aabb.max.x, aabb.max.y, aabb.min.z};
    corners[4] = .{aabb.min.x, aabb.min.y, aabb.max.z};
    corners[5] = .{aabb.max.x, aabb.min.y, aabb.max.z};
    corners[6] = .{aabb.min.x, aabb.max.y, aabb.max.z};
    corners[7] = .{aabb.max.x, aabb.max.y, aabb.max.z};
    
    for i: 0..7 {
        corners[i] = transform_point(matrix, corners[i]);
    }
    
    return aabb_from_points(corners);
}

transform_obb :: (obb: OBB, transform: Transform) -> OBB {
    matrix := matrix4_from_transform(transform);
    return .{
        center = transform_point(matrix, obb.center),
        half_extents = .{
            obb.half_extents.x * transform.scale.x,
            obb.half_extents.y * transform.scale.y,
            obb.half_extents.z * transform.scale.z
        },
        orientation = quaternion_multiply(transform.rotation, obb.orientation)
    };
}

transform_capsule :: (capsule: Capsule, transform: Transform) -> Capsule {
    matrix := matrix4_from_transform(transform);
    return .{
        point_a = transform_point(matrix, capsule.point_a),
        point_b = transform_point(matrix, capsule.point_b),
        radius = capsule.radius * max(transform.scale.x, max(transform.scale.y, transform.scale.z))
    };
}

detect_collisions :: (world: *CollisionWorld) {
    update_broadphase(world);
    array_clear(*world.contact_manifolds);
    
    for pair: world.collision_pairs {
        collider_a := *world.colliders[pair.a];
        collider_b := *world.colliders[pair.b];
        
        info := test_collider_pair(collider_a, collider_b);
        
        if info.has_collision {
            manifold: ContactManifold;
            manifold.collider_a = pair.a;
            manifold.collider_b = pair.b;
            manifold.normal = info.normal;
            manifold.contact_count = min(info.contact_points.count, 4);
            
            for i: 0..manifold.contact_count-1 {
                manifold.contacts[i].position = info.contact_points[i];
                manifold.contacts[i].penetration = info.penetration_depth;
            }
            
            array_add(*world.contact_manifolds, manifold);
        }
    }
}

ray_cast :: (world: *CollisionWorld, ray: Ray, max_distance: float = FLOAT32_MAX) -> RaycastHit {
    best_hit: RaycastHit;
    best_hit.distance = max_distance;
    
    for i: 0..world.colliders.count-1 {
        collider := *world.colliders[i];
        hit := ray_cast_collider(ray, collider, max_distance);
        
        if hit.hit && hit.distance < best_hit.distance {
            best_hit = hit;
        }
    }
    
    return best_hit;
}

ray_cast_collider :: (ray: Ray, collider: *Collider, max_distance: float) -> RaycastHit {
    hit: RaycastHit;
    
    if collider.type == .SPHERE {
        world_sphere := transform_sphere(collider.sphere, collider.transform);
        hit = ray_cast_sphere(ray, world_sphere, max_distance);
    } else if collider.type == .AABB {
        world_aabb := transform_aabb(collider.aabb, collider.transform);
        hit = ray_cast_aabb(ray, world_aabb, max_distance);
    }
    
    return hit;
}

ray_cast_sphere :: (ray: Ray, sphere: Sphere, max_distance: float) -> RaycastHit {
    hit: RaycastHit;
    
    oc := ray.origin - sphere.center;
    a := dot(ray.direction, ray.direction);
    b := 2.0 * dot(oc, ray.direction);
    c := dot(oc, oc) - sphere.radius * sphere.radius;
    
    discriminant := b * b - 4 * a * c;
    if discriminant < 0 return hit;
    
    sqrt_discriminant := sqrt(discriminant);
    t1 := (-b - sqrt_discriminant) / (2 * a);
    t2 := (-b + sqrt_discriminant) / (2 * a);
    
    t := t1;
    if t < 0 t = t2;
    if t < 0 || t > max_distance return hit;
    
    hit.hit = true;
    hit.distance = t;
    hit.point = ray.origin + ray.direction * t;
    hit.normal = normalize(hit.point - sphere.center);
    
    return hit;
}

ray_cast_aabb :: (ray: Ray, aabb: AABB, max_distance: float) -> RaycastHit {
    hit: RaycastHit;
    
    inv_dir := Vector3.{1.0 / ray.direction.x, 1.0 / ray.direction.y, 1.0 / ray.direction.z};
    
    t1 := (aabb.min.x - ray.origin.x) * inv_dir.x;
    t2 := (aabb.max.x - ray.origin.x) * inv_dir.x;
    t3 := (aabb.min.y - ray.origin.y) * inv_dir.y;
    t4 := (aabb.max.y - ray.origin.y) * inv_dir.y;
    t5 := (aabb.min.z - ray.origin.z) * inv_dir.z;
    t6 := (aabb.max.z - ray.origin.z) * inv_dir.z;
    
    tmin := max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
    tmax := min(min(max(t1, t2), max(t3, t4)), max(t5, t6));
    
    if tmax < 0 || tmin > tmax || tmin > max_distance return hit;
    
    hit.hit = true;
    hit.distance = ifx tmin < 0 then tmax else tmin;
    hit.point = ray.origin + ray.direction * hit.distance;
    
    epsilon := 0.0001;
    if abs(hit.point.x - aabb.min.x) < epsilon hit.normal = .{-1, 0, 0};
    else if abs(hit.point.x - aabb.max.x) < epsilon hit.normal = .{1, 0, 0};
    else if abs(hit.point.y - aabb.min.y) < epsilon hit.normal = .{0, -1, 0};
    else if abs(hit.point.y - aabb.max.y) < epsilon hit.normal = .{0, 1, 0};
    else if abs(hit.point.z - aabb.min.z) < epsilon hit.normal = .{0, 0, -1};
    else if abs(hit.point.z - aabb.max.z) < epsilon hit.normal = .{0, 0, 1};
    
    return hit;
}