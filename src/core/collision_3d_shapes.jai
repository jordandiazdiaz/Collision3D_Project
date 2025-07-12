#import "Basic";
#load "../core/collision_3d_math.jai";

ColliderType :: enum {
    SPHERE;
    AABB;
    OBB;
    CAPSULE;
    CONVEX_MESH;
}

CollisionInfo :: struct {
    has_collision: bool;
    penetration_depth: float;
    normal: Vector3;
    contact_points: [..]Vector3;
}

Sphere :: struct {
    center: Vector3;
    radius: float;
}

AABB :: struct {
    min: Vector3;
    max: Vector3;
}

OBB :: struct {
    center: Vector3;
    half_extents: Vector3;
    orientation: Quaternion;
}

Capsule :: struct {
    point_a: Vector3;
    point_b: Vector3;
    radius: float;
}

Plane :: struct {
    normal: Vector3;
    distance: float;
}

Triangle :: struct {
    a, b, c: Vector3;
}

ConvexMesh :: struct {
    vertices: [..]Vector3;
    faces: [..]Triangle;
    planes: [..]Plane;
}

Ray :: struct {
    origin: Vector3;
    direction: Vector3;
}

RaycastHit :: struct {
    hit: bool;
    distance: float;
    point: Vector3;
    normal: Vector3;
}

Collider :: struct {
    type: ColliderType;
    transform: Transform;
    union {
        sphere: Sphere;
        aabb: AABB;
        obb: OBB;
        capsule: Capsule;
        convex_mesh: *ConvexMesh;
    }
}

aabb_from_points :: (points: []Vector3) -> AABB {
    if points.count == 0 return .{};
    
    result: AABB;
    result.min = points[0];
    result.max = points[0];
    
    for i: 1..points.count-1 {
        result.min.x = min(result.min.x, points[i].x);
        result.min.y = min(result.min.y, points[i].y);
        result.min.z = min(result.min.z, points[i].z);
        
        result.max.x = max(result.max.x, points[i].x);
        result.max.y = max(result.max.y, points[i].y);
        result.max.z = max(result.max.z, points[i].z);
    }
    
    return result;
}

aabb_from_sphere :: (sphere: Sphere) -> AABB {
    return .{
        min = sphere.center - Vector3.{sphere.radius, sphere.radius, sphere.radius},
        max = sphere.center + Vector3.{sphere.radius, sphere.radius, sphere.radius}
    };
}

aabb_from_obb :: (obb: OBB) -> AABB {
    corners: [8]Vector3;
    
    x_axis := quaternion_rotate_vector(obb.orientation, .{1, 0, 0}) * obb.half_extents.x;
    y_axis := quaternion_rotate_vector(obb.orientation, .{0, 1, 0}) * obb.half_extents.y;
    z_axis := quaternion_rotate_vector(obb.orientation, .{0, 0, 1}) * obb.half_extents.z;
    
    corners[0] = obb.center - x_axis - y_axis - z_axis;
    corners[1] = obb.center + x_axis - y_axis - z_axis;
    corners[2] = obb.center - x_axis + y_axis - z_axis;
    corners[3] = obb.center + x_axis + y_axis - z_axis;
    corners[4] = obb.center - x_axis - y_axis + z_axis;
    corners[5] = obb.center + x_axis - y_axis + z_axis;
    corners[6] = obb.center - x_axis + y_axis + z_axis;
    corners[7] = obb.center + x_axis + y_axis + z_axis;
    
    return aabb_from_points(corners);
}

merge_aabb :: (a: AABB, b: AABB) -> AABB {
    return .{
        min = .{min(a.min.x, b.min.x), min(a.min.y, b.min.y), min(a.min.z, b.min.z)},
        max = .{max(a.max.x, b.max.x), max(a.max.y, b.max.y), max(a.max.z, b.max.z)}
    };
}

aabb_surface_area :: (aabb: AABB) -> float {
    extent := aabb.max - aabb.min;
    return 2.0 * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);
}

closest_point_on_line_segment :: (point: Vector3, a: Vector3, b: Vector3) -> Vector3 {
    ab := b - a;
    t := dot(point - a, ab) / dot(ab, ab);
    t = clamp(t, 0.0, 1.0);
    return a + ab * t;
}

closest_point_on_aabb :: (point: Vector3, aabb: AABB) -> Vector3 {
    return .{
        clamp(point.x, aabb.min.x, aabb.max.x),
        clamp(point.y, aabb.min.y, aabb.max.y),
        clamp(point.z, aabb.min.z, aabb.max.z)
    };
}

closest_point_on_obb :: (point: Vector3, obb: OBB) -> Vector3 {
    local_point := point - obb.center;
    
    x_axis := quaternion_rotate_vector(obb.orientation, .{1, 0, 0});
    y_axis := quaternion_rotate_vector(obb.orientation, .{0, 1, 0});
    z_axis := quaternion_rotate_vector(obb.orientation, .{0, 0, 1});
    
    x := clamp(dot(local_point, x_axis), -obb.half_extents.x, obb.half_extents.x);
    y := clamp(dot(local_point, y_axis), -obb.half_extents.y, obb.half_extents.y);
    z := clamp(dot(local_point, z_axis), -obb.half_extents.z, obb.half_extents.z);
    
    return obb.center + x_axis * x + y_axis * y + z_axis * z;
}

distance_point_to_plane :: (point: Vector3, plane: Plane) -> float {
    return dot(point, plane.normal) - plane.distance;
}

project_polygon_on_axis :: (vertices: []Vector3, axis: Vector3) -> float, float {
    min_proj := dot(vertices[0], axis);
    max_proj := min_proj;
    
    for i: 1..vertices.count-1 {
        proj := dot(vertices[i], axis);
        min_proj = min(min_proj, proj);
        max_proj = max(max_proj, proj);
    }
    
    return min_proj, max_proj;
}

intervals_overlap :: (min1: float, max1: float, min2: float, max2: float) -> bool {
    return min1 <= max2 && min2 <= max1;
}

create_convex_mesh :: (vertices: []Vector3, indices: []int) -> *ConvexMesh {
    mesh := New(ConvexMesh);
    
    array_reserve(*mesh.vertices, vertices.count);
    for v: vertices array_add(*mesh.vertices, v);
    
    for i: 0..indices.count-1 {
        if i % 3 == 2 {
            tri: Triangle;
            tri.a = vertices[indices[i-2]];
            tri.b = vertices[indices[i-1]];
            tri.c = vertices[indices[i]];
            array_add(*mesh.faces, tri);
            
            edge1 := tri.b - tri.a;
            edge2 := tri.c - tri.a;
            normal := normalize(cross(edge1, edge2));
            distance := dot(normal, tri.a);
            
            plane: Plane;
            plane.normal = normal;
            plane.distance = distance;
            array_add(*mesh.planes, plane);
        }
    }
    
    return mesh;
}