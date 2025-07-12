#import "Basic";
#load "../core/collision_3d_shapes.jai";

test_sphere_sphere :: (a: Sphere, b: Sphere) -> CollisionInfo {
    info: CollisionInfo;
    
    distance_vec := b.center - a.center;
    distance_squared := length_squared(distance_vec);
    radius_sum := a.radius + b.radius;
    
    if distance_squared <= radius_sum * radius_sum {
        info.has_collision = true;
        distance := sqrt(distance_squared);
        
        if distance > EPSILON {
            info.normal = normalize(distance_vec);
            info.penetration_depth = radius_sum - distance;
        } else {
            info.normal = .{0, 1, 0};
            info.penetration_depth = radius_sum;
        }
        
        contact_point := a.center + info.normal * (a.radius - info.penetration_depth * 0.5);
        array_add(*info.contact_points, contact_point);
    }
    
    return info;
}

test_aabb_aabb :: (a: AABB, b: AABB) -> CollisionInfo {
    info: CollisionInfo;
    
    if a.min.x <= b.max.x && a.max.x >= b.min.x &&
       a.min.y <= b.max.y && a.max.y >= b.min.y &&
       a.min.z <= b.max.z && a.max.z >= b.min.z {
        
        info.has_collision = true;
        
        overlap_x := min(a.max.x, b.max.x) - max(a.min.x, b.min.x);
        overlap_y := min(a.max.y, b.max.y) - max(a.min.y, b.min.y);
        overlap_z := min(a.max.z, b.max.z) - max(a.min.z, b.min.z);
        
        if overlap_x <= overlap_y && overlap_x <= overlap_z {
            info.penetration_depth = overlap_x;
            center_a := (a.min + a.max) * 0.5;
            center_b := (b.min + b.max) * 0.5;
            info.normal = .{ifx center_a.x < center_b.x then -1.0 else 1.0, 0, 0};
        } else if overlap_y <= overlap_x && overlap_y <= overlap_z {
            info.penetration_depth = overlap_y;
            center_a := (a.min + a.max) * 0.5;
            center_b := (b.min + b.max) * 0.5;
            info.normal = .{0, ifx center_a.y < center_b.y then -1.0 else 1.0, 0};
        } else {
            info.penetration_depth = overlap_z;
            center_a := (a.min + a.max) * 0.5;
            center_b := (b.min + b.max) * 0.5;
            info.normal = .{0, 0, ifx center_a.z < center_b.z then -1.0 else 1.0};
        }
    }
    
    return info;
}

test_sphere_aabb :: (sphere: Sphere, aabb: AABB) -> CollisionInfo {
    info: CollisionInfo;
    
    closest := closest_point_on_aabb(sphere.center, aabb);
    distance_squared := length_squared(closest - sphere.center);
    
    if distance_squared <= sphere.radius * sphere.radius {
        info.has_collision = true;
        
        if distance_squared > EPSILON {
            distance := sqrt(distance_squared);
            info.normal = normalize(sphere.center - closest);
            info.penetration_depth = sphere.radius - distance;
        } else {
            info.normal = .{0, 1, 0};
            info.penetration_depth = sphere.radius;
        }
        
        array_add(*info.contact_points, closest);
    }
    
    return info;
}

test_obb_obb :: (a: OBB, b: OBB) -> CollisionInfo {
    info: CollisionInfo;
    
    axes: [15]Vector3;
    
    a_axes: [3]Vector3;
    a_axes[0] = quaternion_rotate_vector(a.orientation, .{1, 0, 0});
    a_axes[1] = quaternion_rotate_vector(a.orientation, .{0, 1, 0});
    a_axes[2] = quaternion_rotate_vector(a.orientation, .{0, 0, 1});
    
    b_axes: [3]Vector3;
    b_axes[0] = quaternion_rotate_vector(b.orientation, .{1, 0, 0});
    b_axes[1] = quaternion_rotate_vector(b.orientation, .{0, 1, 0});
    b_axes[2] = quaternion_rotate_vector(b.orientation, .{0, 0, 1});
    
    axes[0] = a_axes[0];
    axes[1] = a_axes[1];
    axes[2] = a_axes[2];
    axes[3] = b_axes[0];
    axes[4] = b_axes[1];
    axes[5] = b_axes[2];
    
    index := 6;
    for i: 0..2 {
        for j: 0..2 {
            axes[index] = cross(a_axes[i], b_axes[j]);
            if length_squared(axes[index]) > EPSILON {
                axes[index] = normalize(axes[index]);
            }
            index += 1;
        }
    }
    
    min_overlap := FLOAT32_MAX;
    min_axis: Vector3;
    
    for i: 0..14 {
        if length_squared(axes[i]) < EPSILON continue;
        
        a_proj := abs(dot(a_axes[0], axes[i])) * a.half_extents.x +
                  abs(dot(a_axes[1], axes[i])) * a.half_extents.y +
                  abs(dot(a_axes[2], axes[i])) * a.half_extents.z;
        
        b_proj := abs(dot(b_axes[0], axes[i])) * b.half_extents.x +
                  abs(dot(b_axes[1], axes[i])) * b.half_extents.y +
                  abs(dot(b_axes[2], axes[i])) * b.half_extents.z;
        
        distance := abs(dot(b.center - a.center, axes[i]));
        overlap := a_proj + b_proj - distance;
        
        if overlap < 0 return info;
        
        if overlap < min_overlap {
            min_overlap = overlap;
            min_axis = axes[i];
        }
    }
    
    info.has_collision = true;
    info.penetration_depth = min_overlap;
    
    if dot(b.center - a.center, min_axis) < 0 {
        info.normal = min_axis * -1;
    } else {
        info.normal = min_axis;
    }
    
    return info;
}

test_capsule_capsule :: (a: Capsule, b: Capsule) -> CollisionInfo {
    info: CollisionInfo;
    
    closest_a := closest_point_on_line_segment(a.point_a, b.point_a, b.point_b);
    closest_b := closest_point_on_line_segment(b.point_a, a.point_a, a.point_b);
    
    segment_a := closest_point_on_line_segment(closest_a, a.point_a, a.point_b);
    segment_b := closest_point_on_line_segment(closest_b, b.point_a, b.point_b);
    
    distance_vec := segment_b - segment_a;
    distance_squared := length_squared(distance_vec);
    radius_sum := a.radius + b.radius;
    
    if distance_squared <= radius_sum * radius_sum {
        info.has_collision = true;
        
        if distance_squared > EPSILON {
            distance := sqrt(distance_squared);
            info.normal = normalize(distance_vec);
            info.penetration_depth = radius_sum - distance;
        } else {
            info.normal = .{0, 1, 0};
            info.penetration_depth = radius_sum;
        }
        
        contact_point := segment_a + info.normal * (a.radius - info.penetration_depth * 0.5);
        array_add(*info.contact_points, contact_point);
    }
    
    return info;
}

gjk_support :: (mesh: *ConvexMesh, direction: Vector3) -> Vector3 {
    max_dot := -FLOAT32_MAX;
    support_point: Vector3;
    
    for vertex: mesh.vertices {
        d := dot(vertex, direction);
        if d > max_dot {
            max_dot = d;
            support_point = vertex;
        }
    }
    
    return support_point;
}

minkowski_support :: (mesh_a: *ConvexMesh, mesh_b: *ConvexMesh, direction: Vector3) -> Vector3 {
    return gjk_support(mesh_a, direction) - gjk_support(mesh_b, direction * -1);
}

Simplex :: struct {
    points: [4]Vector3;
    count: int;
}

gjk_update_simplex :: (simplex: *Simplex, direction: *Vector3) -> bool {
    if simplex.count == 2 {
        a := simplex.points[1];
        b := simplex.points[0];
        ab := b - a;
        ao := a * -1;
        
        if dot(ab, ao) > 0 {
            <<direction = cross(cross(ab, ao), ab);
        } else {
            simplex.points[0] = a;
            simplex.count = 1;
            <<direction = ao;
        }
    } else if simplex.count == 3 {
        a := simplex.points[2];
        b := simplex.points[1];
        c := simplex.points[0];
        
        ab := b - a;
        ac := c - a;
        ao := a * -1;
        
        abc := cross(ab, ac);
        
        if dot(cross(abc, ac), ao) > 0 {
            if dot(ac, ao) > 0 {
                simplex.points[0] = c;
                simplex.points[1] = a;
                simplex.count = 2;
                <<direction = cross(cross(ac, ao), ac);
            } else {
                return gjk_update_simplex_2d(simplex, direction, a, b, ab, ao);
            }
        } else {
            if dot(cross(ab, abc), ao) > 0 {
                return gjk_update_simplex_2d(simplex, direction, a, b, ab, ao);
            } else {
                if dot(abc, ao) > 0 {
                    <<direction = abc;
                } else {
                    temp := b;
                    simplex.points[1] = c;
                    simplex.points[0] = temp;
                    <<direction = abc * -1;
                }
            }
        }
    } else if simplex.count == 4 {
        a := simplex.points[3];
        b := simplex.points[2];
        c := simplex.points[1];
        d := simplex.points[0];
        
        ab := b - a;
        ac := c - a;
        ad := d - a;
        ao := a * -1;
        
        abc := cross(ab, ac);
        acd := cross(ac, ad);
        adb := cross(ad, ab);
        
        if dot(abc, ao) > 0 {
            simplex.points[0] = c;
            simplex.points[1] = b;
            simplex.points[2] = a;
            simplex.count = 3;
            return gjk_update_simplex(simplex, direction);
        }
        if dot(acd, ao) > 0 {
            simplex.points[0] = d;
            simplex.points[1] = c;
            simplex.points[2] = a;
            simplex.count = 3;
            return gjk_update_simplex(simplex, direction);
        }
        if dot(adb, ao) > 0 {
            simplex.points[0] = b;
            simplex.points[1] = d;
            simplex.points[2] = a;
            simplex.count = 3;
            return gjk_update_simplex(simplex, direction);
        }
        
        return true;
    }
    
    return false;
}

gjk_update_simplex_2d :: (simplex: *Simplex, direction: *Vector3, a: Vector3, b: Vector3, ab: Vector3, ao: Vector3) -> bool {
    if dot(ab, ao) > 0 {
        simplex.points[0] = b;
        simplex.points[1] = a;
        simplex.count = 2;
        <<direction = cross(cross(ab, ao), ab);
    } else {
        simplex.points[0] = a;
        simplex.count = 1;
        <<direction = ao;
    }
    return false;
}

test_convex_convex_gjk :: (mesh_a: *ConvexMesh, mesh_b: *ConvexMesh) -> bool {
    direction := Vector3.{1, 0, 0};
    simplex: Simplex;
    
    support := minkowski_support(mesh_a, mesh_b, direction);
    simplex.points[0] = support;
    simplex.count = 1;
    
    direction = support * -1;
    
    max_iterations := 64;
    for i: 0..max_iterations-1 {
        support = minkowski_support(mesh_a, mesh_b, direction);
        
        if dot(support, direction) < 0 {
            return false;
        }
        
        simplex.points[simplex.count] = support;
        simplex.count += 1;
        
        if gjk_update_simplex(*simplex, *direction) {
            return true;
        }
    }
    
    return false;
}